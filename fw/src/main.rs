#![no_main]
#![no_std]

use core::fmt::Write;
use embedded_hal::digital::v2::{ToggleableOutputPin};
use panic_rtt_target as _;
use rtic::app;
use rtic::cyccnt::U32Ext as CyccntU32Ext;
use rtt_target::{rtt_init_print, rprintln};
use self::{
    hal::gpio::{gpiod, Output, PushPull},
};
use smoltcp::{
    iface::{EthernetInterfaceBuilder, Neighbor, NeighborCache},
    socket::{SocketHandle, SocketSetItem, TcpSocket, TcpSocketBuffer},
    time::Instant,
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};
use stm32_eth::{
    {EthPins, PhyAddress, RingEntry, RxDescriptor, TxDescriptor},
    hal::gpio::GpioExt,
    hal::rcc::RccExt,
    hal::time::U32Ext as TimeU32Ext,
    //stm32::{interrupt, CorePeripherals, Peripherals, SYST},
};
use stm32f4xx_hal as hal;

type Led = gpiod::PD13<Output<PushPull>>;
type Eth = stm32_eth::Eth<'static, 'static>;
type EthernetInterface = smoltcp::iface::EthernetInterface<'static, 'static, 'static, &'static mut Eth>;
type SocketSet = smoltcp::socket::SocketSet<'static, 'static, 'static>;

/// The cycles per second of the CPU.
const CYCLE_HZ: u32 = 168_000_000;
const ONE_SEC: u32 = CYCLE_HZ;
const ONE_MS: u32 = ONE_SEC / 1_000;
const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];


struct RttLogger {}

static mut LOGGER: RttLogger = RttLogger {};

impl log::Log for RttLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= log::Level::Trace
    }

    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            rprintln!("[{}] {}", record.level(), record.args());
        }
    }
    fn flush(&self) {}
}


#[app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led: Led,
        #[init(0)]
        now_ms: u32,
        eth_iface: EthernetInterface,
        sockets: SocketSet,
        server_handle: SocketHandle,
    }

    #[init(schedule = [blink, every_ms])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // Enable debugging via RTT.
        rtt_init_print!();

        // Enable logging for smoltcp.
        unsafe {
            log::set_logger(&LOGGER).unwrap();
        }
        log::set_max_level(log::LevelFilter::Info);

        // Enable cycle counter for scheduler.
        cx.core.DWT.enable_cycle_counter();

        // Setup clocks.
        rprintln!("Setup clocks");
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(CYCLE_HZ.hz()).freeze();

        // Setup ethernet.
        rprintln!("Setup ethernet");
        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();
        let eth_pins = EthPins {
            ref_clk: gpioa.pa1,
            md_io: gpioa.pa2,
            md_clk: gpioc.pc1,
            crs: gpioa.pa7,
            tx_en: gpiob.pb11,
            tx_d0: gpiob.pb12,
            tx_d1: gpiob.pb13,
            rx_d0: gpioc.pc4,
            rx_d1: gpioc.pc5,
        };
        let eth = {
            static mut RX_RING: Option<[RingEntry<RxDescriptor>; 8]> = None;
            static mut TX_RING: Option<[RingEntry<TxDescriptor>; 2]> = None;
            static mut ETH: Option<Eth> = None;
            unsafe {
                RX_RING = Some(Default::default());
                TX_RING = Some(Default::default());
                rprintln!("Creating ethernet");
                let eth = Eth::new(
                    cx.device.ETHERNET_MAC,
                    cx.device.ETHERNET_DMA,
                    &mut RX_RING.as_mut().unwrap()[..],
                    &mut TX_RING.as_mut().unwrap()[..],
                    PhyAddress::_0,
                    clocks,
                    eth_pins,
                ).unwrap();
                rprintln!("Created ethernet");
                ETH = Some(eth);
                ETH.as_mut().unwrap()
            }
        };
        rprintln!("Enabling interrupt");
        eth.enable_interrupt();

        // Setup TCP/IP.
        rprintln!("Setup TCP/IP");
        let local_addr = Ipv4Address::new(10, 0, 0, 1);
        let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
        let (ip_addrs, neighbor_storage) = {
            static mut IP_ADDRS: Option<[IpCidr; 1]> = None;
            // TODO: Doesn't need outer Option
            static mut NEIGHBOR_STORAGE: Option<[Option<(IpAddress, Neighbor)>; 16]> = None;
            unsafe {
                IP_ADDRS = Some([ip_addr]);
                NEIGHBOR_STORAGE = Some([None; 16]);
                (IP_ADDRS.as_mut().unwrap(), NEIGHBOR_STORAGE.as_mut().unwrap())
            }
        };
        let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
        let ethernet_addr = EthernetAddress(SRC_MAC);
        let eth_iface = EthernetInterfaceBuilder::new(eth)
            .ethernet_addr(ethernet_addr)
            .ip_addrs(&mut ip_addrs[..])
            .neighbor_cache(neighbor_cache)
            .finalize();
        let (server_socket, mut sockets) = {
            static mut RX_BUFFER: [u8; 2048] = [0; 2048];
            static mut TX_BUFFER: [u8; 2048] = [0; 2048];
            static mut SOCKETS_STORAGE: [Option<SocketSetItem>; 2] = [None, None];
            unsafe {
                let server_socket = TcpSocket::new(
                    TcpSocketBuffer::new(&mut RX_BUFFER[..]),
                    TcpSocketBuffer::new(&mut TX_BUFFER[..]),
                );
                let sockets = SocketSet::new(&mut SOCKETS_STORAGE[..]);
                (server_socket, sockets)
            }
        };
        let server_handle = sockets.add(server_socket);

        // Setup LED.
        let gpiod = cx.device.GPIOD.split();
        let led = gpiod.pd13.into_push_pull_output();

        // Schedule the `blink` and `every_ms` tasks.
        cx.schedule.blink(cx.start + ONE_SEC.cycles()).unwrap();
        cx.schedule.every_ms(cx.start + ONE_MS.cycles()).unwrap();

        rprintln!("Run!");
        init::LateResources { led, eth_iface, sockets, server_handle }
    }

    #[task(resources = [led, now_ms, eth_iface, sockets, server_handle], schedule = [blink])]
    fn blink(mut cx: blink::Context) {
        let r = &mut cx.resources;
        r.led.toggle().unwrap();
        //rprintln!("Blink at {} ms", r.now_ms);
        cx.schedule.blink(cx.scheduled + CYCLE_HZ.cycles()).unwrap();
        poll_eth_iface(r.eth_iface, r.sockets, *r.server_handle, *r.now_ms);
    }

    #[task(resources = [now_ms], schedule = [every_ms])]
    fn every_ms(mut cx: every_ms::Context) {
        let r = &mut cx.resources;
        *r.now_ms = r.now_ms.wrapping_add(1);
        cx.schedule.every_ms(cx.scheduled + ONE_MS.cycles()).unwrap();
    }

    #[task(binds = ETH, resources = [eth_iface, now_ms, sockets, server_handle])]
    fn eth(mut cx: eth::Context) {
        rprintln!("ETH called!");
        let r = &mut cx.resources;
        // Clear interrupt flags.
        r.eth_iface.device_mut().interrupt_handler();
        poll_eth_iface(r.eth_iface, r.sockets, *r.server_handle, *r.now_ms);
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            core::sync::atomic::spin_loop_hint();
        }
    }

    extern "C" {
        fn EXTI0();
    }
};

fn poll_eth_iface(
    iface: &mut EthernetInterface,
    sockets: &mut SocketSet,
    server_handle: SocketHandle,
    now_ms: u32,
) {
    let now = Instant::from_millis(now_ms as i64);
    match iface.poll(sockets, now) {
        // Ignore malformed packets
        Err(e) => {
            rprintln!("Error: {:?}", e);
            return;
        }
        Ok(false) => return,
        Ok(true) => (),
    }

    let mut socket = sockets.get::<TcpSocket>(server_handle);
    if !socket.is_open() {
        if let Err(e) = socket.listen(80) {
            rprintln!("TCP listen error: {:?}", e);
        }
    }

    if socket.can_send() {
        if let Err(e) = write!(socket, "hello\n") {
            rprintln!("TCP send error: {:?}", e);
            return;
        }
        socket.close();
    }
}

// fn configure_rcc(rcc: &stm32f4xx_hal::stm32::RCC, syscfg: &stm32f4xx_hal::stm32::SYSCFG) {
//     rcc.apb2enr.write(|w| w.syscfgen().set_bit());
//
//     if rcc.ahb1enr.read().ethmacen().bit_is_set() {
//         // pmc must be changed with the ethernet controller disabled or under reset
//         rcc.ahb1enr.write(|w| w.ethmacen().clear_bit());
//     }
//
//     // 0 = MII, 1 = RMII.
//     syscfg.pmc.write(|w| w.mii_rmii_sel().set_bit());
//
//     // Enable ethernet clocks.
//     rcc.ahb1enr.write(|w| w.ethmacen().set_bit());
//     rcc.ahb1enr.write(|w| w.ethmactxen().set_bit());
//     rcc.ahb1enr.write(|w| w.ethmacrxen().set_bit());
//
//     // Reset pulse.
//     rcc.ahb1rstr.write(|w| w.ethmacrst().reset());
//     rcc.ahb1rstr.write(|w| w.ethmacrst().clear_bit());
// }
