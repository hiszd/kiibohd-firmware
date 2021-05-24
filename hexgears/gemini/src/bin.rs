// Copyright 2021 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

// TODO Remove this
#![allow(clippy::inconsistent_struct_constructor)]
#![no_std]
#![no_main]

use const_env::from_env;
use core::panic::PanicInfo;
use cortex_m_rt::exception;
use kiibohd_log::{log, Logger};
use kiibohd_usb::HidCountryCode;
use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

use gemini::{
    controller::*,
    hal::{
        clock::{ClockController, MainClock, SlowClock},
        gpio::*,
        pac::Peripherals,
        prelude::*,
        rtt::RealTimeTimer,
        time::duration::Extensions,
        udp::{
            usb_device,
            usb_device::{
                bus::UsbBusAllocator,
                device::{UsbDeviceBuilder, UsbVidPid},
            },
            UdpBus,
        },
        watchdog::Watchdog,
        ToggleableOutputPin,
    },
    Pins,
};

#[from_env]
const VID: u16 = 0x1c11;
#[from_env]
const PID: u16 = 0xb04d;
#[from_env]
const USB_MANUFACTURER: &'static str = "Unknown";
#[from_env]
const USB_PRODUCT: &'static str = "Kiibohd";

// Define static lifetimes for USB
type UsbDevice = usb_device::device::UsbDevice<'static, UdpBus>;
type HidInterface = kiibohd_usb::HidInterface<'static, UdpBus>;

static LOGGER: Logger = Logger::new(log::LevelFilter::Trace);

#[app(device = gemini::hal::pac, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    //
    // Resources used by tasks/interrupts
    //
    struct Resources {
        debug_led: Pb0<Output<PushPull>>,
        wdt: Watchdog,
        rtt: RealTimeTimer,
        rtt_host: rtt_target::DownChannel,
        usb_dev: UsbDevice,
        usb_hid: HidInterface,
    }

    //
    // Initialization
    //
    #[init(schedule = [key_scan])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // TODO once_cell?
        static mut USB_BUS: Option<UsbBusAllocator<UdpBus>> = None;

        // XXX (HaaTa): Fix this in the bootloader if possible!
        unsafe { cx.core.SCB.vtor.write(0x6000) };

        // Setup RTT logging
        //let channels = rtt_target::rtt_init_default!();
        let channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 1024
                    mode: BlockIfFull
                    name: "Terminal"
                }
            }
            down: {
                0: {
                    size: 16
                    name: "Terminal"
                }
            }
        };
        rtt_target::set_print_channel(channels.up.0);
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Trace);
        log::info!(">>>> Initializing <<<<");

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Setup main and slow clocks
        let peripherals = Peripherals::take().unwrap();
        log::trace!("Clock initialization");
        let clocks = ClockController::new(
            peripherals.PMC,
            &peripherals.SUPC,
            &peripherals.EFC0,
            MainClock::Crystal12Mhz,
            SlowClock::RcOscillator32Khz,
        );
        log::trace!("Clock initialized");

        // Setup gpios
        let gpio_ports = Ports::new(
            (
                peripherals.PIOA,
                clocks.peripheral_clocks.pio_a.into_enabled_clock(),
            ),
            (
                peripherals.PIOB,
                clocks.peripheral_clocks.pio_b.into_enabled_clock(),
            ),
        );
        let pins = Pins::new(gpio_ports, &peripherals.MATRIX);

        // Prepare watchdog to be fed
        let mut wdt = Watchdog::new(peripherals.WDT);
        wdt.feed();
        log::trace!("Watchdog first feed");

        // Setup USB
        *USB_BUS = Some(UsbBusAllocator::<UdpBus>::new(UdpBus::new(
            peripherals.UDP,
            clocks.peripheral_clocks.udp,
            pins.udp_ddm,
            pins.udp_ddp,
        )));
        let usb_bus = USB_BUS.as_ref().unwrap();
        let usb_hid = HidInterface::new(usb_bus, HidCountryCode::NotSupported);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer(USB_MANUFACTURER)
            .max_packet_size_0(64)
            .max_power(500)
            .product(USB_PRODUCT)
            .supports_remote_wakeup(true) // TODO Add support
            .build();
        //.serial_number(&USB_SERIAL) // TODO how to store and format string
        //.device_release() // TODO Get git revision info (sequential commit number)

        // Setup main timer (TODO May want to use a TC timer instead and reserve this for sleeping)
        let mut rtt = RealTimeTimer::new(peripherals.RTT, 3, false);
        rtt.start(1_000_000u32.microseconds());
        rtt.enable_alarm_interrupt();
        log::trace!("RTT Timer started");

        // Schedule tasks
        cx.schedule.key_scan(cx.start).unwrap();
        log::trace!("All tasks scheduled");

        init::LateResources {
            debug_led: pins.debug_led,
            wdt,
            rtt,
            rtt_host: channels.down.0,
            usb_dev,
            usb_hid,
        }
    }

    //
    // LED Blink Task
    #[task(binds = RTT, resources = [rtt, debug_led])]
    fn rtt(cx: rtt::Context) {
        cx.resources.rtt.clear_interrupt_flags();
        cx.resources.debug_led.toggle().ok();
        log::info!("Blink");
    }

    /// Keyscanning Task
    /// High-priority scheduled tasks as consistency is more important than speed for scanning
    /// key states
    /// Scans one strobe at a time
    #[task(schedule = [key_scan], spawn = [macro_process], priority = 14)]
    fn key_scan(cx: key_scan::Context) {
        // TODO Only schedule on result
        //if unsafe { Scan_periodic() } != 0
        /*
        {
            if cx.spawn.macro_process().is_err() {
                log::warn!("Could not schedule macro_process");
            }
        }
        */

        /*
        if cx
            .schedule
            .key_scan(Instant::now() + 4800000.cycles())
            .is_err()
        {
            log::warn!("Could not schedule key_scan");
        }
        */
    }

    /// Macro Processing Task
    /// Handles incoming key scan triggers and turns them into results (actions and hid events)
    #[task(spawn = [usb_process])]
    fn macro_process(cx: macro_process::Context) {
        // TODO Enable
        //unsafe { Macro_periodic() };
        /*
        if cx.spawn.usb_process().is_err() {
            log::warn!("Could not schedule usb_process");
        }
        */
    }

    /// USB Outgoing Events Task
    /// Sends outgoing USB HID events generated by the macro_process task
    #[task(resources = [usb_hid])]
    fn usb_process(mut cx: usb_process::Context) {
        // TODO Enable
        //unsafe { Output_periodic() };
        cx.resources.usb_hid.lock(|usb_hid| {
            //usb_hid.push();
        });
    }

    /// Background polling loop
    /// Used to handle misc background tasks
    /// Scheduled tightly and at a low priority
    #[idle(resources = [rtt_host, wdt])]
    fn idle(cx: idle::Context) -> ! {
        // Initialize controller
        controller_setup();
        log::trace!("controller_setup done");

        // TODO (HaaTa): This should be tuned
        //               Eventually each of these polling tasks should be split out
        //               but this will likely have to wait until the tasks are converted
        //               to Rust.

        loop {
            // TODO Cleanup
            //unsafe {
            /*
            // Gather RTT input and send to kiibohd/controller CLI module
            let input = &mut *cx.resources.rtt_host;
            let mut buf = [0u8; 16];
            let count = input.read(&mut buf[..]);
            CLI_pushInput(buf.as_ptr(), count as u8);

            // Process CLI
            CLI_process();
            */

            // Macro module poll routines
            //Macro_poll();
            //}

            // Not locked up, reset watchdog
            cx.resources.wdt.feed();
        }
    }

    #[task(binds = TWI0, priority = 12)]
    fn twi0(_: twi0::Context) {
        //unsafe { TWI0_Handler() };
    }

    #[task(binds = TWI1, priority = 12)]
    fn twi1(_: twi1::Context) {
        //unsafe { TWI1_Handler() };
    }

    #[task(binds = UART0, priority = 15)]
    fn uart0(_: uart0::Context) {
        //unsafe { UART0_Handler() };
    }

    #[task(binds = UDP, priority = 13, resources = [usb_dev, usb_hid])]
    fn udp(cx: udp::Context) {
        let usb_dev = cx.resources.usb_dev;
        let usb_hid = cx.resources.usb_hid;
        // Poll USB endpoints
        if usb_dev.poll(&mut usb_hid.interfaces()) {
            usb_hid.poll();
        }
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn UART1();
        fn USART0();
        fn USART1();
        fn SPI();
        fn SSC();
        fn TC0(); // Timer Module 0, Channel 0
        fn TC1(); // Timer Module 0, Channel 1
        fn TC2(); // Timer Module 0, Channel 2
        fn ADC();
        fn PWM();
        fn ACC();
    }
};

#[exception]
fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    log::trace!("HF!");
    loop {}
    panic!("HardFault!");
}

fn controller_setup() {
    /*
    unsafe {
        //Latency_init();
        //CLI_init();

        // TODO Periodic function

        //Storage_init();
        //Macro_setup();

        //storage_load_settings();
    }
    */
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("Panic! {}", info);
    // TODO Handle restart in non-debug mode
    loop {}
}
