// Copyright 2021 Jacob Alexander, Zion Koyl
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

// Error in the segger_rtt section 95% certain
// Writing a buffer(not sure which one) over necessary data
// TODO determine starting memory address in the heap
// TODO how to watch/monitor memory in GDB
// Likely what is happening is size is not defined correctly for something in unsafe code


// TODO Remove this
#![allow(clippy::inconsistent_struct_constructor)]
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::exception;
use embedded_time::duration::*;
use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};
use rtt_target::{rprintln, rtt_init_default, set_print_channel};
//use kiibohd_core::keyscanning::Scan;
use atsam4_hal::InputPin;
use generic_array::typenum::{U17, U6};
use keyberon::impl_heterogenous_array;
use kiibohd_keyscanning::matrix::{Matrix, StateReturn, State};

use gemini::{
    controller::*,
    hal::{
        clock::{get_master_clock_frequency, ClockController, MainClock, SlowClock},
        gpio::*,
        pac::Peripherals,
        prelude::*,
        rtt::*,
        watchdog::Watchdog,
        OutputPin,
    },
    Pins,
};

struct KiibohdLogger {
    level_filter: log::LevelFilter,
}

impl KiibohdLogger {
    pub const fn new(level_filter: log::LevelFilter) -> KiibohdLogger {
        KiibohdLogger { level_filter }
    }
}

impl log::Log for KiibohdLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        self.level_filter.ge(&metadata.level())
    }

    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            let color = match record.level() {
                log::Level::Error => "1;5;31",
                log::Level::Warn => "1;33",
                log::Level::Info => "1;32",
                log::Level::Debug => "1;35",
                log::Level::Trace => "1;90",
            };
            rprintln!(
                "\x1b[{}m{}\x1b[0m - {}",
                color,
                record.level(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

static LOGGER: KiibohdLogger = KiibohdLogger::new(log::LevelFilter::Trace);

pub struct Rows(
    pub Pa26<Input<PullDown>>,
    pub Pa25<Input<PullDown>>,
    pub Pa24<Input<PullDown>>,
    pub Pa13<Input<PullDown>>,
    pub Pa14<Input<PullDown>>,
    pub Pa31<Input<PullDown>>,
);
impl_heterogenous_array! {
    Rows,
    dyn InputPin<Error = ()>,
    U6,
    [0, 1, 2, 3, 4, 5]
}

pub struct Cols(
    pub Pb1<Output<PushPull>>,
    pub Pb2<Output<PushPull>>,
    pub Pb3<Output<PushPull>>,
    pub Pa18<Output<PushPull>>,
    pub Pa19<Output<PushPull>>,
    pub Pa23<Output<PushPull>>,
    pub Pa20<Output<PushPull>>,
    pub Pa11<Output<PushPull>>,
    pub Pa8<Output<PushPull>>,
    pub Pa7<Output<PushPull>>,
    pub Pa6<Output<PushPull>>,
    pub Pa5<Output<PushPull>>,
    pub Pa27<Output<PushPull>>,
    pub Pa28<Output<PushPull>>,
    pub Pa29<Output<PushPull>>,
    pub Pa30<Output<PushPull>>,
    pub Pa2<Output<PushPull>>,
);
impl_heterogenous_array! {
    Cols,
    dyn OutputPin<Error = ()>,
    U17,
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
}

#[app(device = gemini::hal::pac, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    //
    // Resources used by tasks/interrupts
    //
    struct Resources {
        debug_led: Pb0<Output<PushPull>>,
        wdt: Watchdog,
        rtt_host: rtt_target::DownChannel,
        rtt: RealTimeTimer,
        scan_periodic: Matrix<Cols, Rows>,
    }

    //
    // Initialization
    //
    #[init(schedule = [key_scan])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // XXX (HaaTa): Fix this in the bootloader if possible!
        unsafe { cx.core.SCB.vtor.write(0x6000) };
        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        let channels = rtt_init_default!();
        rtt_target::set_print_channel(channels.up.0);
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Trace);
        log::info!(">>>> Initializing <<<<");

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

        // Setup RTT
        let mut rtt = RealTimeTimer::new(peripherals.RTT, 3, false);
        rtt.start(100_000_u32.microseconds());
        rtt.enable_alarm_interrupt();
        log::trace!("RTT initialized");

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


        // Configure GPIO pins for sense and strobe
        //TODO Remove dead code after testing
        //let rows = RowArray::new([&pins.sense1, &pins.sense2, &pins.sense3, &pins.sense4, &pins.sense5, &pins.sense6], 6);
        //let cols = ColArray::new([&mut pins.strobe1, &mut pins.strobe2, &mut pins.strobe3, &mut pins.strobe4, &mut pins.strobe5, &mut pins.strobe6, &mut pins.strobe7, &mut pins.strobe8, &mut pins.strobe9, &mut pins.strobe10, &mut pins.strobe11, &mut pins.strobe12, &mut pins.strobe13, &mut pins.strobe14, &mut pins.strobe15, &mut pins.strobe16, &mut pins.strobe17],17);
        let scan = Matrix::new(
            Cols(
                pins.strobe1,
                pins.strobe2,
                pins.strobe3,
                pins.strobe4,
                pins.strobe5,
                pins.strobe6,
                pins.strobe7,
                pins.strobe8,
                pins.strobe9,
                pins.strobe10,
                pins.strobe11,
                pins.strobe12,
                pins.strobe13,
                pins.strobe14,
                pins.strobe15,
                pins.strobe16,
                pins.strobe17,
            ),
            Rows(
                pins.sense1,
                pins.sense2,
                pins.sense3,
                pins.sense4,
                pins.sense5,
                pins.sense6,
            ),
            100_000_u32.microseconds(),
        );

        // Prepare watchdog to be fed
        let mut wdt = Watchdog::new(peripherals.WDT);
        wdt.feed();
        log::trace!("Watchdog first feed");

        // Initialize controller
        controller_setup();
        log::trace!("controller_setup done");

        // Schedule tasks
        cx.schedule.key_scan(cx.start).unwrap();

        // Task scheduling
        /*
        cx.schedule
            .blink_led(cx.start + get_master_clock_frequency().0.cycles())
            .unwrap();*/
        log::trace!("All tasks scheduled");

        init::LateResources {
            debug_led: pins.debug_led,
            wdt,
            rtt_host: channels.down.0,
            rtt: rtt,
            scan_periodic: scan.unwrap(),
        }
    }

    #[task(binds = RTT, resources = [scan_periodic, rtt, debug_led])]
    fn rtt(cx: rtt::Context) {
        cx.resources.rtt.clear_interrupt_flags();
        /*
        unsafe {
            static mut state: bool = false;

            if state == false {
                cx.resources.debug_led.set_low().ok();
                state = true;
            } else {
                cx.resources.debug_led.set_high().ok();
                state = false;
            }
            log::debug!("Blink");
        }*/
        cx.resources.scan_periodic.get(|state: StateReturn, i: usize, j: usize, high: bool| {
            if state.state_change == true {
                log::debug!("{:?} ({}, {}), {}", state.ending_state, i, j, high);
            }
        }).unwrap();
    }

    //
    // LED Blink Task
    //
    /*#[task(resources = [debug_led], schedule = [blink_led])]
    fn blink_led(cx: blink_led::Context) {
        static mut STATE: bool = false;

        if !(*STATE) {
            cx.resources.debug_led.set_low().ok();
            cx.schedule
                .blink_led(Instant::now() + (get_master_clock_frequency().0 / 20).cycles())
                .unwrap();
            *STATE = true;
        } else {
            cx.resources.debug_led.set_high().ok();
            cx.schedule
                .blink_led(Instant::now() + (get_master_clock_frequency().0 / 2).cycles())
                .unwrap();
            *STATE = false;
        }
    }*/

    /// Keyscanning Task
    /// High-priority scheduled tasks as consistency is more important than speed for scanning
    /// key states
    /// Scans one strobe at a time
    #[task(schedule = [key_scan], spawn = [macro_process], priority = 14)]
    fn key_scan(cx: key_scan::Context) {
        // TODO Only schedule on result
        //if unsafe { Scan_periodic() } != 0
        {
            if cx.spawn.macro_process().is_err() {
                log::warn!("Could not schedule macro_process");
            }
        }

        if cx
            .schedule
            .key_scan(Instant::now() + 48000.cycles())
            .is_err()
        {
            log::warn!("Could not schedule key_scan");
        }
    }

    /// Macro Processing Task
    /// Handles incoming key scan triggers and turns them into results (actions and hid events)
    #[task(spawn = [usb_process], priority = 14)]
    fn macro_process(cx: macro_process::Context) {
        // TODO Enable
        //unsafe { Macro_periodic() };
        if cx.spawn.usb_process().is_err() {
            log::warn!("Could not schedule usb_process");
        }
    }

    /// USB Outgoing Events Task
    /// Sends outgoing USB HID events generated by the macro_process task
    #[task(priority = 14)]
    fn usb_process(_cx: usb_process::Context) {
        // TODO Enable
        //unsafe { Output_periodic() };
    }

    /// Background polling loop
    /// Used to handle misc background tasks
    /// Scheduled tightly and at a low priority
    #[idle(resources = [rtt_host, wdt])]
    fn idle(cx: idle::Context) -> ! {
        // TODO (HaaTa): This should be tuned
        //               Eventually each of these polling tasks should be split out
        //               but this will likely have to wait until the tasks are converted
        //               to Rust.

        loop {
            // TODO Cleanup
            /*unsafe {
                // Gather RTT input and send to kiibohd/controller CLI module
                let input = &mut *cx.resources.rtt_host;
                let mut buf = [0u8; 16];
                let count = input.read(&mut buf[..]);
                CLI_pushInput(buf.as_ptr(), count as u8);

                // Process CLI
                CLI_process();

                // Scan module poll routines
                //Scan_poll();

                // Macro module poll routines
                //Macro_poll();

                // Output module poll routines
                //Output_poll();
            }*/

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

    #[task(binds = UDP, priority = 13)]
    fn udp(_: udp::Context) {
        //unsafe { UDP_Handler() };
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
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("{:?}", ef);
}

fn controller_setup() {
    unsafe {
        //Latency_init();
        //CLI_init();

        // TODO Periodic function

        //Storage_init();
        //Output_setup();
        //Macro_setup();
        //Scan_setup();

        //storage_load_settings();
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    panic!("Panic! {}", info);
}
