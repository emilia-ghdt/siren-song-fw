#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::future::poll_fn;
use core::mem;
use core::sync::atomic::{compiler_fence, AtomicBool, Ordering};
use core::task::Poll;

use defmt::{info, unwrap, warn};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{AnyPin, Flex, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::pac::{self, comp, comp::psel::PSEL_A, lpcomp, COMP};
use embassy_nrf::saadc::{self, Input as SaadcInput};
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, interrupt, peripherals, spim, usb, Peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};

use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    COMP_LPCOMP => InterruptHandler;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<peripherals::SPI2>;
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
});

/// Interrupt handler.
pub struct InterruptHandler {
    _private: (),
}

static SUSPENDED: AtomicBool = AtomicBool::new(false);

impl interrupt::typelevel::Handler<interrupt::typelevel::COMP_LPCOMP> for InterruptHandler {
    unsafe fn on_interrupt() {
        let r = unsafe { &*COMP::ptr() };

        if r.events_ready.read().bits() != 0 {
            r.intenclr.write(|w| w.ready().clear());
            WAKER.wake();
        }
    }
}

static WAKER: AtomicWaker = AtomicWaker::new();

const NUM_COL_PINS: usize = 3;
const NUM_ROW_PINS: usize = 3;
const LED_MAP: [[[usize; 2]; NUM_COL_PINS]; NUM_ROW_PINS] = [
    [[5, 4], [3, 2], [1, 18]],
    [[10, 9], [8, 7], [6, 17]],
    [[15, 14], [13, 12], [11, 16]],
];

const KEY_MAP: [[[u8; 2]; NUM_COL_PINS]; NUM_ROW_PINS] = [
    [
        [b'q' - b'a' + 0x04, b'w' - b'a' + 0x04],
        [b'f' - b'a' + 0x04, b'p' - b'a' + 0x04],
        [b'b' - b'a' + 0x04, b'1' - b'1' + 0x1e],
    ],
    [
        [b'a' - b'a' + 0x04, b'r' - b'a' + 0x04],
        [b's' - b'a' + 0x04, b't' - b'a' + 0x04],
        [b'g' - b'a' + 0x04, 0x2c],
    ],
    [
        [b'z' - b'a' + 0x04, b'x' - b'a' + 0x04],
        [b'c' - b'a' + 0x04, b'd' - b'a' + 0x04],
        [b'v' - b'a' + 0x04, b'3' - b'1' + 0x1e],
    ],
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let p: Peripherals = embassy_nrf::init(Default::default());

    let clock: pac::CLOCK = unsafe { mem::transmute(()) };

    info!("Enabling ext hfosc...");
    clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while clock.events_hfclkstarted.read().bits() != 1 {}

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("HID keyboard example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.supports_remote_wakeup = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);

    // Build the builder.
    let mut usb = builder.build();

    let remote_wakeup: Signal<CriticalSectionRawMutex, _> = Signal::new();

    // Run the USB device.
    let usb_fut = async {
        loop {
            usb.run_until_suspend().await;
            match select(usb.wait_resume(), remote_wakeup.wait()).await {
                Either::First(()) => {}
                Either::Second(()) => {
                    unwrap!(usb.remote_wakeup().await);
                }
            }
        }
    };

    let (reader, mut writer) = hid.split();

    let out_fut = async {
        reader.run(false, &mut request_handler).await;
    };

    let mut led = Output::new(p.P0_26, Level::High, OutputDrive::Standard);

    let mut rows: [AnalogPin; NUM_ROW_PINS] = [p.P0_02.into(), p.P0_04.into(), p.P0_05.into()];

    let mut cols: [AnalogPin; NUM_COL_PINS] = [p.P0_29.into(), p.P0_03.into(), p.P0_28.into()];

    let mut led_en_pin = Output::new(p.P1_00.degrade(), Level::High, OutputDrive::Standard);
    led_en_pin.set_high();
    let led_mosi_pin = p.P0_16.degrade();
    let led_unused0 = p.P0_15.degrade();
    let led_unused1 = p.P0_19.degrade();

    let mut buf0 = [[[false; 2]; NUM_COL_PINS]; NUM_ROW_PINS];
    let mut buf1 = [[[false; 2]; NUM_COL_PINS]; NUM_ROW_PINS];
    let mut any_key = false;

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M4;

    let mut spim = spim::Spim::new(p.SPI2, Irqs, led_unused0, led_unused1, led_mosi_pin, config);

    let main_loop = async {
        loop {
            for _ in 0..1000 {
                let sample_matrix_future = sample_matrix(&mut rows, &mut cols, &mut buf0);
                let write_leds_future = write_leds_lazy(&mut spim, &buf1, any_key, &mut led_en_pin);
                (_, any_key) = join(sample_matrix_future, write_leds_future).await;

                let mut keycodes_index = 0;
                let mut keycodes_buf = [0u8; 6];
                for (i, row) in buf0.iter().enumerate() {
                    for (j, scan_pair) in row.iter().enumerate() {
                        for (k, key) in scan_pair.iter().enumerate() {
                            if keycodes_index >= 6 {
                                break;
                            }
                            if *key {
                                keycodes_buf[keycodes_index] = KEY_MAP[i][j][k].into();
                                keycodes_index += 1;
                            }
                        }
                        if keycodes_index >= 6 {
                            break;
                        }
                    }
                    if keycodes_index >= 6 {
                        break;
                    }
                }
                if keycodes_index >= 1 || any_key {
                    let report = KeyboardReport {
                        keycodes: keycodes_buf,
                        leds: 0,
                        modifier: 0,
                        reserved: 0,
                    };
                    match writer.write_serialize(&report).await {
                        Ok(()) => {}
                        Err(e) => warn!("Failed to send report: {:?}", e),
                    };
                }

                let sample_matrix_future = sample_matrix(&mut rows, &mut cols, &mut buf1);
                let write_leds_future = write_leds_lazy(&mut spim, &buf0, any_key, &mut led_en_pin);
                (_, any_key) = join(sample_matrix_future, write_leds_future).await;

                let mut keycodes_index = 0;
                let mut keycodes_buf = [0u8; 6];
                for (i, row) in buf1.iter().enumerate() {
                    for (j, scan_pair) in row.iter().enumerate() {
                        for (k, key) in scan_pair.iter().enumerate() {
                            if keycodes_index >= 6 {
                                break;
                            }
                            if *key {
                                keycodes_buf[keycodes_index] = KEY_MAP[i][j][k].into();
                                keycodes_index += 1;
                            }
                        }
                        if keycodes_index >= 6 {
                            break;
                        }
                    }
                    if keycodes_index >= 6 {
                        break;
                    }
                }
                if keycodes_index >= 1 || any_key {
                    let report = KeyboardReport {
                        keycodes: keycodes_buf,
                        leds: 0,
                        modifier: 0,
                        reserved: 0,
                    };
                    match writer.write_serialize(&report).await {
                        Ok(()) => {}
                        Err(e) => warn!("Failed to send report: {:?}", e),
                    };
                }
            }
            // info!("sampled");
            led.set_low();
            Timer::after_millis(100).await;
            led.set_high();
        }
    };
    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, join(main_loop, out_fut)).await;
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        SUSPENDED.store(false, Ordering::Release);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!("Device configured, it may now draw up to the configured current limit from Vbus.")
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }

    fn suspended(&mut self, suspended: bool) {
        if suspended {
            info!("Device suspended, the Vbus current limit is 500µA (or 2.5mA for high-power devices with remote wakeup enabled).");
            SUSPENDED.store(true, Ordering::Release);
        } else {
            SUSPENDED.store(false, Ordering::Release);
            if self.configured.load(Ordering::Relaxed) {
                info!("Device resumed, it may now draw up to the configured current limit from Vbus");
            } else {
                info!("Device resumed, the Vbus current limit is 100mA");
            }
        }
    }
}

async fn write_leds_lazy<'d, T: embassy_nrf::spim::Instance>(
    spim: &mut spim::Spim<'_, T>,
    buf: &[[[bool; 2]; NUM_COL_PINS]; NUM_ROW_PINS],
    prev_any_key: bool,
    led_en_pin: &mut Output<'_, AnyPin>,
) -> bool {
    let any_key = buf.as_flattened().as_flattened().iter().any(|x| *x);
    if any_key && !prev_any_key {
        led_en_pin.set_high();
        Timer::after_millis(50).await;
    }

    if any_key || prev_any_key {
        write_leds(spim, buf).await;
    }

    if !any_key && prev_any_key {
        led_en_pin.set_high();
    }

    any_key
}

async fn write_leds<'d, T: embassy_nrf::spim::Instance>(
    spim: &mut spim::Spim<'_, T>,
    buf: &[[[bool; 2]; NUM_COL_PINS]; NUM_ROW_PINS],
) {
    // freq = 4MHz => 1 bit = 0.25 µs
    // rst = 80 * 4 bits = 40 bytes
    // T0H = 1 bit, T0L = 7 bit
    // T1H = 3 bit, T1L = 5 bit
    let low_bit: u8 = 0b1000_0000;
    let high_bit: u8 = 0b1110_0000;
    let mut spi_buf = [0u8; 24 * 2 * NUM_COL_PINS * NUM_ROW_PINS + 50];
    for (i, row) in buf.iter().enumerate() {
        for (j, scan_pair) in row.iter().enumerate() {
            for (k, key) in scan_pair.iter().enumerate() {
                let led_index = LED_MAP[i][j][k] * 24;
                if *key {
                    spi_buf[led_index] = high_bit;
                    for c in 1..24 {
                        spi_buf[led_index + c] = low_bit;
                    }
                } else {
                    for c in 0..24 {
                        spi_buf[led_index + c] = low_bit;
                    }
                }
            }
        }
    }

    unwrap!(spim.transfer_from_ram(&mut [], spi_buf.as_slice()).await);
}

async fn sample_matrix(
    rows: &mut [AnalogPin; NUM_ROW_PINS],
    cols: &mut [AnalogPin; NUM_COL_PINS],
    buf: &mut [[[bool; 2]; NUM_COL_PINS]; NUM_ROW_PINS],
) {
    for i in 0..NUM_ROW_PINS {
        sample_row(&mut rows[i], cols, &mut buf[i]).await;
    }
}

async fn sample_row(row: &mut AnalogPin, cols: &mut [AnalogPin; NUM_COL_PINS], buf: &mut [[bool; 2]; NUM_COL_PINS]) {
    let row_psel = row.comp_psel;
    let row: AnyPin = row.into();
    let mut row_pin: Flex<AnyPin> = Flex::new(row);

    let mut cols: [Flex<AnyPin>; NUM_COL_PINS] = cols.each_mut().map(|p| Flex::new(Into::<AnyPin>::into(p)));
    for col in &mut cols {
        col.set_as_disconnected();
    }

    row_pin.set_as_input(Pull::Down);

    const THRESH_HIGH: u8 = 42;
    config_comp(row_psel, THRESH_HIGH, THRESH_HIGH);

    enable_comp().await;
    for i in 0..NUM_COL_PINS {
        cols[i].set_high();
        compiler_fence(core::sync::atomic::Ordering::SeqCst);
        cols[i].set_as_output(OutputDrive::Standard);

        cortex_m::asm::delay(100);

        // col high => switch pressed when row above thresh
        buf[i][0] = sample_comp_is_above();

        cols[i].set_as_disconnected();
    }
    disable_comp();

    row_pin.set_as_input(Pull::Up);

    const THRESH_LOW: u8 = 12;
    config_comp(row_psel, THRESH_LOW, THRESH_LOW);

    enable_comp().await;
    for i in 0..NUM_COL_PINS {
        cols[i].set_low();
        compiler_fence(core::sync::atomic::Ordering::SeqCst);
        cols[i].set_as_output(OutputDrive::Standard);

        cortex_m::asm::delay(100);

        // col low => switch pressed when row below thresh
        buf[i][1] = !sample_comp_is_above();

        cols[i].set_as_disconnected();
    }
    disable_comp();

    row_pin.set_as_disconnected();
}

fn config_comp(psel: comp::psel::PSEL_A, thup: u8, thdown: u8) {
    let r: &comp::RegisterBlock = unsafe { &*COMP::ptr() };

    r.psel.write(|w| w.psel().variant(psel));
    r.refsel.write(|w| w.refsel().vdd()); // default
    r.th.write(|w| unsafe { w.thup().bits(thup).thdown().bits(thdown) });
    r.mode.write(|w| w.sp().low().main().se()); // default

    // Ensure order of configuration and enabling
    compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

async fn enable_comp() {
    let r: &comp::RegisterBlock = unsafe { &*COMP::ptr() };

    // Enable
    r.enable.write(|w| unsafe { w.bits(2) });
    // Disable all events interrupts
    r.intenclr.write(|w| unsafe { w.bits(0x0000_000F) });

    interrupt::COMP_LPCOMP.unpend();
    unsafe { interrupt::COMP_LPCOMP.enable() };

    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    r.events_ready.reset();
    r.intenset.write(|w| w.ready().set_bit());

    // Don't reorder the start event before the previous writes
    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    r.tasks_start.write(|w| w.tasks_start().set_bit());

    // Wait for 'ready' event.
    poll_fn(move |cx| {
        WAKER.register(cx.waker());

        if r.events_ready.read().bits() != 0 {
            r.events_ready.reset();
            return Poll::Ready(());
        }

        Poll::Pending
    })
    .await;
}

fn sample_comp_is_above() -> bool {
    let r: &comp::RegisterBlock = unsafe { &*COMP::ptr() };

    r.tasks_sample.write(|w| unsafe { w.bits(1) });
    compiler_fence(Ordering::SeqCst);
    r.result.read().result().bit()
}

fn disable_comp() {
    let r: &comp::RegisterBlock = unsafe { &*COMP::ptr() };

    r.tasks_stop.write(|w| w.tasks_stop().set_bit());

    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    r.enable.write(|w| unsafe { w.bits(0) });

    compiler_fence(Ordering::SeqCst);
}

#[non_exhaustive]
pub struct AnalogPin {
    pub lpcomp_psel: lpcomp::psel::PSEL_A,
    pub comp_psel: PSEL_A,
    _saadc_any_input: saadc::AnyInput,
    _pin_port: u8,
}

impl Into<AnyPin> for &mut AnalogPin {
    fn into(self) -> AnyPin {
        unsafe { AnyPin::steal(self._pin_port) }
    }
}

impl<'d> Into<&'d mut saadc::AnyInput> for &'d mut AnalogPin {
    fn into(self) -> &'d mut saadc::AnyInput {
        &mut self._saadc_any_input
    }
}

macro_rules! impl_analog_pin {
    ($pin:ident, $pin_port:literal, $psel:ident) => {
        impl_analog_pin!(@local, embassy_nrf::peripherals::$pin, $pin_port, $psel);
    };
    (@local, $pin:ty, $pin_port:literal, $psel:ident) => {
        impl From<$pin> for AnalogPin {
            fn from(value: $pin) -> Self {
                AnalogPin { lpcomp_psel: embassy_nrf::pac::lpcomp::psel::PSEL_A::$psel, comp_psel: embassy_nrf::pac::comp::psel::PSEL_A::$psel, _saadc_any_input: value.degrade_saadc(), _pin_port: $pin_port }
            }
        }
    };
}

impl_analog_pin!(P0_02, 2, ANALOG_INPUT0);
impl_analog_pin!(P0_03, 3, ANALOG_INPUT1);
impl_analog_pin!(P0_04, 4, ANALOG_INPUT2);
impl_analog_pin!(P0_05, 5, ANALOG_INPUT3);
impl_analog_pin!(P0_28, 28, ANALOG_INPUT4);
impl_analog_pin!(P0_29, 29, ANALOG_INPUT5);
impl_analog_pin!(P0_30, 30, ANALOG_INPUT6);
impl_analog_pin!(P0_31, 31, ANALOG_INPUT7);
