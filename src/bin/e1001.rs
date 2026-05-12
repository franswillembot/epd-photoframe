#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embedded_hal::digital::{ErrorType, OutputPin as EmbeddedOutputPin};
use esp_hal::clock::CpuClock;

use esp_hal::gpio::{Input, InputConfig, Pin, Pull, RtcPin};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::GPIO12;

use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::Config as SpiConfig;
use esp_hal::spi::master::Spi;
use esp_hal::uart::{Config as UartConfig, UartRx};

extern crate alloc;

use epd_photoframe::app::{AppHardware, init_runtime, run_app};
use epd_photoframe::panel::gdey075t7::{Gdey075t7, ResetSleepHold};

struct HeldRtcResetPin {
    output: Output<'static>,
    hold: GPIO12<'static>,
}

impl HeldRtcResetPin {
    fn new(pin: GPIO12<'static>) -> Self {
        let output = Output::new(pin, Level::Low, OutputConfig::default());
        // The Output owns GPIO12 for normal runtime driving. This second
        // handle is only used to toggle RTC pad hold on the same pad before
        // entering ESP deep sleep and after waking.
        let hold = unsafe { GPIO12::steal() };
        Self { output, hold }
    }
}

impl ErrorType for HeldRtcResetPin {
    type Error = core::convert::Infallible;
}

impl EmbeddedOutputPin for HeldRtcResetPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.output.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.output.set_high();
        Ok(())
    }
}

impl ResetSleepHold for HeldRtcResetPin {
    fn release_sleep_hold(&mut self) {
        self.hold.rtcio_pad_hold(false);
    }

    fn hold_low_for_sleep(&mut self) {
        self.hold.rtcio_pad_hold(true);
    }
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    init_runtime(
        peripherals.PSRAM,
        peripherals.TIMG0,
        peripherals.SW_INTERRUPT,
    );

    let (gpio_btn_refresh, gpio_btn_previous, gpio_btn_next, led_pin) = (
        peripherals.GPIO3,
        peripherals.GPIO5,
        peripherals.GPIO4,
        peripherals.GPIO6,
    );

    // E1001 / E1002 appear to have an SY6974B on I2C1, but it is not
    // accessible from this firmware setup, so normal-mode power status
    // reporting is disabled for them.
    let (refresh_button_label, has_sy6974b) = ("Refresh button (green)", false);
    let uart_rx = UartRx::new(peripherals.UART0, UartConfig::default())
        .unwrap()
        .with_rx(peripherals.GPIO44)
        .into_async();

    // --- Build the panel SPI bus and EPD driver (shared by both flows) ---
    let epd_spi_bus = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_write_bit_order(esp_hal::spi::BitOrder::MsbFirst)
            .with_frequency(esp_hal::time::Rate::from_mhz(20))
            .with_mode(SpiMode::_0),
    )
    .unwrap();

    let mut epd_spi_bus = epd_spi_bus
        .with_sck(peripherals.GPIO7)
        .with_mosi(peripherals.GPIO9)
        .into_async();

    let epd = Gdey075t7::new(
        &mut epd_spi_bus,
        Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default()),
        Input::new(
            peripherals.GPIO13,
            InputConfig::default().with_pull(Pull::Up),
        ),
        Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default()),
        HeldRtcResetPin::new(peripherals.GPIO12),
    );

    // One task later drives both per-wake sensor reads (battery ADC +
    // SHT40 over I²C0) concurrently via `join`.
    let i2c0 =
        esp_hal::i2c::master::I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
            .unwrap()
            .with_sda(peripherals.GPIO19)
            .with_scl(peripherals.GPIO20)
            .into_async();

    let board = AppHardware {
        gpio_btn_refresh: gpio_btn_refresh.degrade(),
        gpio_btn_previous: gpio_btn_previous.degrade(),
        gpio_btn_next: gpio_btn_next.degrade(),
        led_pin: led_pin.degrade(),
        refresh_button_label,
        has_sy6974b,
        uart_rx,
        spi_bus: epd_spi_bus,
        epd,
        i2c0,
        flash: peripherals.FLASH,
        lpwr: peripherals.LPWR,
        wifi: peripherals.WIFI,
        battery_enable_pin: peripherals.GPIO21,
        adc1: peripherals.ADC1,
        battery_sense: peripherals.GPIO1,
        ledc: peripherals.LEDC,
        buzzer_pin: peripherals.GPIO45,
    };

    run_app(spawner, board).await
}
