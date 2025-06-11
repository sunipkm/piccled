//! Control an LED connected to GPIO6 of a RP2040 using
//! 10 kHz PWM.
//!
//! The RP2040 is exposed to the host as a CDC ACM serial
//! device
#![no_std]
#![no_main]

use core::{
    fmt::{self, Write},
    panic,
    str::from_utf8,
};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_rp::{bind_interrupts, clocks};
use embassy_rp::{
    peripherals::USB,
    usb::{Driver, InterruptHandler},
};
use embassy_usb::{
    Builder as UsbBuilder, Config as UsbConfig,
    class::cdc_acm::{CdcAcmClass, State as CdcAcmState},
    driver::EndpointError,
};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Debug, driver);
}

/// This macro simplifies the [embassy_futures::join::join] function.
///
/// Example usage:
/// ```no_run
/// let task1 = async { /* ... */ };
/// let task2 = async { /* ... */ };
/// let task3 = async { /* ... */ };
/// let task4 = async { /* ... */ };
/// // single join
/// embassy_join!((task1)).await;
/// // double join
/// embassy_join!((task1, task2)).await;
/// // multiple join
/// embassy_join!((task1, task2, (task3, task4))).await;
///
macro_rules! embassy_join {
    (($task:expr)) => { $task };
    (($task1:expr, $task2:expr)) => {
        embassy_futures::join::join($task1, $task2)
    };
    (($task1:expr, $task2:expr, $($rest:tt),+)) => {
        embassy_futures::join::join($task1, embassy_futures::join::join($task2, embassy_join!($($rest)*)))
    };
}

/// This macro simplifies the [embassy_futures::select::select] function.
///
/// Example usage:
/// ```no_run
/// let task1 = async { /* ... */ };
/// let task2 = async { /* ... */ };
/// let task3 = async { /* ... */ };
/// let task4 = async { /* ... */ };
/// // single select
/// embassy_select!((task1)).await;
/// // double select
/// embassy_select!((task1, task2)).await;
/// // multiple select
/// embassy_select!((task1, task2, (task3, task4))).await;
///
#[allow(unused_macros)]
macro_rules! embassy_select {
    (($task:expr)) => { $task };
    (($task1:expr, $task2:expr)) => {
        embassy_futures::select::select($task1, $task2)
    };
    (($task1:expr, $task2:expr, $($rest:tt),+)) => {
        embassy_futures::select::select($task1, embassy_futures::select::select($task2, embassy_select!($($rest)*)))
    };
}

/// It requires an external signal to be manually triggered on PIN 16. For
/// example, this could be accomplished using an external power source with a
/// button so that it is possible to toggle the signal from low to high.
///
/// This example will begin with turning on the LED on the board and wait for a
/// high signal on PIN 16. Once the high event/signal occurs the program will
/// continue and turn off the LED, and then wait for 2 seconds before completing
/// the loop and starting over again.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Initialized peripherals");

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = UsbConfig::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("PIC-D LED Controller");
    config.serial_number = Some(env!("GIT_HASH"));
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 512];

    let mut state = CdcAcmState::new();

    let mut builder = UsbBuilder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    info!("Created CDC ACM class");

    // Build the builder.
    let mut usb = builder.build();
    info!("Built USB device");

    // Run the USB device.
    let usb_fut = usb.run();
    info!("Running USB device");

    // PWM Controller
    let (mut pwma, mut pwmled) = {
        let desired_freq_hz = 10_000; // 10 kHz
        let clk_hz = clocks::clk_sys_freq(); // Clock frequency in Hz
        let divider = 16u8;
        let period = (clk_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

        let mut pwmcfg = PwmConfig::default();
        pwmcfg.top = period;
        pwmcfg.divider = divider.into();
        (
            Pwm::new_output_a(p.PWM_SLICE3, p.PIN_6, pwmcfg.clone()),
            Pwm::new_output_b(p.PWM_SLICE4, p.PIN_25, pwmcfg),
        )
    };

    pwma.set_duty_cycle_fully_off()
        .expect("Could not turn off the PWM pin.");
    pwmled
        .set_duty_cycle_fully_off()
        .expect("Could not turn off the PWM pin.");
    info!("Started PWM cycle in fully off mode.");

    let mut data = [0; 64];
    let mut output = Buffer([0; 64], 0);
    let control_fut = async {
        loop {
            class.wait_connection().await;
            info!("Client connected");
            while let Ok(n) = class.read_packet(&mut data).await {
                // read from tty
                if let Some(s) = from_utf8(&data[..n])
                    .ok()
                    .and_then(|s| s.trim().parse::<u8>().ok())
                    .map(|s| s.clamp(0, 100))
                {
                    info!("Received: {}", s);
                    if let Err(e) = {
                        if pwmled.set_duty_cycle_percent(s).is_err() {
                            error!("Error setting LED PWM to {}%.", s);
                        }
                        if pwma.set_duty_cycle_percent(s).is_err() {
                            error!("Error setting PWM to {}%.", s);
                            if core::write!(&mut output, "ERR {s}\r\n").is_err() {
                                error!("Could not write output to buffer.");
                            }
                        } else if core::write!(&mut output, "OK {s}\r\n").is_err() {
                            error!("Could not write output to buffer.");
                        }
                        class.write_packet(&output.0[..output.1])
                    }
                    .await
                    {
                        error!("Could not write output to USB: {:?}", e);
                    }
                } else {
                    error!("Received invalid data: {:?}", &data[..n]);
                    if core::write!(&mut output, "ERR\r\n").is_err() {
                        error!("Could not write output to buffer.");
                    }
                    if class.write_packet(&output.0[..output.1]).await.is_err() {
                        error!("Could not write output to USB.");
                    }
                }
            }
            if pwma.set_duty_cycle_fully_off().is_err() {
                error!("Could not turn off the PWM pin.");
            }
            if pwmled.set_duty_cycle_fully_off().is_err() {
                error!("Could not turn off the PWM pin.");
            }
        }
    };

    embassy_join!((usb_fut, control_fut)).await;
}

#[derive(Debug)]
struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

struct Buffer<const N: usize>([u8; N], usize);

impl<const N: usize> Write for Buffer<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let space_left = self.0.len() - self.1;
        if space_left > s.len() {
            self.0[self.1..][..s.len()].copy_from_slice(s.as_bytes());
            self.1 += s.len();
            Ok(())
        } else {
            Err(fmt::Error)
        }
    }
}
