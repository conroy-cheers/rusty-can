#![no_main]
#![no_std]

use panic_halt as _;
mod slcan;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use crate::slcan::SLCAN;
    use stm32f4xx_hal::{
        gpio::{Output, AF7, PB0, PB14, PB7, PD8, PD9},
        pac,
        prelude::*,
        rcc::RccExt,
        serial::{Config, Rx, Tx},
        timer::monotonic::MonoTimerUs,
    };

    type RxType = Rx<pac::USART3, u8>;
    type TxType = Tx<pac::USART3, u8>;

    #[shared]
    struct Shared {
        #[lock_free]
        tx_queue: crate::slcan::QueueType,
        #[lock_free]
        rx_queue: crate::slcan::QueueType,
    }

    #[local]
    struct Local {
        led_green: PB0<Output>,
        led_blue: PB7<Output>,
        led_red: PB14<Output>,
        slcan: SLCAN,
        rx: RxType,
        tx: TxType,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimerUs<pac::TIM2>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let gpiob = ctx.device.GPIOB.split();
        let gpiod = ctx.device.GPIOD.split();

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48_u32.MHz()).freeze();

        let led_green = gpiob.pb0.into_push_pull_output();
        let led_blue = gpiob.pb7.into_push_pull_output();
        let led_red = gpiob.pb14.into_push_pull_output();

        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        tick::spawn().ok();

        let tx_pin: PD8<AF7> = gpiod.pd8.into_alternate();
        let rx_pin: PD9<AF7> = gpiod.pd9.into_alternate();

        let serial = ctx
            .device
            .USART3
            .serial(
                (tx_pin, rx_pin),
                Config::default().baudrate(115200.bps()),
                &clocks,
            )
            .unwrap();

        let (tx, mut rx) = serial.split();
        rx.listen();

        let tx_queue = crate::slcan::QueueType::new();
        let rx_queue = crate::slcan::QueueType::new();

        let slcan = SLCAN::new();

        (
            Shared { tx_queue, rx_queue },
            Local {
                led_green,
                led_blue,
                led_red,
                slcan,
                rx,
                tx,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(shared=[tx_queue], local=[led_green, led_blue, tx])]
    fn tick(ctx: tick::Context) {
        ctx.local.led_green.toggle();
        // send all contents of the tx queue
        while let Some(to_send) = ctx.shared.tx_queue.pop_back() {
            serial_write(ctx.local.tx, ctx.local.led_blue, to_send);
        }
        tick::spawn_after(500.millis()).ok();
    }

    #[task(binds=USART3, shared=[tx_queue, rx_queue], local=[led_red, rx, slcan])]
    fn serial(ctx: serial::Context) {
        match ctx.local.rx.read() {
            Ok(c) => {
                ctx.local.slcan.handle_incoming_byte(c, ctx.shared.rx_queue).unwrap();
            }
            Err(_e) => {
                ctx.local.led_red.set_high();
            }
        }
    }

    fn serial_write(tx: &mut TxType, led: &mut PB7<Output>, data: u8) {
        led.set_high();
        tx.write(data).unwrap();
        while !tx.is_tx_empty() {} // serial.flush() not supported on USART3? so poll instead
        led.set_low();
    }
}
