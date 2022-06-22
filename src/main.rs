#![no_main]
#![no_std]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use panic_halt as _;
mod canbus;
mod slcan;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use crate::canbus::CANBus;
    use crate::slcan::SLCAN;
    use stm32f4xx_hal::{
        can::Can,
        gpio::{Output, AF7, AF9, PB0, PB14, PB7, PD0, PD1, PD8, PD9},
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
        #[lock_free]
        can: CANBus<Can<pac::CAN1, (PD1<AF9>, PD0<AF9>)>>,
        #[lock_free]
        slcan: SLCAN,
    }

    #[local]
    struct Local {
        led_green: PB0<Output>,
        led_blue: PB7<Output>,
        led_red: PB14<Output>,
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
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

        let led_green = gpiob.pb0.into_push_pull_output();
        let led_blue = gpiob.pb7.into_push_pull_output();
        let led_red = gpiob.pb14.into_push_pull_output();

        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        tick::spawn().ok();
        tick_blink::spawn().ok();

        let can = {
            let rx_pin: PD0<AF9> = gpiod.pd0.into_alternate();
            let tx_pin: PD1<AF9> = gpiod.pd1.into_alternate();

            let can = ctx.device.CAN1.can((tx_pin, rx_pin));
            CANBus::new(can)
        };
        tick_can::spawn().ok();

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
            Shared {
                tx_queue,
                rx_queue,
                can,
                slcan,
            },
            Local {
                led_green,
                led_blue,
                led_red,
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

    #[task(priority=2, local=[led_green])]
    fn tick_blink(ctx: tick_blink::Context) {
        ctx.local.led_green.toggle();
        tick_blink::spawn_after(250.millis()).ok();
    }

    #[task(priority=2, shared=[tx_queue], local=[led_blue, tx])]
    fn tick(ctx: tick::Context) {
        // send all contents of the tx queue
        while let Some(to_send) = ctx.shared.tx_queue.pop_front() {
            serial_write(ctx.local.tx, ctx.local.led_blue, to_send);
        }
        tick::spawn_after(50.millis()).ok();
    }

    #[task(priority=2, shared=[can, tx_queue, slcan], local=[])]
    fn tick_can(ctx: tick_can::Context) {
        if ctx.shared.can.is_enabled() {
            match ctx.shared.can.receive() {
                Ok(frame) => {
                    SLCAN::handle_incoming_can_frame(&frame, ctx.shared.tx_queue).unwrap();
                }
                Err(_e) => {}
            }
        }
        tick_can::spawn_after(1.millis()).ok();
    }

    #[task(priority=2, binds=USART3, shared=[tx_queue, rx_queue, can, slcan], local=[rx, led_red])]
    fn serial(ctx: serial::Context) {
        ctx.local.rx.unlisten();
        loop {
            let read_byte = ctx.local.rx.read();
            if read_byte.is_err() {
                break;
            }
            let read_byte = read_byte.unwrap();
            match ctx
                .shared
                .slcan
                .handle_incoming_byte(read_byte, ctx.shared.rx_queue)
            {
                Ok(cmd) => {
                    if cmd.is_some() {
                        // Handle command
                        let cmd_output = cmd.unwrap().run(ctx.shared.slcan, ctx.shared.can);
                        match &cmd_output {
                            Ok(_) => {}
                            Err(_e) => ctx.local.led_red.set_high(),
                        }
                        // panic if buffer full
                        ctx.shared
                            .slcan
                            .handle_command_output(&cmd_output, ctx.shared.tx_queue)
                            .unwrap();
                    }
                }
                Err(_e) => {
                    // Invalid command
                    ctx.local.led_red.set_high()
                }
            }
        }
        ctx.local.rx.listen();
    }

    fn serial_write(tx: &mut TxType, led: &mut PB7<Output>, data: u8) {
        led.set_high();
        tx.write(data).unwrap();
        while !tx.is_tx_empty() {} // serial.flush() not supported on USART3? so poll instead
        led.set_low();
    }
}
