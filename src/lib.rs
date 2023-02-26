#![no_std]
use embedded_hal::{
    blocking::spi,
    spi::{Phase, Polarity},
};
use fugit::HertzU32;
use pio::{Instruction, InstructionOperands};
use rp2040_hal::{
    gpio::{Disabled, DisabledConfig, Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{
        PIOExt, PinDir, PinState, Rx, ShiftDirection, StateMachine, StateMachineIndex, Tx,
        UninitStateMachine, PIO,
    },
};

#[cfg(not(feature = "defmt"))]
mod defmt {
    #[macro_export]
    macro_rules! info {
        ($($_:tt)*) => {{
            compile_error!("woops");
        }};
    }
    #[macro_export]
    macro_rules! error {
        ($($_:tt)*) => {{}};
    }

    // macros are exported at the root of the crate so pull them back here
    pub use super::{error, info};
}

pub struct Spi<'pio, P, SMI, MISO, MOSI, SCLK, const DS: u8>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    MISO: PinId,
    MOSI: PinId,
    SCLK: PinId,
    Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
{
    _pio: &'pio mut PIO<P>,
    _sm: StateMachine<(P, SMI), rp2040_hal::pio::Running>,
    tx: Tx<(P, SMI)>,
    rx: Rx<(P, SMI)>,
    #[allow(clippy::type_complexity)]
    _pins: (
        Pin<MISO, Function<P>>,
        Pin<MOSI, Function<P>>,
        Pin<SCLK, Function<P>>,
    ),
}

type NewErr<P, SMI, MISO, MOSI, SCLK, MisoDisabledConfig, MosiDisabledConfig, SclkDisabledConfig> = (
    UninitStateMachine<(P, SMI)>,
    Pin<MISO, Disabled<MisoDisabledConfig>>,
    Pin<MOSI, Disabled<MosiDisabledConfig>>,
    Pin<SCLK, Disabled<SclkDisabledConfig>>,
);
impl<'pio, P, SMI, MISO, MOSI, SCLK, const DS: u8> Spi<'pio, P, SMI, MISO, MOSI, SCLK, DS>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    MISO: PinId,
    MOSI: PinId,
    SCLK: PinId,
    Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
{
    #[allow(clippy::type_complexity)]
    pub fn new<MisoDisabledConfig, MosiDisabledConfig, SclkDisabledConfig>(
        (pio, sm): (&'pio mut PIO<P>, UninitStateMachine<(P, SMI)>),
        (miso, mosi, mut sclk): (
            Pin<MISO, Disabled<MisoDisabledConfig>>,
            Pin<MOSI, Disabled<MosiDisabledConfig>>,
            Pin<SCLK, Disabled<SclkDisabledConfig>>,
        ),
        mode: embedded_hal::spi::Mode,
        bus_freq: HertzU32,
        clock_freq: HertzU32,
    ) -> Result<
        Self,
        NewErr<
            P,
            SMI,
            MISO,
            MOSI,
            SCLK,
            MisoDisabledConfig,
            MosiDisabledConfig,
            SclkDisabledConfig,
        >,
    >
    where
        MisoDisabledConfig: DisabledConfig,
        MosiDisabledConfig: DisabledConfig,
        SclkDisabledConfig: DisabledConfig,
    {
        let program = pio_proc::pio_asm!(
        ".side_set 1 opt"

        ".wrap_target"
        "bit_loop:"
        "  out pins 1   side    0 [1] ; Set MOSI on to-inactive transition"
        "  in  pins 1   side    1 [1] ; Get MISO state on to-active transition"
        "  jmp !osre    bit_loop"

        "public entry_point:"
        "  nop          side    0 ; reset clk to inactive state"
        "  pull ifempty"
        ".wrap"
        );
        let mut entry_point = program.public_defines.entry_point;
        let mut program = program.program;

        if let Phase::CaptureOnSecondTransition = mode.phase {
            // transition sclk to-active with setting mosi
            program.code[0] |= 0b0000_1000_0000_0000;
            // transition sclk to-inactive with getting miso
            program.code[1] &= 0b1111_0111_1111_1111;
        };

        let installed = match pio.install(&program) {
            Ok(inst) => inst,
            Err(_) => return Err((sm, miso, mosi, sclk)),
        };
        entry_point += i32::from(installed.offset());
        if entry_point > 32 {
            // TODO: this should check against a value from the PIO, not a hardcoded value.
            defmt::error!("Entry point set beyond pio's memory.");
            return Err((sm, miso, mosi, sclk));
        }
        let entry_point = entry_point as u8;

        // we need u64 to handle the magnitude of rem*256
        let bus_freq = u64::from(bus_freq.to_Hz()) * 2;
        let clock_freq = u64::from(clock_freq.to_Hz());
        let mut int = clock_freq / bus_freq;
        let rem = clock_freq - (int * bus_freq);
        let frac = (rem * 256) / bus_freq;

        if !(1..=65536).contains(&int) || (int == 65536 && frac != 0) {
            defmt::error!("The ratio between the bus frequency and the system clock must be within [1.0, 65536.0].");
            pio.uninstall(installed);
            return Err((sm, miso, mosi, sclk));
        }
        // 65536.0 is represented as 0 in the pio's clock divider
        if int == 65536 {
            int = 0;
        }
        // Using lossy conversion because range have been checked
        let int: u16 = int as u16;
        let frac: u8 = frac as u8;

        let (mut sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            .buffers(rp2040_hal::pio::Buffers::RxTx)
            .out_pins(MOSI::DYN.num, 1)
            .in_pin_base(MISO::DYN.num)
            .side_set_pin_base(SCLK::DYN.num)
            .autopull(true)
            .autopush(true)
            // msb/lsb first can be selected here
            .out_shift_direction(ShiftDirection::Left)
            .in_shift_direction(ShiftDirection::Left)
            .pull_threshold(DS)
            .push_threshold(DS)
            .clock_divisor_fixed_point(int, frac)
            .build(sm);

        if let Polarity::IdleHigh = mode.polarity {
            sclk.set_output_override(rp2040_hal::gpio::OutputOverride::Invert);
        } else {
            sclk.set_output_override(rp2040_hal::gpio::OutputOverride::DontInvert);
        }
        let sclk = sclk.into_push_pull_output_in_state(rp2040_hal::gpio::PinState::Low);
        let miso = miso.into_floating_input();
        let mosi = mosi.into_push_pull_output_in_state(rp2040_hal::gpio::PinState::Low);

        sm.set_pins([
            (MOSI::DYN.num, PinState::Low),
            (SCLK::DYN.num, PinState::Low),
        ]);
        sm.set_pindirs([
            (MISO::DYN.num, PinDir::Input),
            (MOSI::DYN.num, PinDir::Output),
            (SCLK::DYN.num, PinDir::Output),
        ]);
        let miso = miso.into_mode();
        let mosi = mosi.into_mode();
        let sclk = sclk.into_mode();

        sm.exec_instruction(Instruction {
            operands: InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: entry_point,
            },
            delay: 0,
            side_set: None,
        });
        let sm = sm.start();

        Ok(Self {
            _pio: pio,
            _sm: sm,
            tx,
            rx,
            _pins: (miso, mosi, sclk),
        })
    }
}

macro_rules! impl_write {
    ($type:ty, $fn:ident, [$($nr:expr),+]) => {
        $(
        impl<'pio, P, SMI, MISO, MOSI, SCLK> embedded_hal::spi::FullDuplex<$type>
            for Spi<'pio, P, SMI, MISO, MOSI, SCLK, $nr>
        where
            P: PIOExt + FunctionConfig,
            SMI: StateMachineIndex,
            MISO: PinId,
            MOSI: PinId,
            SCLK: PinId,
            Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
        {
            type Error = core::convert::Infallible;

            fn read(&mut self) -> nb::Result<$type, Self::Error> {
                if let Some(r) = self.rx.read() {
                    Ok(r as $type)
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }

            fn send(&mut self, word: $type) -> nb::Result<(), Self::Error> {
                if self.tx.$fn(word) {
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }
        impl<'pio, P, SMI, MISO, MOSI, SCLK> spi::write::Default<$type>
            for Spi<'pio, P, SMI, MISO, MOSI, SCLK, $nr>
        where
            P: PIOExt + FunctionConfig,
            SMI: StateMachineIndex,
            MISO: PinId,
            MOSI: PinId,
            SCLK: PinId,
            Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
        {
        }
        impl<'pio, P, SMI, MISO, MOSI, SCLK> spi::transfer::Default<$type>
            for Spi<'pio, P, SMI, MISO, MOSI, SCLK, $nr>
        where
            P: PIOExt + FunctionConfig,
            SMI: StateMachineIndex,
            MISO: PinId,
            MOSI: PinId,
            SCLK: PinId,
            Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
        {
        }
        impl<'pio, P, SMI, MISO, MOSI, SCLK> spi::write_iter::Default<$type>
            for Spi<'pio, P, SMI, MISO, MOSI, SCLK, $nr>
        where
            P: PIOExt + FunctionConfig,
            SMI: StateMachineIndex,
            MISO: PinId,
            MOSI: PinId,
            SCLK: PinId,
            Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
        {
        }
        )+
    };
}
impl_write!(u8, write_u8_replicated, [1, 2, 3, 4, 5, 6, 7, 8]);
impl_write!(u16, write_u16_replicated, [9, 10, 11, 12, 13, 14, 15, 16]);
impl_write!(
    u32,
    write,
    [17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32]
);

impl<'pio, P, SMI, MISO, MOSI, SCLK, const DS: u8> rp2040_hal::dma::ReadTarget
    for Spi<'pio, P, SMI, MISO, MOSI, SCLK, DS>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    MISO: PinId,
    MOSI: PinId,
    SCLK: PinId,
    Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
{
    type ReceivedWord = u32;

    fn rx_treq() -> Option<u8> {
        <Rx<(P, SMI)>>::rx_treq()
    }

    fn rx_address_count(&self) -> (u32, u32) {
        self.rx.rx_address_count()
    }

    fn rx_increment(&self) -> bool {
        self.rx.rx_increment()
    }
}
impl<'pio, P, SMI, MISO, MOSI, SCLK, const DS: u8> rp2040_hal::dma::WriteTarget
    for Spi<'pio, P, SMI, MISO, MOSI, SCLK, DS>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    MISO: PinId,
    MOSI: PinId,
    SCLK: PinId,
    Function<P>: ValidPinMode<MISO> + ValidPinMode<MOSI> + ValidPinMode<SCLK>,
{
    type TransmittedWord = u32;

    fn tx_treq() -> Option<u8> {
        <Tx<(P, SMI)>>::tx_treq()
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        self.tx.tx_address_count()
    }

    fn tx_increment(&self) -> bool {
        self.tx.tx_increment()
    }
}
