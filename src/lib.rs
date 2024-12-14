//! # IS31FL32xx library
//! A rust-embedded driver for the Lumissil Microsystems IS31FL32xx LED driver
//! Currently supports the IS31FL3205 and IS31FL3237

#![no_std]
#![allow(unused)]

use core::marker::PhantomData;
use embedded_hal::{
    blocking::i2c::{Read, Write, WriteRead},
    digital::v2::OutputPin,
};

#[derive(Debug)]
pub enum Error<I> {
    /// I2C bus error
    I2C(I),
    /// Connection error (device not found)
    Conn,
    /// Address error (invalid or out of bounds)
    Address,
    /// Port error (invalid or out of bounds)
    Port,
    /// Configured Interface Error
    InterfaceConfig,
    /// Enable line
    EnableLine,
    /// Channel is beyond the supported number
    ChannelOutOfBounds,
}

/// Model
pub trait Model {
    /// Retrieve the register value for the specific model
    fn register_value(register: Register) -> u8;
    /// Retreive the channel count for the specific count
    fn channel_count() -> usize;
}

/// The IS31FL3237 is an LED driver with 36 constant current channels.
pub struct IS31FL3236;
impl Model for IS31FL3236 {
    fn register_value(register: Register) -> u8 {
        match register {
            Register::Shutdown => 0x00,
            Register::Pwm => 0x01,
            Register::Update => 0x25,
            Register::LedControl => 0x26,
            Register::GlobalOutput => 0x4a,
            Register::OutputFrequency => 0x4b,
            Register::Reset => 0x4f,
        }
    }

    fn channel_count() -> usize {
        36
    }
}

#[derive(Copy, Clone)]
pub enum Register {
    /// Power control register
    Shutdown,
    /// PWM register
    Pwm,
    /// Update the PWM and Scaling data
    Update,
    // LED enable and current setting
    LedControl,
    GlobalOutput,
    /// Output frequency of the device
    OutputFrequency,
    /// Reset all registers
    Reset,
}

pub enum SoftwareShutdownMode {
    SoftwareShutdown = 0b0,
    Normal = 0b1,
}

pub enum OutputCurrent {
    IMax = 0b00,
    IMaxDiv2 = 0b01,
    IMaxDiv3 = 0b10,
    IMaxDiv4 = 0b11,
}

pub enum OutputMode {
    LEDOn = 0b0,
    LEDOff = 0b1,
}

pub enum OutputFrequency {
    ThreeKHz = 0b0,
    TwentyTwoKHz = 0b1,
}

pub struct Is31fl32xx<MODEL, I2C> {
    /// `embedded-hal` compatible I2C instance
    interface: Option<I2C>,
    /// Transfer callbacks
    transfer_callback: Option<fn(addr: u8, data: &[u8])>,
    /// Device address
    address: u8,
    /// Model
    model: PhantomData<MODEL>,
}

/// Each of the IS31FL32xx messages are defined by the Message struct.
/// These provide an interface to the transferred data to modify or read registers of the IS31FL32xx
pub struct Message<MODEL> {
    /// The register to write or read to/from
    register: Register,
    /// An offset of the base register address
    register_offset: u8,
    /// The data buffer, with a statically allocated 24 bytes
    data: [u8; 24],
    /// The length of the transferred data
    data_length: usize,
    /// Model
    model: PhantomData<MODEL>,
}

/// TODO: Add support for advanced features:
/// - phase delay & clock phase
/// - spread spectrum for EMI reduction techniques
/// - open / short functionality and detection
/// - temperature detection
impl<MODEL> Message<MODEL>
where
    MODEL: Model,
{
    /// Creates a new Message
    fn new(register: Register, raw_data: &[u8]) -> Self {
        let mut data = [0u8; 24];
        data[0..raw_data.len()].copy_from_slice(raw_data);
        Self {
            register,
            data,
            data_length: raw_data.len(),
            register_offset: 0,
            model: PhantomData,
        }
    }

    pub fn payload(&self) -> ([u8; 25], usize) {
        let mut buff = [0u8; 25];
        let data = &mut buff[0..self.data_length + 1];
        data[0] = self.register_value() + self.register_offset;
        data[1..self.data_length + 1].copy_from_slice(&self.data[0..self.data_length]);
        (buff, self.data_length + 1)
    }

    fn register_value(&self) -> u8 {
        MODEL::register_value(self.register)
    }

    /// Defines a register offset, for example when targeting a specific channel
    fn register_offset(mut self, offset: u8) -> Self {
        self.register_offset = offset;
        self
    }

    fn pwm(channel: u8, value: u8) -> Self {
        Self::new(Register::Pwm, &[value]).register_offset(channel)
    }

    /// Update all PWM registers with the loaded values
    /// Must be called after setting the Pulse Width Modulation register
    pub fn update() -> Self {
        Self::new(Register::Update, &[0x00])
    }

    pub fn led_control(channel: u8, current: OutputCurrent, mode: OutputMode) -> Self {
        Self::new(Register::LedControl, &[(current as u8) << 1 | (mode as u8)]).register_offset(channel)
    }

    /// Adjust the global current usage of the device, see manual for detail about current usage
    pub fn global_output(value: OutputMode) -> Self {
        Self::new(Register::GlobalOutput, &[value as u8])
    }

    pub fn shutdown(mode: SoftwareShutdownMode) -> Self {
        Self::new(Register::Shutdown, &[mode as u8])
    }

    pub fn output_frequency(frequency: OutputFrequency) -> Self {
        Self::new(Register::OutputFrequency, &[frequency as u8])
    }

    pub fn reset() -> Self {
        Self::new(Register::Reset, &[0x00])
    }
}

impl<MODEL, I2C, S> Is31fl32xx<MODEL, I2C>
where
    MODEL: Model,
    I2C: Write<u8, Error = S> + Read<u8, Error = S> + WriteRead<u8, Error = S>,
{
    /// Initialize the Is31fl32xx with a flexible asynchronous callback interface
    /// * `address` - User definable address which should associate with the physical AD pin(s) of the device
    /// * `en` - The enable line
    /// * `callback` - Callback for custom transmission of the address and dataframe.
    pub fn init_with_callback(address: u8, callback: fn(addr: u8, data: &[u8])) -> Self {
        Self {
            address,
            interface: None,
            transfer_callback: Some(callback),
            model: PhantomData,
        }
    }

    /// Initialize the ≈ with blocking i2c
    /// * `address` - User definable address which should associate with the physical AD pin(s) of the device
    /// * `en` - The enable line
    /// * `i2c` - i2c interface
    pub fn init_with_i2c(address: u8, i2c: I2C) -> Self {
        Self {
            address,
            interface: Some(i2c),
            transfer_callback: None,
            model: PhantomData,
        }
    }

    /// Release underlying resources
    pub fn release(self) -> Option<I2C> {
        self.interface
    }

    /// Write a Message to the Is31fl32xx, will either use the blocking i2c interface
    /// Or write to the callback for DMA / Interrupt asynchronous communication
    fn write(&mut self, message: Message<MODEL>) -> Result<(), Error<S>> {
        let mut buff: [u8; 24] = [0u8; 24];

        // Take the first 2 bits of the user configurable address and
        // OR it with the hardcoded manufacturer address
        let address = 0b1111 | ((self.address & 0x3) << 1);

        let data = &mut buff[0..message.data_length + 1];
        data[0] = message.register_value() + message.register_offset;
        data[1..message.data_length + 1].copy_from_slice(&message.data[0..message.data_length]);

        if self.interface.is_some() {
            self.interface
                .as_mut()
                .unwrap()
                .write(address, &data[0..message.data_length + 1])
                .map_err(Error::I2C)?;
        } else if self.transfer_callback.is_some() {
            self.transfer_callback.unwrap()(address, &data[0..message.data_length + 1]);
        } else {
            return Err(Error::InterfaceConfig);
        }
        Ok(())
    }

    pub fn set_shutdown(
        &mut self,
        mode: SoftwareShutdownMode,
    ) -> Result<(), Error<S>> {
        self.write(Message::shutdown(mode))
    }

    pub fn reset(&mut self) -> Result<(), Error<S>> {
        self.write(Message::reset())
    }

    pub fn set_output_frequency(&mut self, frequency: OutputFrequency) -> Result<(), Error<S>> {
        self.write(Message::output_frequency(frequency))
    }

    /// Set the global current usage, a value of 0xFF will use the maximum current as allowed by the Rin resistance
    pub fn set_global_output(&mut self, value: OutputMode) -> Result<(), Error<S>> {
        self.write(Message::global_output(value))
    }

    pub fn set_pwm(&mut self, channel: u8, value: u8) -> Result<(), Error<S>> {
        if channel as usize > MODEL::channel_count() - 1 {
            return Err(Error::ChannelOutOfBounds);
        }

        self.write(Message::pwm(channel, value))?;
        // TODO: do we need this update?
        self.write(Message::update())
    }

    pub fn set_led(&mut self, channel: u8, current: OutputCurrent, state: OutputMode) -> Result<(), Error<S>> {
        if channel as usize > MODEL::channel_count() - 1 {
            return Err(Error::ChannelOutOfBounds);
        }

        self.write(Message::led_control(channel, current, state))?;
        // TODO: do we need this update?
        self.write(Message::update())
    }
}
