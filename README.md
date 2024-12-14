# is31fl3236

A rust-embedded driver for the Lumissil Microsystems IS31FL32xx LED driver

Fork of [this repo](https://github.com/ost-ing/is31fl32xx) for the is31fl3236

- https://crates.io/crates/is31fl32xx

## About

The IS31FL323 is LED drivers with 36 constant current channels respectively. Each channel can be pulse width modulated (PWM) by 16 bits for smooth LED brightness control.

This crate provides both blocking and DMA compatible APIs (via static callbacks) for applications utilising `embedded_hal` traits.
