#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use rtfm::app;

#[app(device = atsamd21e16l)]
const APP: () = {

    fn start_aux_adc_read() {
        static mut adc_aux_i: u8 = 0;
        *adc_aux_i = (*adc_aux_i + 1) % 3;
        if adc_aux_i == 0 {

        } else if adc_aux_i == 1 {

        } else {

        }
    }

    #[init]
    fn init() {}
};
