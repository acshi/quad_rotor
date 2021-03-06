#[doc = r" Register block"]
#[repr(C)]
pub struct RegisterBlock {
    #[doc = "0x00 - Data Direction"]
    pub dir0: DIR,
    #[doc = "0x04 - Data Direction Clear"]
    pub dirclr0: DIRCLR,
    #[doc = "0x08 - Data Direction Set"]
    pub dirset0: DIRSET,
    #[doc = "0x0c - Data Direction Toggle"]
    pub dirtgl0: DIRTGL,
    #[doc = "0x10 - Data Output Value"]
    pub out0: OUT,
    #[doc = "0x14 - Data Output Value Clear"]
    pub outclr0: OUTCLR,
    #[doc = "0x18 - Data Output Value Set"]
    pub outset0: OUTSET,
    #[doc = "0x1c - Data Output Value Toggle"]
    pub outtgl0: OUTTGL,
    #[doc = "0x20 - Data Input Value"]
    pub in0: IN,
    #[doc = "0x24 - Control"]
    pub ctrl0: CTRL,
    #[doc = "0x28 - Write Configuration"]
    pub wrconfig0: WRCONFIG,
    _reserved0: [u8; 4usize],
    #[doc = "0x30 - Peripheral Multiplexing n - Group 0"]
    pub pmux0_: [PMUX0_; 16],
    #[doc = "0x40 - Pin Configuration n - Group 0"]
    pub pincfg0_: [PINCFG0_; 32],
    _reserved1: [u8; 32usize],
    #[doc = "0x80 - Data Direction"]
    pub dir1: DIR,
    #[doc = "0x84 - Data Direction Clear"]
    pub dirclr1: DIRCLR,
    #[doc = "0x88 - Data Direction Set"]
    pub dirset1: DIRSET,
    #[doc = "0x8c - Data Direction Toggle"]
    pub dirtgl1: DIRTGL,
    #[doc = "0x90 - Data Output Value"]
    pub out1: OUT,
    #[doc = "0x94 - Data Output Value Clear"]
    pub outclr1: OUTCLR,
    #[doc = "0x98 - Data Output Value Set"]
    pub outset1: OUTSET,
    #[doc = "0x9c - Data Output Value Toggle"]
    pub outtgl1: OUTTGL,
    #[doc = "0xa0 - Data Input Value"]
    pub in1: IN,
    #[doc = "0xa4 - Control"]
    pub ctrl1: CTRL,
    #[doc = "0xa8 - Write Configuration"]
    pub wrconfig1: WRCONFIG,
    _reserved2: [u8; 4usize],
    #[doc = "0xb0 - Peripheral Multiplexing n - Group 1"]
    pub pmux1_0: PMUX1_,
    #[doc = "0xb4 - Peripheral Multiplexing n - Group 1"]
    pub pmux1_4: PMUX1_,
    #[doc = "0xb8 - Peripheral Multiplexing n - Group 1"]
    pub pmux1_8: PMUX1_,
    #[doc = "0xbc - Peripheral Multiplexing n - Group 1"]
    pub pmux1_12: PMUX1_,
    #[doc = "0xc0 - Pin Configuration n - Group 1"]
    pub pincfg1_0: PINCFG1_,
    #[doc = "0xc4 - Pin Configuration n - Group 1"]
    pub pincfg1_4: PINCFG1_,
    #[doc = "0xc8 - Pin Configuration n - Group 1"]
    pub pincfg1_8: PINCFG1_,
    #[doc = "0xcc - Pin Configuration n - Group 1"]
    pub pincfg1_12: PINCFG1_,
    #[doc = "0xd0 - Pin Configuration n - Group 1"]
    pub pincfg1_16: PINCFG1_,
    #[doc = "0xd4 - Pin Configuration n - Group 1"]
    pub pincfg1_20: PINCFG1_,
    #[doc = "0xd8 - Pin Configuration n - Group 1"]
    pub pincfg1_24: PINCFG1_,
    #[doc = "0xdc - Pin Configuration n - Group 1"]
    pub pincfg1_28: PINCFG1_,
}
#[doc = "Data Direction"]
pub struct DIR {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Direction"]
pub mod dir;
#[doc = "Data Direction Clear"]
pub struct DIRCLR {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Direction Clear"]
pub mod dirclr;
#[doc = "Data Direction Set"]
pub struct DIRSET {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Direction Set"]
pub mod dirset;
#[doc = "Data Direction Toggle"]
pub struct DIRTGL {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Direction Toggle"]
pub mod dirtgl;
#[doc = "Data Output Value"]
pub struct OUT {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Output Value"]
pub mod out;
#[doc = "Data Output Value Clear"]
pub struct OUTCLR {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Output Value Clear"]
pub mod outclr;
#[doc = "Data Output Value Set"]
pub struct OUTSET {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Output Value Set"]
pub mod outset;
#[doc = "Data Output Value Toggle"]
pub struct OUTTGL {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Output Value Toggle"]
pub mod outtgl;
#[doc = "Data Input Value"]
pub struct IN {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Data Input Value"]
pub mod in_;
#[doc = "Control"]
pub struct CTRL {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Control"]
pub mod ctrl;
#[doc = "Write Configuration"]
pub struct WRCONFIG {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Write Configuration"]
pub mod wrconfig;
#[doc = "Peripheral Multiplexing n - Group 0"]
pub struct PMUX0_ {
    register: ::vcell::VolatileCell<u8>,
}
#[doc = "Peripheral Multiplexing n - Group 0"]
pub mod pmux0_;
#[doc = "Peripheral Multiplexing n - Group 1"]
pub struct PMUX1_ {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Peripheral Multiplexing n - Group 1"]
pub mod pmux1_;
#[doc = "Pin Configuration n - Group 0"]
pub struct PINCFG0_ {
    register: ::vcell::VolatileCell<u8>,
}
#[doc = "Pin Configuration n - Group 0"]
pub mod pincfg0_;
#[doc = "Pin Configuration n - Group 1"]
pub struct PINCFG1_ {
    register: ::vcell::VolatileCell<u32>,
}
#[doc = "Pin Configuration n - Group 1"]
pub mod pincfg1_;
