/*
 * BldcDriver.c
 *
 * Created: 5/1/2018 10:46:23 PM
 * Author : Acshi
 */

#include <stdint.h>
#include <asf.h>
#include <fastmath.h>
#include <unistd.h>
//#include <core_cm0plus.h>

#define TRACE_BUFFER_SIZE 256
__attribute__((__aligned__(TRACE_BUFFER_SIZE * sizeof(uint32_t)))) uint32_t mtb[TRACE_BUFFER_SIZE];

#include "sam.h"

#define PUTS(s) write(1, s, sizeof(s))

#define CLOCK_FREQ 48000000
#define PWM_FREQ 24000
#define PWM_PERIOD (CLOCK_FREQ/PWM_FREQ)
#define PWM_REST (80) // min cycles for the high side mosfet-driver charge pumps to recover
#define COMP_DELAY 20 // cycles of delay after start of rest for analog comparators to start running
#define MAX_DUTY_CYCLE (PWM_PERIOD - PWM_REST)
#define MIN_DUTY_CYCLE (PWM_PERIOD / 24) // (PWM_PERIOD / 16)
// index-0 will be for 1hz
//#define SYNC_DUTY_N 7
//#define SYNC_DUTY_BY_3EHZ ((uint16_t[]){ PWM_PERIOD / 16, PWM_PERIOD / 15, PWM_PERIOD / 14, PWM_PERIOD / 13, PWM_PERIOD / 12, PWM_PERIOD / 11, PWM_PERIOD / 10})
//#define SYNC_DUTY_BY_EHZ ((uint16_t[]){ PWM_PERIOD / 12})
//#define SYNC_DUTY_BY_EHZ ((uint16_t[]){ PWM_PERIOD / 12})
//#define SYNC_DUTY_N (sizeof(SYNC_DUTY_BY_EHZ) / sizeof(SYNC_DUTY_BY_EHZ[0]))
#define MAX_DUTY_INC 50
#define MAX_DUTY_DEC 50
#define OVER_CURRENT_CHANGE 5

// synchronous speed for start up
#define SYNCHRONOUS_EHZ_INC 2
#define SYNCHRONOUS_EHZ_START (10*6)
#define SYNCHRONOUS_EHZ_END (24*6)
#define SYNCHRONOUS_FIRST_STEP_SEC 0.004
#define SYNCHRONOUS_FIRST_STEP_TICKS (PWM_FREQ * SYNCHRONOUS_FIRST_STEP_SEC)
#define SYNCHRONOUS_STEP_SEC 0.004
#define SYNCHRONOUS_LAST_STEP_SEC 0.004
#define SYNCHRONOUS_LAST_STEP_TICKS (PWM_FREQ * SYNCHRONOUS_LAST_STEP_SEC)
#define SYNCHRONOUS_STEP_TICKS (PWM_FREQ * SYNCHRONOUS_STEP_SEC)

#define MOTOR_RUNNING_CURRENT_THRESH 20

// Alignment of motor phases for start up
#define ALIGN_STEP_SEC 0.0 // seconds to assert an alignment step
#define ALIGN_STEP_TICKS (PWM_FREQ * ALIGN_STEP_SEC)

// rate at which speed measurement is updated
#define HZ_MEASUREMENT_HZ 250

#define DUTY_ADJUST_HZ 100
#define DUTY_ADJUST_TICKS (PWM_FREQ / DUTY_ADJUST_HZ)
#define MIN_MOTOR_CURRENT 10

#define MOTOR_STATUS_PIN PIN_PA22
#define MOTOR_STATUS (1 << MOTOR_STATUS_PIN)
#define MOTOR_CURRENT_ADC ADC_POSITIVE_INPUT_PIN10
#define MOTOR_V_ADC ADC_POSITIVE_INPUT_PIN11

#define MOTOR_COMP_POS_A AC_CHAN_POS_MUX_PIN0
#define MOTOR_COMP_NEG_A AC_CHAN_NEG_MUX_PIN1
#define MOTOR_COMP_POS_B AC_CHAN_POS_MUX_PIN2
#define MOTOR_COMP_NEG_B AC_CHAN_NEG_MUX_PIN3
#define MOTOR_COMP_POS_C AC_CHAN_POS_MUX_PIN0
#define MOTOR_COMP_NEG_C AC_CHAN_NEG_MUX_PIN1

#define MOTOR_ENABLE_PIN PIN_PA18
#define MOTOR_ENABLE (1 << MOTOR_ENABLE_PIN)
#define MOTOR_HA PIN_PA08
#define MOTOR_LA PIN_PA09
#define MOTOR_HB PIN_PA10
#define MOTOR_LB PIN_PA11
#define MOTOR_HC PIN_PA14
#define MOTOR_LC PIN_PA15

#define TRIGGER_OUT PIN_PA25
#define TRIGGER2_OUT PIN_PA23

#define FAULT_IN PIN_PA19

//                                                  HA/LB        HA/LC        HB/LC        HB/LA         HC/LA         HC/LB
#define STATE_TO_PWM_ENABLE         ((uint8_t[]) {  4|16,        4|16,        1|16,        1|16,         1|4,          1|4 })
#define STATE_TO_LOW_GATE_ENABLE    ((uint16_t[]) { 1<<MOTOR_LB, 1<<MOTOR_LC, 1<<MOTOR_LC, 1<<MOTOR_LA,  1<<MOTOR_LA,  1<<MOTOR_LB })
#define PWM_GATES_ALL (1|4|16)
#define LOW_GATES_ALL ((1<<11) | (1<<15) | (1<<9))

#define COIL_HIGH_GATE_DISABLE    ((uint16_t[]) { 1, 4, 16 })
#define COIL_LOW_GATE_ENABLE    ((uint16_t[]) { 1<<MOTOR_LA, 1<<MOTOR_LB, 1<<MOTOR_LC })

#define I2C_ADDR 0x10

// Communication
enum MSG_CODES {
    SET_DUTY_MSG = 1,
    READ_DUTY_MSG,
    READ_MEASURED_HZ_MSG,
    SET_CURRENT_LIMIT_MSG,
    READ_CURRENT_LIMIT_MSG,
    SET_START_DUTY_MSG,
    READ_START_DUTY_MSG,
    READ_MEASURED_CURRENT_MSG,
    READ_MEASURED_VOLTAGE_MSG,
    READ_TEMPERATURE_MSG,
    READ_ERROR_MSG,
    DIAGNOSTIC_MSG,
    ADDRESS_MSG,
    PHASE_STATE_MSG,
};


#define MSG_END_BYTE 0xed
#define VAL_END_BYTE 0xec
#define MSG_MARK_BYTE 0xfe

// 0x1800 needs to be specified to say "referenced to GND" even though we aren't in differential mode. :/
#define ADC_CURRENT_CHANNEL (0x1800 | MOTOR_CURRENT_ADC)
#define ADC_MOTOR_V_CHANNEL (0x1800 | MOTOR_V_ADC)
#define ADC_TEMP_CHANNEL (0x1800 | ADC_POSITIVE_INPUT_TEMP)
struct adc_module adc_mod;
uint8_t adc_started_channel = MOTOR_CURRENT_ADC;
uint8_t adc_next_coil_channel;
uint8_t adc_aux_i = 0;

struct ac_module ac_mod0;
struct ac_module ac_mod1;

// used for inductive position sensing
// and has a different filter alpha than motor_current_raw
volatile uint16_t startup_current_raw = 0; // raw adc value

volatile uint32_t motor_current_raw = 0; // 16-bit fixed point
volatile uint32_t motor_voltage_raw = 0; // 16-bit fixed point
volatile uint16_t temp_raw = 0;
volatile uint32_t motor_ehz_raw = 0; // 16-bit fixed point, by transition (divide by 6 for true electrical hz)
volatile uint32_t motor_ehz = 0; // by transition (divide by 6 for true electrical hz)
volatile uint32_t motor_count = 0; // state change count

volatile bool has_scheduled_tick = false; // enables the below scheduled tick
volatile uint32_t scheduled_transition_ticks = 0; // when ticks_pwm_cycles reaches this value, we increment the motor state
volatile bool has_reported_tick = false; // enables reporting scheduled ticks when they are not used/executed

#define MIN_TICKS_PER_PHASE (PWM_FREQ / SYNCHRONOUS_EHZ_START)
volatile uint32_t ticks_per_phase_raw; // 16-bit fixed point
volatile uint32_t ticks_per_phase;
volatile uint32_t last_zero_ticks;
volatile uint32_t last_zero_subticks;

#define DEBUG_BUFFER_N 128
int16_t debug_buffer1[DEBUG_BUFFER_N] = { 0 };
int16_t debug_buffer1b[DEBUG_BUFFER_N] = { 0 };
int16_t debug_buffer2[DEBUG_BUFFER_N] = { 0 };
uint16_t debug_buffer_i = 0;
uint16_t debug_buffer2_i = 0;

// for a last_ticks filter
#define LTICKS_FILTER_MIN_EHZ (10*6*7)
#define LTICKS_FILTER_MAX_EHZ (100*6*7)
#define LTICKS_FILTER_MIN_N 2
#define LTICKS_FILTER_MAX_N 32
uint32_t last_subticks_buffer[LTICKS_FILTER_MAX_N] = { 0 };
//uint16_t last_ticks_sorted[LTICKS_FILTER_N] = { 0 };

// current state of each motor, in terms of which coil is power, ground, and floating
uint8_t motor_state = 0;

volatile uint16_t current_limit = 2048;

uint16_t user_duty_cycle = 0; // this is the one directly set by commands
uint16_t target_duty_cycle = 0;
uint16_t start_duty_cycle = PWM_PERIOD / 12;

uint16_t motor_min_ehz = 0; // minimum hz for transitions in case we miss commutation, also for synchronous start up
uint32_t last_transition_ticks = 0; // for timing transitions
uint32_t last_duty_adjust_ticks = 0; // for current pid
uint32_t last_hz_ticks = 0; // for hz measurement
uint32_t sync_start_ticks = 0; // synchronous starting
bool sync_done = false;
bool align_done = false;
bool adc_current_only = false;

// incremented at whatever our pwm frequency
volatile uint32_t ticks_pwm_cycles = 0;

volatile uint32_t system_tick_counter = 0;

struct i2c_slave_module i2c_periph;
uint8_t i2c_read_buf[16];
uint8_t i2c_out_buf[16];
uint8_t i2c_out_bytes = 0;

struct tcc_module tcc_mod;

static inline void start_analog_read(uint16_t channel) {
    // The ADC runs at 2MHz, 24 times slower than the main clock
    // and most of these instructions run synchronously to the ADC clock...
    // meaning they take if they take 1 ADC cycle, that is actually 24 main cycles.. yikes!
    //adc_prepped_channel = channel;
    while (ADC->STATUS.bit.SYNCBUSY) { }
    ADC->INPUTCTRL.reg = channel;
    //while (ADC->STATUS.bit.SYNCBUSY) { }

    adc_started_channel = channel; //adc_prepped_channel;
    //while (ADC->STATUS.bit.SYNCBUSY) { }
    ADC->SWTRIG.bit.START = 1;
    //while (ADC->STATUS.bit.SYNCBUSY) { }
}

static inline uint16_t read_analog() {
    while (ADC->STATUS.bit.SYNCBUSY) { }
    return ADC->RESULT.reg;
}

static inline void start_aux_adc_read() {
    if (adc_current_only) {
        ADC->REFCTRL.bit.REFSEL = ADC_REFERENCE_INTVCC0;
        start_analog_read(ADC_CURRENT_CHANNEL);
        return;
    }

    adc_aux_i = (adc_aux_i + 1) % 3;
    if (adc_aux_i == 0) {
        // use 2.23V reference
        ADC->REFCTRL.bit.REFSEL = ADC_REFERENCE_INTVCC0;
        start_analog_read(ADC_CURRENT_CHANNEL);
    } else if (adc_aux_i == 1) {
        // use 2.23V reference
        ADC->REFCTRL.bit.REFSEL = ADC_REFERENCE_INTVCC0;
        start_analog_read(ADC_MOTOR_V_CHANNEL);
    } else {
        // use internal 1V reference
        ADC->REFCTRL.bit.REFSEL = ADC_REFERENCE_INT1V;
        start_analog_read(ADC_TEMP_CHANNEL);
    }
}

void ADC_Handler() {
    uint16_t adc_val = read_analog();

    switch (adc_started_channel) {
        case MOTOR_CURRENT_ADC:
            // on the order of 10-20Hz low pass
            motor_current_raw = ((uint64_t)motor_current_raw * 511 + (adc_val << 16) + 256) >> 9;
            // no filtering here
            startup_current_raw = adc_val;
            break;
        case MOTOR_V_ADC:
            motor_voltage_raw = ((uint64_t)motor_voltage_raw * 511 + (adc_val << 16) + 256) >> 9;
            break;
        case ADC_POSITIVE_INPUT_TEMP:
            temp_raw = adc_val;
            break;
    }

    start_aux_adc_read();
}

static void configure_adc_mux() {
    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    // ADC on MUX setting B
    config.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = 1;

    system_pinmux_pin_set_config(PIN_PB02, &config);
    system_pinmux_pin_set_config(PIN_PB03, &config);
}

static void configure_adc() {
    configure_adc_mux();

    struct adc_config adc_conf;
    adc_get_config_defaults(&adc_conf);

    adc_conf.gain_factor = ADC_GAIN_FACTOR_1X;
    adc_conf.clock_prescaler = ADC_CLOCK_PRESCALER_DIV4;
    adc_conf.reference = ADC_REFERENCE_INTVCC0; // V_cc / 1.48 ~= 2.23V
    adc_conf.positive_input = MOTOR_CURRENT_ADC;
    adc_conf.resolution = ADC_RESOLUTION_12BIT;
    adc_conf.clock_source = GCLK_GENERATOR_3;
    adc_conf.sample_length = 2;

    SYSCTRL->VREF.bit.TSEN = 1; // turn on the temperature sensor

    adc_init(&adc_mod, ADC, &adc_conf);
    adc_enable(&adc_mod);
    //adc_register_callback(&adc_mod, adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
    //adc_enable_callback(&adc_mod, ADC_CALLBACK_READ_BUFFER);

    ADC->INTENSET.bit.RESRDY = 1;
    NVIC_SetPriority(ADC_IRQn, 1); // lower priority (than pwm especially)
    NVIC_EnableIRQ(ADC_IRQn);

    //adc_start_conversion(&adc_mod);
    //adc_read_buffer_job(&adc_mod, &adc_value, 1);

    // start ADC conversions
    start_aux_adc_read();
}

static void update_motor_state();
static void delay_increment_motor_state();
static void increment_motor_state();

static void comparator_coil_a_callback(struct ac_module *const module_inst) {
    // coil a is floating in states 2 (falling below zero), 5 (rising back)
    uint8_t status = AC1->STATUSA.bit.STATE0;
    if ((motor_state == 2 && !status) || (motor_state == 5 && status)) {
        delay_increment_motor_state();
    }
}

static void comparator_coil_b_callback(struct ac_module *const module_inst) {
    // coil b is floating in states 1 (rising), 4 (falling)
    uint8_t status = AC->STATUSA.bit.STATE0;
    //if (status) {
        //PORT->Group[0].OUTSET.reg = 1 << TRIGGER_OUT;
    //} else {
        //PORT->Group[0].OUTCLR.reg = 1 << TRIGGER_OUT;
    //}
    if ((motor_state == 1 && !status) || (motor_state == 4 && status)) {
        delay_increment_motor_state();
    }
}

static void comparator_coil_c_callback(struct ac_module *const module_inst) {
    // coil c is floating in states 0 (falling), 3 (rising)
    uint8_t status = AC->STATUSA.bit.STATE1;
    if ((motor_state == 0 && !status) || (motor_state == 3 && status)) {
        delay_increment_motor_state();
    }
}

static void configure_comparators_mux() {
    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    // AC on MUX setting B
    config.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = 1;

    system_pinmux_pin_set_config(PIN_PA04, &config);
    system_pinmux_pin_set_config(PIN_PA05, &config);
    system_pinmux_pin_set_config(PIN_PA06, &config);
    system_pinmux_pin_set_config(PIN_PA07, &config);
    system_pinmux_pin_set_config(PIN_PB04, &config);
    system_pinmux_pin_set_config(PIN_PB05, &config);

    // AC1 CMP0 (coil A) out on MUX setting H
    config.mux_position = 7;
    system_pinmux_pin_set_config(PIN_PA24, &config);

    // AC0 CMP1 (coil C) out on MUX setting H
    config.mux_position = 7;
    system_pinmux_pin_set_config(PIN_PA19, &config);
}

static void configure_comparators() {
    configure_comparators_mux();

    struct ac_config ac_conf;
    ac_get_config_defaults(&ac_conf);

    // disable unnecessary voltage doubler, since VCC > 2.5
    AC->CTRLA.bit.LPMUX = 1;
    AC1->CTRLA.bit.LPMUX = 1;

    // slow down AC clock as a way of adding hysteresis
    struct system_gclk_gen_config clock_conf = { 0 };
    clock_conf.source_clock = GCLK_SOURCE_OSC8M;
    clock_conf.division_factor = 1;
    system_gclk_gen_set_config(GCLK_GENERATOR_4, &clock_conf);
    system_gclk_gen_enable(GCLK_GENERATOR_4);

    ac_conf.dig_source_generator = GCLK_GENERATOR_4;
    ac_conf.ana_source_generator = GCLK_GENERATOR_4;
    
    ac_init(&ac_mod0, AC, &ac_conf);
    ac_init(&ac_mod1, AC1, &ac_conf);

    ac_enable(&ac_mod0);
    ac_enable(&ac_mod1);

    ac_register_callback(&ac_mod1, comparator_coil_a_callback, AC_CALLBACK_COMPARATOR_0);
    ac_register_callback(&ac_mod0, comparator_coil_b_callback, AC_CALLBACK_COMPARATOR_0);
    ac_register_callback(&ac_mod0, comparator_coil_c_callback, AC_CALLBACK_COMPARATOR_1);

    ac_enable_callback(&ac_mod1, AC_CALLBACK_COMPARATOR_0);
    ac_enable_callback(&ac_mod0, AC_CALLBACK_COMPARATOR_0);
    ac_enable_callback(&ac_mod0, AC_CALLBACK_COMPARATOR_1);

    struct ac_chan_config ac_chan_coil_comp;
    // default is continuous comparisons with interrupt on compare change
    // and 50mV of hysteresis
    ac_chan_get_config_defaults(&ac_chan_coil_comp);
    //ac_chan_coil_comp.interrupt_selection = AC_CHAN_INTERRUPT_SELECTION_TOGGLE;
    ac_chan_coil_comp.interrupt_selection = AC_CHAN_INTERRUPT_SELECTION_END_OF_COMPARE;
    ac_chan_coil_comp.sample_mode = AC_CHAN_MODE_SINGLE_SHOT;
    ac_chan_coil_comp.filter = AC_CHAN_FILTER_MAJORITY_5;
    ac_chan_coil_comp.enable_hysteresis = false; // does nothing in single shot mode

    ac_chan_coil_comp.positive_input = MOTOR_COMP_POS_A;
    ac_chan_coil_comp.negative_input = MOTOR_COMP_NEG_A;
    ac_chan_coil_comp.output_mode = AC_CHAN_OUTPUT_SYNCHRONOUS; // we have an open pin to show this
    ac_chan_set_config(&ac_mod1, AC_CHAN_CHANNEL_0, &ac_chan_coil_comp);

    ac_chan_coil_comp.positive_input = MOTOR_COMP_POS_B;
    ac_chan_coil_comp.negative_input = MOTOR_COMP_NEG_B;
    ac_chan_coil_comp.output_mode = AC_CHAN_OUTPUT_INTERNAL; // but not for the other two comparators
    ac_chan_set_config(&ac_mod0, AC_CHAN_CHANNEL_0, &ac_chan_coil_comp);

    ac_chan_coil_comp.positive_input = MOTOR_COMP_POS_C;
    ac_chan_coil_comp.negative_input = MOTOR_COMP_NEG_C;
    ac_chan_coil_comp.output_mode = AC_CHAN_OUTPUT_SYNCHRONOUS; // we repurpose the fault pin to show this
    ac_chan_set_config(&ac_mod0, AC_CHAN_CHANNEL_1, &ac_chan_coil_comp);

    // high speed, higher power usage (increased bias current)
    AC1->COMPCTRL[0].bit.SPEED = 1;
    AC->COMPCTRL[0].bit.SPEED = 1;
    AC->COMPCTRL[1].bit.SPEED = 1;

    ac_chan_enable(&ac_mod1, AC_CHAN_CHANNEL_0);
    ac_chan_enable(&ac_mod0, AC_CHAN_CHANNEL_0);
    ac_chan_enable(&ac_mod0, AC_CHAN_CHANNEL_1);
}

static void set_send_reply(uint8_t *msg, uint16_t val) {
    msg[0] = MSG_MARK_BYTE;
    msg[1] = (val >> 8) & 0xff;
    msg[2] = val & 0xff;
}

// received read request, we write our response here
static void i2c_periph_read_req(struct i2c_slave_module *const module) {
    if (i2c_out_bytes >= 0) {
        struct i2c_slave_packet packet;
        packet.data_length = i2c_out_bytes;
        packet.data = i2c_out_buf;
        i2c_slave_write_packet_job(module, &packet);

        i2c_out_bytes = 0;
    }
}

// request to write to us, we read that request here
static void i2c_periph_write_req(struct i2c_slave_module *const module) {
    struct i2c_slave_packet packet;
    packet.data_length = 8;
    packet.data = i2c_read_buf;

    for (int i = 0; i < 8; i++) {
        i2c_read_buf[i] = 0;
    }

    i2c_slave_read_packet_job(module, &packet);
}

static int16_t get_corrected_temperature() {
    uint8_t *temp_cal_table = (uint8_t*)0x00806030;
    uint8_t temp_room_int = temp_cal_table[0];
    uint8_t temp_room_dec = temp_cal_table[1] & 0x0f;
    uint8_t temp_hot_int = ((temp_cal_table[2] & 0x0f) << 4) | ((temp_cal_table[1] & 0xf0) >> 4);
    uint8_t temp_hot_dec = temp_cal_table[2] & 0xf0 >> 4;
    float int1v_room = 1.0 - 0.001 * *(int8_t*)(&temp_cal_table[3]);
    float int1v_hot = 1.0 - 0.001 * *(int8_t*)(&temp_cal_table[4]);
    float adc_room = ((temp_cal_table[6] & 0x0f) << 8) | temp_cal_table[5];
    float adc_hot = (temp_cal_table[7] << 4) | ((temp_cal_table[6] & 0xf0) >> 4);

    float temp_room = (float)temp_room_int + temp_room_dec * 0.1;
    float temp_hot = (float)temp_hot_int + temp_hot_dec * 0.1;

    float res = 1.0 / ((2 << 12) - 1);

    float factor = (temp_hot - temp_room) / (adc_hot * int1v_hot * res - adc_room * int1v_room * res);
    float coarse_temp = temp_room + (temp_raw * res - adc_room * int1v_room * res) * factor;
    float int1v = int1v_room + (int1v_hot - int1v_room) * (coarse_temp - temp_room) / (temp_hot - temp_room);

    float temp = temp_room + (temp_raw * int1v * res - adc_room * int1v_room * res) * factor;
    return temp * 10;
}

// finished reading the write request, prepare the write here
static void i2c_periph_read_complete(struct i2c_slave_module *const module) {
    // For data integrity, we demand that data bytes be repeated and that message-end and value-end bytes be present and valid
    uint8_t *buf = i2c_read_buf;

    // Wait for a message of at least length 3. (Length 3 for normal command, 8 for set value command)
    bool gotMsg = (buf[0] == buf[1]) & (buf[2] == MSG_END_BYTE);
    bool gotValue = (buf[3] == buf[5]) & (buf[4] == buf[6]) & (buf[7] == VAL_END_BYTE);

    if (!gotMsg) {
        return;
    }

    int16_t value = (buf[3] << 8) | buf[4];

    i2c_out_bytes = 3; // setSendReply responds with three bytes
    switch (buf[0]) {
        case SET_DUTY_MSG:
            if (gotValue) {
                if (value > MAX_DUTY_CYCLE) {
                    value = MAX_DUTY_CYCLE;
                }
                user_duty_cycle = value;
            }
            i2c_out_bytes = 0;
            break;
        case READ_DUTY_MSG:
            set_send_reply(i2c_out_buf, target_duty_cycle);//user_duty_cycle);
            break;
        case READ_MEASURED_HZ_MSG:
            set_send_reply(i2c_out_buf, motor_ehz);
            break;
        case SET_CURRENT_LIMIT_MSG:
            if (gotValue) {
                current_limit = value;
            }
            i2c_out_bytes = 0;
            break;
        case READ_CURRENT_LIMIT_MSG:
            set_send_reply(i2c_out_buf, current_limit);
            break;
        case READ_MEASURED_CURRENT_MSG:
            set_send_reply(i2c_out_buf, motor_current_raw >> 16);
            break;
        case READ_MEASURED_VOLTAGE_MSG:
            set_send_reply(i2c_out_buf, motor_voltage_raw >> 16);
            break;
        case READ_TEMPERATURE_MSG:
            set_send_reply(i2c_out_buf, get_corrected_temperature());
            break;
        case READ_ERROR_MSG:
            set_send_reply(i2c_out_buf, 0);
            break;
        case DIAGNOSTIC_MSG:
            set_send_reply(i2c_out_buf, 'HI');
            break;
        case ADDRESS_MSG:
            set_send_reply(i2c_out_buf, I2C_ADDR);
            break;
        case PHASE_STATE_MSG:
            set_send_reply(i2c_out_buf, ticks_per_phase);
            break;
        case SET_START_DUTY_MSG:
            if (gotValue) {
                start_duty_cycle = value;
                if (start_duty_cycle < MIN_DUTY_CYCLE) {
                    start_duty_cycle = MIN_DUTY_CYCLE;
                } else if (start_duty_cycle > MAX_DUTY_CYCLE) {
                    start_duty_cycle = MAX_DUTY_CYCLE;
                }
            }
            i2c_out_bytes = 0;
            break;
        case READ_START_DUTY_MSG:
            set_send_reply(i2c_out_buf, start_duty_cycle);
            break;
        default:
            i2c_out_bytes = 0;
            break;
    }
}

static void configure_i2c_mux() {
    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    // SERCOM1 on MUX setting C
    config.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = 2;

    system_pinmux_pin_set_config(PIN_PA16, &config);
    system_pinmux_pin_set_config(PIN_PA17, &config);
}

static void configure_i2c() {
    configure_i2c_mux();

    struct i2c_slave_config config;
    i2c_slave_get_config_defaults(&config);
    config.address = I2C_ADDR;
    //config.generator_source = GCLK_GENERATOR_3;

    i2c_slave_init(&i2c_periph, SERCOM1, &config);
    i2c_slave_enable(&i2c_periph);
    i2c_slave_register_callback(&i2c_periph, i2c_periph_read_req, I2C_SLAVE_CALLBACK_READ_REQUEST);
    i2c_slave_register_callback(&i2c_periph, i2c_periph_write_req, I2C_SLAVE_CALLBACK_WRITE_REQUEST);
    i2c_slave_register_callback(&i2c_periph, i2c_periph_read_complete, I2C_SLAVE_CALLBACK_READ_COMPLETE);
    i2c_slave_enable_callback(&i2c_periph, I2C_SLAVE_CALLBACK_READ_REQUEST);
    i2c_slave_enable_callback(&i2c_periph, I2C_SLAVE_CALLBACK_WRITE_REQUEST);
    i2c_slave_enable_callback(&i2c_periph, I2C_SLAVE_CALLBACK_READ_COMPLETE);

    //SERCOM1->I2CS.INTENSET.bit.AMATCH = 1;
    //NVIC_EnableIRQ(SERCOM1_IRQn);
    NVIC_SetPriority(SERCOM1_IRQn, 1); // lower priority (than pwm especially)
}

static void configure_gpio_mux() {
    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    config.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = SYSTEM_PINMUX_GPIO;
    config.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT_WITH_READBACK;

    system_pinmux_pin_set_config(MOTOR_STATUS_PIN, &config);
    system_pinmux_pin_set_config(MOTOR_ENABLE_PIN, &config);

    // the low enables are either on or off, pwm is only given to the high enables
    system_pinmux_pin_set_config(MOTOR_LA, &config);
    system_pinmux_pin_set_config(MOTOR_LB, &config);
    system_pinmux_pin_set_config(MOTOR_LC, &config);

    // debugging trigger output
    system_pinmux_pin_set_config(TRIGGER_OUT, &config);
    system_pinmux_pin_set_config(TRIGGER2_OUT, &config);

    //config.input_pull = SYSTEM_PINMUX_PIN_PULL_UP;
    //config.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
    //system_pinmux_pin_set_config(FAULT_IN, &config);
}

ISR(TCC0_Handler) {
    TCC_INTFLAG_Type intflag = TCC0->INTFLAG;
    if (intflag.bit.OVF) {
        ticks_pwm_cycles++;
        if (has_scheduled_tick && ticks_pwm_cycles >= scheduled_transition_ticks) {
            if (!has_reported_tick) {
                has_reported_tick = true;
                PORT->Group[0].OUTTGL.reg = 1 << TRIGGER_OUT;
            }
            if (sync_done) {
                increment_motor_state();
            }
        }
    }
    // time to check for zero crossing
    if (intflag.bit.MC1) {
        switch (motor_state) {
            case 0:
            case 3:
                // coil c
                ac_chan_trigger_single_shot(&ac_mod0, AC_CHAN_CHANNEL_1);
                break;
            case 1:
            case 4:
                // coil b
                ac_chan_trigger_single_shot(&ac_mod0, AC_CHAN_CHANNEL_0);
                break;
            case 2:
            case 5:
                // coil a
                ac_chan_trigger_single_shot(&ac_mod1, AC_CHAN_CHANNEL_0);
                break;
        }
    }
    TCC0->INTFLAG.reg = intflag.reg; // clear all bits
}

static void configure_pwm_mux() {
    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    config.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = 4; // E
    system_pinmux_pin_set_config(MOTOR_HA, &config);

    config.mux_position = 5; // F
    system_pinmux_pin_set_config(MOTOR_HB, &config);
    system_pinmux_pin_set_config(MOTOR_HC, &config);
}

static void configure_pwm() {
    configure_gpio_mux();
    configure_pwm_mux();

    struct tcc_config config;
    tcc_get_config_defaults(&config, TCC0);
    config.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
    config.counter.period = PWM_PERIOD;
    config.wave.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

    //config.double_buffering_enabled = true;
    tcc_init(&tcc_mod, TCC0, &config);
    TCC0->WEXCTRL.bit.OTMX = 1; // assigns output of control 0 to HA, HB, and HC
    // output matrix (OTMX) must be set before enabling
    tcc_enable(&tcc_mod);

    // we override 2 of these at all times to be held low, with the "pattern generation"
    // and using a fixed pattern of "low"
    TCC0->PATT.reg = 0;
    TCC0->PATT.vec.PGE = PWM_GATES_ALL; // all overridden to low

    // enable interrupts for compare match on channels 0, 1, and overflow/reset
    TCC0->INTENSET.bit.OVF = 1;
    TCC0->INTENSET.bit.MC1 = 1;

    NVIC_EnableIRQ(TCC0_IRQn);
}

static void update_motor_state() {
    if (target_duty_cycle == 0) {
        // turn everything off
        PORT->Group[0].OUTCLR.reg = MOTOR_ENABLE | LOW_GATES_ALL;
        TCC0->PATT.vec.PGE = PWM_GATES_ALL;
        return;
    }

    // modify the pattern (of lows/zeros) override to
    // update which output (of HA, HB, HC) the pwm is directed to
    // so we specify which two of the three we want zeroed, leaving the third to take pwm
    TCC0->PATT.vec.PGE = STATE_TO_PWM_ENABLE[motor_state];
    PORT->Group[0].OUTCLR.reg = LOW_GATES_ALL;
    PORT->Group[0].OUTSET.reg = MOTOR_ENABLE | STATE_TO_LOW_GATE_ENABLE[motor_state];
}

// how big of a filter should we use for our current speed?
// linear interpolation between min and max
static uint8_t lticks_filter_size() {
    if (motor_ehz < LTICKS_FILTER_MIN_EHZ) {
        return LTICKS_FILTER_MIN_N;
    } else if (motor_ehz > LTICKS_FILTER_MAX_EHZ) {
        return LTICKS_FILTER_MAX_N;
    }
    return 3 + (LTICKS_FILTER_MAX_N - LTICKS_FILTER_MIN_N)
               * (motor_ehz - LTICKS_FILTER_MIN_EHZ)
               / (LTICKS_FILTER_MAX_EHZ - LTICKS_FILTER_MIN_EHZ);
    //return 9;
}

static void delay_increment_motor_state() {
    if (!has_scheduled_tick) {
        // read our sub-tick position 
        TCC0->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
        while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB) { }
        uint32_t now_subticks = TCC0->COUNT.reg;

        uint32_t now_ticks = ticks_pwm_cycles;
        int32_t full_ticks_change = now_ticks - last_zero_ticks;
        int32_t subticks_change = (int32_t)now_subticks - (int32_t)last_zero_subticks;
        uint32_t total_subticks_change = full_ticks_change * PWM_PERIOD + subticks_change;

        // rotate in new value
        for (uint8_t i = 0; i < LTICKS_FILTER_MAX_N - 1; i++) {
            last_subticks_buffer[i] = last_subticks_buffer[i + 1];
        }
        last_subticks_buffer[LTICKS_FILTER_MAX_N - 1] = total_subticks_change;

        uint8_t filter_size = lticks_filter_size();
        int32_t subticks_sum = 0;
        for (uint8_t i = LTICKS_FILTER_MAX_N - filter_size; i < LTICKS_FILTER_MAX_N; i++) {
            subticks_sum += last_subticks_buffer[i];
        }

        uint32_t subticks_per_phase = subticks_sum / filter_size;
        // advance the time interval to be 22.5 degrees early
        subticks_per_phase = ((subticks_per_phase * 15) >> 4);

        ticks_per_phase = subticks_per_phase / PWM_PERIOD;
        scheduled_transition_ticks = last_transition_ticks + ticks_per_phase;
        has_scheduled_tick = true;
        has_reported_tick = false;
        last_zero_ticks = now_ticks;
        last_zero_subticks = now_subticks;
        //PORT->Group[0].OUTTGL.reg = 1 << TRIGGER_OUT;
    }
}

static void increment_motor_state() {
    PORT->Group[0].OUTTGL.reg = 1 << TRIGGER2_OUT;
    motor_state = (motor_state + 1) % 6;
    update_motor_state();

    motor_count++;
    last_transition_ticks = ticks_pwm_cycles;
    has_scheduled_tick = false;
}

static void update_motor() {
    uint16_t motor_current = motor_current_raw >> 16;
    bool restart_sync = sync_done && (motor_current < MIN_MOTOR_CURRENT || motor_ehz < SYNCHRONOUS_EHZ_START);
    if (((user_duty_cycle == 0 || current_limit == 0) && (!sync_done || target_duty_cycle <= MIN_DUTY_CYCLE)) || restart_sync) {
        if (restart_sync) {
            __asm("nop");
        }

        // turn off all the outputs
        target_duty_cycle = 0;
        tcc_set_compare_value(&tcc_mod, 0, 0);
        update_motor_state();

        // turn off status light
        PORT->Group[0].OUTCLR.reg = MOTOR_STATUS;

        // reset state to not-moving
        motor_ehz_raw = 0;
        motor_ehz = 0;
        align_done = false;
        sync_done = false;
        motor_min_ehz = 0;
        last_duty_adjust_ticks = 0;
        return;
    }

    // turn status light on
    PORT->Group[0].OUTSET.reg = MOTOR_STATUS;

    // make sure we have locked onto the initial rotor position
    if (!align_done) {
        //determine_rotor_position();

        align_done = true;
        sync_start_ticks = ticks_pwm_cycles;
        motor_min_ehz = SYNCHRONOUS_EHZ_START;
        target_duty_cycle = start_duty_cycle;
    }

    // manage our target duty cycle, and thus the rate at which the motor changes velocity
    volatile uint32_t now_ticks = ticks_pwm_cycles;
    bool time_to_adjust = now_ticks - last_duty_adjust_ticks > DUTY_ADJUST_TICKS;
    if ((current_limit > 0 || target_duty_cycle > MIN_DUTY_CYCLE) && time_to_adjust) {
        if (sync_done) {
            int16_t diff = (int16_t)user_duty_cycle - (int16_t)target_duty_cycle;
            if (diff > MAX_DUTY_INC) {
                diff = MAX_DUTY_INC;
            }
            if (diff < -MAX_DUTY_DEC) {
                diff = -MAX_DUTY_DEC;
            }
            target_duty_cycle += diff;

            if (target_duty_cycle < MIN_DUTY_CYCLE) {
                target_duty_cycle = MIN_DUTY_CYCLE;
            }
            if (target_duty_cycle > MAX_DUTY_CYCLE) {
                target_duty_cycle = MAX_DUTY_CYCLE;
            }
        }
        if (motor_current >= current_limit) {
            user_duty_cycle -= OVER_CURRENT_CHANGE;
            target_duty_cycle -= OVER_CURRENT_CHANGE;
            if (user_duty_cycle < 0) {
                user_duty_cycle = 0;
            }
        }
        // sets high-side duty cycle
        tcc_set_compare_value(&tcc_mod, 0, target_duty_cycle);
        // time the check on the appropriate comparator during the off part of the period
        tcc_set_compare_value(&tcc_mod, 1, MAX_DUTY_CYCLE / 2 + COMP_DELAY);
        last_duty_adjust_ticks = now_ticks;
    }

    // asynchronous commutation managed by the comparator callbacks
    if (sync_done) {
        return;
    }

    uint32_t step_ticks = (motor_min_ehz == SYNCHRONOUS_EHZ_START) ? SYNCHRONOUS_FIRST_STEP_TICKS : SYNCHRONOUS_STEP_TICKS;
    if (motor_min_ehz + SYNCHRONOUS_EHZ_INC >= SYNCHRONOUS_EHZ_END) {
        step_ticks = SYNCHRONOUS_LAST_STEP_TICKS;
    }
    if (now_ticks - sync_start_ticks > step_ticks) {
        if ((motor_current_raw >> 16) > MOTOR_RUNNING_CURRENT_THRESH) {
            // condition above to make sure the motor has started turning
            // otherwise we stay in slow synchronous state for debugging
            motor_min_ehz = motor_min_ehz + SYNCHRONOUS_EHZ_INC;
            if (motor_min_ehz >= SYNCHRONOUS_EHZ_END) {
                motor_min_ehz = SYNCHRONOUS_EHZ_END;
                sync_done = true;
                motor_min_ehz = SYNCHRONOUS_EHZ_START;
            }
        }
        sync_start_ticks = now_ticks;
    }

    uint16_t ticks_per_transition = (PWM_FREQ + motor_min_ehz / 2) / motor_min_ehz;
    if ((now_ticks - last_transition_ticks) >= ticks_per_transition) {
        increment_motor_state();
    }
}

static void update_speed_estimate() {
    uint32_t now_ticks = ticks_pwm_cycles;
    if ((now_ticks - last_hz_ticks) >= PWM_FREQ / HZ_MEASUREMENT_HZ) {
        uint32_t new_ehz = (motor_count * HZ_MEASUREMENT_HZ + 3);
        motor_ehz_raw = ((uint64_t)motor_ehz_raw * 31 + (new_ehz << 16) + 15) >> 5;
        motor_ehz = motor_ehz_raw >> 16;
        motor_count = 0;
        last_hz_ticks = now_ticks;
    }
}

ISR(HardFault_Handler) {
    // turn off all the outputs
    target_duty_cycle = 0;
    tcc_set_compare_value(&tcc_mod, 0, 0);
    update_motor_state();

    // turn off status light
    PORT->Group[0].OUTCLR.reg = MOTOR_STATUS;

    // Turn off the micro trace buffer so we don't fill it up in the infinite
    // loop below.
    REG_MTB_MASTER = 0x00000000 + 6;
    while (1) {
        __asm("nop");
    }
}

extern void initialise_monitor_handles();

ISR(SysTick_Handler) {
    system_tick_counter++;
}

int main() {
    // Initialize micro-trace buffer
    REG_MTB_POSITION = ((uint32_t) (mtb - REG_MTB_BASE)) & 0xFFFFFFF8;
    REG_MTB_FLOW = ((uint32_t) mtb + TRACE_BUFFER_SIZE * sizeof(uint32_t)) & 0xFFFFFFF8;
    REG_MTB_MASTER = 0x80000000 + 6;

    system_init();

    //SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    //SysTick->LOAD = SysTick->CALIB; // ms timer

    //initialise_monitor_handles();
    //PUTS("BLDC Driver startup\n");

    configure_adc();
    configure_comparators();

    configure_i2c();
    configure_pwm();

    volatile uint32_t update_count = 0;
    while (1) {
        volatile uint16_t update_rate = (uint64_t)update_count * PWM_FREQ / ticks_pwm_cycles;
        update_motor();
        update_speed_estimate();
        update_count++;
    }
    return 0;
}
