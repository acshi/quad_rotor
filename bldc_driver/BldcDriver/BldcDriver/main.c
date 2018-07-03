/*
 * BldcDriver.c
 *
 * Created: 5/1/2018 10:46:23 PM
 * Author : Acshi
 */

#include <stdint.h>
#include <asf.h>
#include <fastmath.h>

#include "sam.h"
#include "pid_ctrl.h"

#define CLOCK_FREQ 48000000
#define PWM_FREQ 24000 //5600
#define PWM_PERIOD (CLOCK_FREQ/PWM_FREQ)
#define PWM_REST (80)
#define MAX_DUTY_CYCLE (PWM_PERIOD - PWM_REST)
#define MIN_DUTY_CYCLE (PWM_PERIOD / 15)
#define MAX_DUTY_CHANGE 20

// synchronous speed for start up
#define SYNCHRONOUS_HZ 3
#define SYNCHRONOUS_STEP_SEC 0.2 // seconds to stay in synchronous mode for startup
#define SYNCHRONOUS_STEP_TICKS (PWM_FREQ * SYNCHRONOUS_STEP_SEC)

// rate at which speed measurement is updated
#define HZ_MEASUREMENT_HZ 10

#define HZ_ADJUST_HZ 10
#define HZ_ADJUST_TICKS (PWM_FREQ / HZ_ADJUST_HZ)
#define CURRENT_ADJUST_HZ 40
#define CURRENT_ADJUST_TICKS (PWM_FREQ / CURRENT_ADJUST_HZ)
#define MIN_MOTOR_CURRENT 10

#define MOTOR0_STATUS PIN_PA09
#define MOTOR1_STATUS PIN_PA08

#define MOTOR0_CURRENT_ADC ADC_POSITIVE_INPUT_PIN1
#define MOTOR0_A_ADC ADC_POSITIVE_INPUT_PIN10
#define MOTOR0_B_ADC ADC_POSITIVE_INPUT_PIN11
#define MOTOR0_C_ADC ADC_POSITIVE_INPUT_PIN0

#define MOTOR1_CURRENT_ADC ADC_POSITIVE_INPUT_PIN12
#define MOTOR1_A_ADC ADC_POSITIVE_INPUT_PIN7
#define MOTOR1_B_ADC ADC_POSITIVE_INPUT_PIN6
#define MOTOR1_C_ADC ADC_POSITIVE_INPUT_PIN13

#define MOTOR0_ENABLE_A PIN_PA25
#define MOTOR0_ENABLE_B PIN_PA24
#define MOTOR0_ENABLE_C PIN_PA18

#define MOTOR1_ENABLE_A PIN_PA15
#define MOTOR1_ENABLE_B PIN_PA14
#define MOTOR1_ENABLE_C PIN_PA11

#define ENABLE_I_TO_MASK ((uint32_t[]){ 1 << 25, 1 << 24, 1 << 18, 1 << 15, 1 << 14, 1 << 11 })

#define MOTOR0_OUT_A PIN_PA23
#define MOTOR0_OUT_B PIN_PA22
#define MOTOR0_OUT_C PIN_PA19

#define MOTOR1_OUT_A PIN_PA10
#define MOTOR1_OUT_B PIN_PA05
#define MOTOR1_OUT_C PIN_PA04

#define MOTOR_I_TO_MASK ((uint32_t[]){ 1 << 23, 1 << 22, 1 << 19, 1 << 10, 1 << 5, 1 << 4 })
#define STATE_TO_FLOATING ((uint8_t[]){ 2, 1, 0, 2, 1, 0 })

// 0x1800 needs to be specified to say "referenced to GND" even though we aren't in differential mode. :/
#define MOTOR0_STATE_TO_FLOATING_ADC ((uint16_t[]){ 0x1800 | MOTOR0_C_ADC, 0x1800 | MOTOR0_B_ADC, 0x1800 | MOTOR0_A_ADC, \
                                                    0x1800 | MOTOR0_C_ADC, 0x1800 | MOTOR0_B_ADC, 0x1800 | MOTOR0_A_ADC })
#define MOTOR1_STATE_TO_FLOATING_ADC ((uint16_t[]){ 0x1800 | MOTOR1_C_ADC, 0x1800 | MOTOR1_B_ADC, 0x1800 | MOTOR1_A_ADC, \
                                                    0x1800 | MOTOR1_C_ADC, 0x1800 | MOTOR1_B_ADC, 0x1800 | MOTOR1_A_ADC })

// 0-2 abc of motor 1, 3-5 abc of motor 2
void motor_enable(int idx) {
    //PORT->Group[0].DIRCLR.reg = 1 << pin_num; // to input
    //PORT->Group[0].PINCFG[pin_num].bit.PULLEN = 1; // w/ pull-up
    PORT->Group[0].OUTSET.reg = ENABLE_I_TO_MASK[idx]; // high
}

void motor_disable(int idx) {
    //PORT->Group[0].DIRSET.reg = 1 << pin_num; // to output
    PORT->Group[0].OUTCLR.reg = ENABLE_I_TO_MASK[idx]; // low
}

// 0-2 abc of motor 1, 3-5 abc of motor 2
void motor_set(int idx) {
    PORT->Group[0].OUTSET.reg = MOTOR_I_TO_MASK[idx]; // high
}

void motor_clear(int idx) {
    PORT->Group[0].OUTCLR.reg = MOTOR_I_TO_MASK[idx]; // low
}

#define I2C_ADDR 0x10

// each motor only has one coil than needs active pwm.
// one more coil will be ground, and one coil will float.
#define MOTOR0_PWM_CHANNEL 0
#define MOTOR1_PWM_CHANNEL 1

// Communication
#define MOTOR1_MASK (1 << 6) // indicates to target motor1/second motor instead of motor0.
#define SET_TARGET_HZ_MSG 1
#define READ_TARGET_HZ_MSG 2
#define READ_MEASURED_HZ_MSG 3
#define SET_TARGET_CURRENT_MSG 4
#define READ_TARGET_CURRENT_MSG 5
#define READ_MEASURED_CURRENT_MSG 6
#define READ_TEMPERATURE_MSG 7
#define READ_ERROR_MSG 8
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
#define MSG_END_BYTE 0xed
#define VAL_END_BYTE 0xec
#define MSG_MARK_BYTE 0xfe

#define ADC_CURRENT_CHANNEL ((uint16_t[]){ 0x1800 | MOTOR0_CURRENT_ADC, 0x1800 | MOTOR1_CURRENT_ADC})
#define ADC_TEMP_CHANNEL (0x1800 | ADC_POSITIVE_INPUT_TEMP)
struct adc_module adc_mod;
uint8_t adc_started_channel = MOTOR0_CURRENT_ADC;
//uint16_t adc_prepped_channel;
uint8_t adc_next_coil_channel;
uint8_t adc_aux_i = 0;

uint8_t adc_motor_on = 0;
uint16_t adc_motor0_float_coil = 0;
uint16_t adc_motor1_float_coil = 0;

// the adc readings from coils A, B, C
//volatile uint16_t motor0_coils[3];
//volatile uint16_t motor1_coils[3];
volatile uint16_t motor_coil[2];
volatile uint32_t motor_current_raw[2]; // 16-bit fixed point
volatile uint32_t motor_current_combined_raw[2]; // 16-bit fixed point
uint16_t motor_current[2]; // same units as are communicated over i2c
volatile uint16_t temp;
volatile uint32_t motor_hz[2];
volatile uint32_t motor_count[2]; // state change count

#define COIL_BUFFER_N 1024
int16_t coil_buffer[COIL_BUFFER_N] = { 0 };
int16_t coil_states[COIL_BUFFER_N] = { 0 };
uint16_t coil_buffer_i = 0;

// current state of each motor, in terms of which coil is power, ground, and floating
uint8_t motor_state[2] = { 0 };

// the bits to set on pwm overflow/reset
uint32_t motor_pwm_bits = 0;

// whether the following settings need to be applied on the next pwm pulse
volatile bool new_motor_bits = false;
// bits to set/clear on next pwm pulse
uint32_t motor_enable_bits = 0;
uint32_t motor_disable_clear_bits = 0;
// update 
uint32_t motor_pwm_bits_next = 0;

volatile uint16_t target_hz[2] = { 0, 0 };
volatile uint16_t target_current[2] = { 0, 0 };

uint16_t target_duty_cycle[2] = { 0, 0 };
//uint16_t duty_cycle_on_time = 0;
//uint32_t duty_cycle_measured[2] = { 0 };
int32_t sense_coil_min[2] = { 0 };
int32_t sense_coil_max[2] = { 0 };
bool coil_high[2] = { false };
uint16_t adc_measurement_time[2] = { 0 };

uint16_t motor_min_hz[2] = { 0 }; // minimum hz for transitions in case we miss commutation, also for synchronous start up
uint32_t last_transition_ticks[2] = { 0 }; // for timing transitions
uint32_t last_hz_adjust_ticks[2] = { 0 }; // for hz pid
uint32_t last_current_adjust_ticks[2] = { 0 }; // for current pid
uint32_t last_hz_ticks[2] = { 0 }; // for hz measurement
uint32_t sync_start_ticks[2] = { 0 }; // synchronous starting
bool sync_done[2] = { false, false };
uint32_t last_zero_ticks[2] = { 0 };
uint32_t state_change_interval[2] = { 0 }; // average ticks between state changes, 16-bit fixed point to allow filtering/smoothing
bool change_after_delay[2] = { false };
uint32_t average_coil[2] = { 0 }; // average value of floating coil, for determining threshold to use, 16-bit fixed point

pid_ctrl_t current_pid[2];
pid_ctrl_t hz_pid[2];

// incremented at whatever our pwm frequency
volatile uint32_t ticks_pwm_cycles = 0;

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

static inline void set_analog_channel(uint16_t channel) {
    //adc_prepped_channel = channel;
    //while (ADC->STATUS.bit.SYNCBUSY) { }
    ADC->INPUTCTRL.reg = channel;
    //while (ADC->STATUS.bit.SYNCBUSY) { }
}

static inline uint16_t read_analog() {
    while (ADC->STATUS.bit.SYNCBUSY) { }
    return ADC->RESULT.reg;
}

static inline void set_aux_adc_compare(uint16_t count) {
    //while(TCC0->SYNCBUSY.bit.CC2) { }
	TCC0->CC[2].reg = count;
}

static inline void start_aux_adc_read() {
    adc_aux_i = (adc_aux_i + 1) % 4;
    if (adc_aux_i < 2) {
        start_analog_read(ADC_CURRENT_CHANNEL[adc_motor_on]);
    } else {
        start_analog_read(ADC_TEMP_CHANNEL);
    }
    adc_motor_on = (adc_motor_on + 1) % 2;
    set_aux_adc_compare(adc_measurement_time[adc_motor_on]);
}

static inline void start_coil_adc_read() {
    uint16_t adc_channel = (adc_motor_on == 0) ? adc_motor0_float_coil : adc_motor1_float_coil;
    start_analog_read(adc_channel);
}

void ADC_Handler() {
    static uint32_t adc_count = 0;
    static volatile uint16_t adc_rate;

    bool got_coil_value = false;
    uint16_t adc_val;
    uint16_t last_adc_channel = adc_started_channel;

    switch (adc_started_channel) {
        case MOTOR0_A_ADC:
        case MOTOR0_B_ADC:
        case MOTOR0_C_ADC:
            start_aux_adc_read();
            adc_val = read_analog();
            if (ticks_pwm_cycles - last_transition_ticks[0] >= state_change_interval[0] >> 19) {
                motor_coil[0] = adc_val;
                coil_buffer[coil_buffer_i] = motor_coil[0];
                coil_buffer_i = (coil_buffer_i + 1) % COIL_BUFFER_N;
                coil_states[coil_buffer_i] = last_adc_channel;
            }
            got_coil_value = true;
            break;
        case MOTOR1_A_ADC:
        case MOTOR1_B_ADC:
        case MOTOR1_C_ADC:
            start_aux_adc_read();
            adc_val = read_analog();
            if (ticks_pwm_cycles - last_transition_ticks[1] >= state_change_interval[1] >> 18) {
                motor_coil[1] = adc_val;
                //coil_buffer[coil_buffer_i] = motor_coil[1];
                //coil_buffer_i = (coil_buffer_i + 1) % COIL_BUFFER_N;
                //coil_states[coil_buffer_i] = last_adc_channel;
            }
            got_coil_value = true;
            break;
        case MOTOR0_CURRENT_ADC:
            adc_val = read_analog();
            motor_current_raw[0] = ((uint64_t)motor_current_raw[0] * 2047 + (adc_val << 16) + 1024) >> 11;
            //coil_buffer[coil_buffer_i] = adc_val;
            //coil_buffer_i = (coil_buffer_i + 1) % COIL_BUFFER_N;
            break;
        case MOTOR1_CURRENT_ADC:
            adc_val = read_analog();
            motor_current_raw[1] = ((uint64_t)motor_current_raw[1] * 511 + (adc_val << 16) + 256) >> 9;
            //coil_buffer[coil_buffer_i] = adc_val;
            //coil_buffer_i = (coil_buffer_i + 1) % COIL_BUFFER_N;
            break;
        case ADC_POSITIVE_INPUT_TEMP:
            temp = read_analog();
            break;
    }

    if (got_coil_value) {
        adc_count++;
        adc_rate = (uint32_t)adc_count * PWM_FREQ / ticks_pwm_cycles;
        adc_rate;
    }
}

static void configure_adc_mux() {
    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    // ADC on MUX setting B
    config.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = 1;

    system_pinmux_pin_set_config(PIN_PA02, &config);
    system_pinmux_pin_set_config(PIN_PA03, &config);
    system_pinmux_pin_set_config(PIN_PA06, &config);
    system_pinmux_pin_set_config(PIN_PA07, &config);
    system_pinmux_pin_set_config(PIN_PB02, &config);
    system_pinmux_pin_set_config(PIN_PB03, &config);
    system_pinmux_pin_set_config(PIN_PB04, &config);
    system_pinmux_pin_set_config(PIN_PB05, &config);
}

static void configure_adc() {
    configure_adc_mux();

    struct adc_config adc_conf;
    adc_get_config_defaults(&adc_conf);

    adc_conf.gain_factor = ADC_GAIN_FACTOR_1X;
    adc_conf.clock_prescaler = ADC_CLOCK_PRESCALER_DIV4;
    adc_conf.reference = ADC_REFERENCE_INTVCC0; // V_cc / 1.48 ~= 2.23V
    adc_conf.positive_input = MOTOR0_CURRENT_ADC;
    adc_conf.resolution = ADC_RESOLUTION_12BIT;
    adc_conf.clock_source = GCLK_GENERATOR_3;

    adc_init(&adc_mod, ADC, &adc_conf);
    adc_enable(&adc_mod);
    //adc_register_callback(&adc_mod, adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
    //adc_enable_callback(&adc_mod, ADC_CALLBACK_READ_BUFFER);

    ADC->INTENSET.bit.RESRDY = 1;
    NVIC_SetPriority(ADC_IRQn, 1); // lower priority (than pwm especially)
    NVIC_EnableIRQ(ADC_IRQn);

    //adc_start_conversion(&adc_mod);
    // start ADC conversions
    //adc_read_buffer_job(&adc_mod, &adc_value, 1);
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

    int motor = (buf[0] & MOTOR1_MASK) ? 1 : 0;
    buf[0] &= ~MOTOR1_MASK;

    i2c_out_bytes = 3; // setSendReply responds with three bytes
    switch (buf[0]) {
        case SET_TARGET_HZ_MSG:
            if (gotValue) {
                target_hz[motor] = value;
                pid_set_setpoint(&hz_pid[motor], target_hz[motor]);
            }
            i2c_out_bytes = 0;
            break;
        case READ_TARGET_HZ_MSG:
            set_send_reply(i2c_out_buf, target_hz[motor]);
            break;
        case READ_MEASURED_HZ_MSG:
            set_send_reply(i2c_out_buf, motor_hz[motor]);
            break;
        case SET_TARGET_CURRENT_MSG:
            if (gotValue) {
                target_current[motor] = value;
                pid_set_setpoint(&current_pid[motor], target_current[motor]);
            }
            i2c_out_bytes = 0;
            break;
        case READ_TARGET_CURRENT_MSG:
            set_send_reply(i2c_out_buf, target_current[motor]);
            break;
        case READ_MEASURED_CURRENT_MSG:
            set_send_reply(i2c_out_buf, motor_current[motor]);
            break;
        case READ_TEMPERATURE_MSG:
            set_send_reply(i2c_out_buf, temp);
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

    system_pinmux_pin_set_config(MOTOR0_ENABLE_A, &config);
    system_pinmux_pin_set_config(MOTOR0_ENABLE_B, &config);
    system_pinmux_pin_set_config(MOTOR0_ENABLE_C, &config);
    system_pinmux_pin_set_config(MOTOR1_ENABLE_A, &config);
    system_pinmux_pin_set_config(MOTOR1_ENABLE_B, &config);
    system_pinmux_pin_set_config(MOTOR1_ENABLE_C, &config);

    system_pinmux_pin_set_config(MOTOR0_OUT_A, &config);
    system_pinmux_pin_set_config(MOTOR0_OUT_B, &config);
    system_pinmux_pin_set_config(MOTOR0_OUT_C, &config);
    system_pinmux_pin_set_config(MOTOR1_OUT_A, &config);
    system_pinmux_pin_set_config(MOTOR1_OUT_B, &config);
    system_pinmux_pin_set_config(MOTOR1_OUT_C, &config);

    system_pinmux_pin_set_config(MOTOR0_STATUS, &config);
    system_pinmux_pin_set_config(MOTOR1_STATUS, &config);

    //system_pinmux_group_set_output_strength(&PORT->Group[0], 0xffffffff, SYSTEM_PINMUX_PIN_STRENGTH_HIGH);
}

void TCC0_Handler() {
    TCC_INTFLAG_Type intflag = TCC0->INTFLAG;
    if (intflag.bit.OVF) {
        if (new_motor_bits) {
            new_motor_bits = false;
            motor_pwm_bits = motor_pwm_bits_next;
            PORT->Group[0].OUTSET.reg = motor_enable_bits;
            PORT->Group[0].OUTCLR.reg = motor_disable_clear_bits;
        }

        PORT->Group[0].OUTSET.reg = motor_pwm_bits;
        //TCC0->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
        //while (TCC0->SYNCBUSY.bit.CTRLB) { }
        //duty_cycle_on_time = TCC0->COUNT.reg;
        ticks_pwm_cycles++;
    }
    // counter match motor 0
    if (intflag.bit.MC0) {
        uint32_t pin_mask = MOTOR_I_TO_MASK[0] | MOTOR_I_TO_MASK[1] | MOTOR_I_TO_MASK[2];
        PORT->Group[0].OUTCLR.reg = pin_mask; // clear
        //TCC0->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
        //while (TCC0->SYNCBUSY.bit.CTRLB) { }
        //uint16_t count = TCC0->COUNT.reg;
        //uint16_t measured_time = (duty_cycle_on_time <= count) ? TCC0->COUNT.reg - duty_cycle_on_time : 0;
        //duty_cycle_measured[0] = (255 * duty_cycle_measured[0] + (measured_time << 16) + 128) >> 8;
    }
    // counter match motor 1
    if (intflag.bit.MC1) {
        uint32_t pin_mask = MOTOR_I_TO_MASK[3] | MOTOR_I_TO_MASK[4] | MOTOR_I_TO_MASK[5];
        PORT->Group[0].OUTCLR.reg = pin_mask; // clear
        //TCC0->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
        //while (TCC0->SYNCBUSY.bit.CTRLB) { }
        //volatile uint16_t count = TCC0->COUNT.reg;
        //volatile uint16_t measured_time = (duty_cycle_on_time <= count) ? TCC0->COUNT.reg - duty_cycle_on_time : 0;
        //duty_cycle_measured[1] = (15 * duty_cycle_measured[1] + (measured_time << 16)) >> 4;
    }
    // match 2, right before pwm goes high again, for floating coil measurements
    if (intflag.bit.MC2) {
        start_coil_adc_read();
    }
    TCC0->INTFLAG.reg = intflag.reg; // clear all bits
}

static void configure_pwm() {
    struct tcc_config config;
    tcc_get_config_defaults(&config, TCC0);
    config.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
    config.counter.period = PWM_PERIOD;
    config.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

    //config.double_buffering_enabled = true;
    tcc_init(&tcc_mod, TCC0, &config);
    tcc_enable(&tcc_mod);

    // enable interrupts for compare match on channels 0, 1, and overflow/reset
    TCC0->INTENSET.bit.OVF = 1;
    TCC0->INTENSET.bit.MC0 = 1;
    TCC0->INTENSET.bit.MC1 = 1;
    TCC0->INTENSET.bit.MC2 = 1;

    NVIC_EnableIRQ(TCC0_IRQn);

    configure_gpio_mux();

    // start up these comparisons, as the compare values update in the interrupt handler
    adc_measurement_time[0] = PWM_PERIOD - 200;
    adc_measurement_time[1] = PWM_PERIOD - 200;
    set_aux_adc_compare(PWM_PERIOD - 200);
    //target_current[0] = 1000;
    //target_duty_cycle[0] = 1000;
    //tcc_set_compare_value(&tcc_mod, 0, 1000);
}

void update_motor_state(int motor) {
    int offset = (motor == 1) ? 3 : 0;
    uint8_t state = motor_state[motor];

    sense_coil_min[motor] = 0;
    sense_coil_max[motor] = 0;

    // reset our own bits in the active pin set/mask bits
    uint32_t motor_mask = MOTOR_I_TO_MASK[0 + offset] | MOTOR_I_TO_MASK[1 + offset] | MOTOR_I_TO_MASK[2 + offset];
    motor_pwm_bits_next &= ~motor_mask;
    motor_disable_clear_bits &= ~motor_mask;

    uint32_t enable_mask = ENABLE_I_TO_MASK[0 + offset] | ENABLE_I_TO_MASK[1 + offset] | ENABLE_I_TO_MASK[2 + offset];
    motor_enable_bits &= ~enable_mask;
    motor_disable_clear_bits &= ~enable_mask;

    // Sanity/safety check! or the motor could get stuck on 100%!
    if (target_duty_cycle[motor] == 0 || (target_current[motor] == 0 && motor_current[motor] < MIN_MOTOR_CURRENT)) {
        PORT->Group[0].OUTCLR.reg = motor_mask; // clear them completely
        return;
    }

    switch (state) {
        case 0:
        case 3:
            motor_enable_bits |= ENABLE_I_TO_MASK[0 + offset] | ENABLE_I_TO_MASK[1 + offset];
            motor_disable_clear_bits |= ENABLE_I_TO_MASK[2 + offset];
            
            //motor_enable(0 + offset);
            //motor_enable(1 + offset);
            //motor_disable(2 + offset);
            if (state == 0) {
                motor_disable_clear_bits |= MOTOR_I_TO_MASK[1 + offset];
                //motor_clear(1 + offset);
                motor_pwm_bits_next |= MOTOR_I_TO_MASK[0 + offset];
            } else {
                motor_disable_clear_bits |= MOTOR_I_TO_MASK[0 + offset];
                //motor_clear(0 + offset);
                motor_pwm_bits_next |= MOTOR_I_TO_MASK[1 + offset];
            }
            break;
        case 1:
        case 4:
            motor_enable_bits |= ENABLE_I_TO_MASK[0 + offset] | ENABLE_I_TO_MASK[2 + offset];
            motor_disable_clear_bits |= ENABLE_I_TO_MASK[1 + offset];

            //motor_enable(0 + offset);
            //motor_disable(1 + offset);
            //motor_enable(2 + offset);
            if (state == 1) {
                motor_disable_clear_bits |= MOTOR_I_TO_MASK[2 + offset];
                //motor_clear(2 + offset);
                motor_pwm_bits_next |= MOTOR_I_TO_MASK[0 + offset];
            } else {
                motor_disable_clear_bits |= MOTOR_I_TO_MASK[0 + offset];
                //motor_clear(0 + offset);
                motor_pwm_bits_next |= MOTOR_I_TO_MASK[2 + offset];
            }
            break;
        case 2:
        case 5:
            motor_enable_bits |= ENABLE_I_TO_MASK[1 + offset] | ENABLE_I_TO_MASK[2 + offset];
            motor_disable_clear_bits |= ENABLE_I_TO_MASK[0 + offset];
            //motor_disable(0 + offset);
            //motor_enable(1 + offset);
            //motor_enable(2 + offset);
            if (state == 2) {
                motor_disable_clear_bits |= MOTOR_I_TO_MASK[2 + offset];
                //motor_clear(2 + offset);
                motor_pwm_bits_next |= MOTOR_I_TO_MASK[1 + offset];
            } else {
                motor_disable_clear_bits |= MOTOR_I_TO_MASK[1 + offset];
                //motor_clear(1 + offset);
                motor_pwm_bits_next |= MOTOR_I_TO_MASK[2 + offset];
            }
            break;
    }

    if (motor == 1) {
        motor_disable(0 + offset);
        motor_disable(1 + offset);
        motor_disable(2 + offset);
        motor_pwm_bits_next &= ~MOTOR_I_TO_MASK[1 + offset];
        motor_disable_clear_bits |= MOTOR_I_TO_MASK[1 + offset];
        motor_enable_bits &= ~ENABLE_I_TO_MASK[1 + offset];
        motor_disable_clear_bits |= ENABLE_I_TO_MASK[1 + offset];
        //return;
    }
    new_motor_bits = true;
}

void update_motor(int motor) {
    int offset = (motor == 1) ? 3 : 0;

    //uint32_t combined_raw = (motor_current_raw[motor] >> 16) * (duty_cycle_measured[motor] / PWM_PERIOD);
    uint32_t combined_raw = (motor_current_raw[motor] / PWM_PERIOD) * target_duty_cycle[motor] * 15; // w/ a correction factor to make it more... accurate?
    motor_current_combined_raw[motor] = (motor_current_combined_raw[motor] * 127 + combined_raw) >> 7;
    uint32_t curr = motor_current_combined_raw[motor] >> 16;
    motor_current[motor] = curr;

    if (target_hz[motor] == 0 && target_current[motor] == 0 && (target_duty_cycle[motor] <= MIN_DUTY_CYCLE || curr < MIN_MOTOR_CURRENT)) {
        motor_disable(0 + offset);
        motor_disable(1 + offset);
        motor_disable(2 + offset);

        PORT->Group[0].OUTCLR.reg = 1 << ((motor == 1) ? MOTOR1_STATUS : MOTOR0_STATUS);

        if (target_duty_cycle[motor] != 0) {
            target_duty_cycle[motor] = 0;
            tcc_set_compare_value(&tcc_mod, motor, 0);
        }

        motor_hz[motor] = 0;
        //duty_cycle_measured[motor] = 0;
        update_motor_state(motor); // turn off pwm too

        motor_clear(0 + offset);
        motor_clear(1 + offset);
        motor_clear(2 + offset);
        return;
    }

    PORT->Group[0].OUTSET.reg = 1 << ((motor == 1) ? MOTOR1_STATUS : MOTOR0_STATUS);

    // target current is our current limit and target
    // target hz gives us a minimum speed, to help, e.g. with starting the motor up

    volatile uint32_t now_ticks = ticks_pwm_cycles;
    
    bool time_to_adjust = now_ticks - last_hz_adjust_ticks[motor] > HZ_ADJUST_TICKS;
    if (target_hz[motor] > 0 && time_to_adjust) {
        target_current[motor] = pid_next(&hz_pid[motor], motor_hz[motor]);
        pid_set_setpoint(&current_pid[motor], target_current[motor]);
        last_hz_adjust_ticks[motor] = now_ticks;
    }

    time_to_adjust = now_ticks - last_current_adjust_ticks[motor] > CURRENT_ADJUST_TICKS;
    if ((target_current[motor] > 0 || target_duty_cycle[motor] > MIN_DUTY_CYCLE) && time_to_adjust) {
        uint16_t new_target_duty = (uint16_t)pid_next(&current_pid[motor], curr);
        int16_t diff = new_target_duty - (int16_t)target_duty_cycle[motor];
        if (diff > MAX_DUTY_CHANGE) {
            diff = MAX_DUTY_CHANGE;
        }
        if (diff < -MAX_DUTY_CHANGE) {
            diff = -MAX_DUTY_CHANGE;
        }
        target_duty_cycle[motor] += diff;

        //target_duty_cycle[motor] = target_current[motor];
        if (!sync_done[motor] && target_duty_cycle[motor] > MIN_DUTY_CYCLE) {
            target_duty_cycle[motor] = MIN_DUTY_CYCLE;
        }
        tcc_set_compare_value(&tcc_mod, motor, target_duty_cycle[motor]);
        last_current_adjust_ticks[motor] = now_ticks;

        //// we need to time measuring from the pwm to get the floating pin during the pwm off cycle
        //// and then to immediately get the current pin after the start of the next on cycle
        uint32_t after_pwm_off = target_duty_cycle[motor] + PWM_REST / 2;
        uint32_t before_pwm_on = PWM_PERIOD - 200;
        uint32_t measure_time = after_pwm_off > before_pwm_on ? after_pwm_off : before_pwm_on;
        adc_measurement_time[motor] = measure_time;
    }

    bool should_advance_state = false;

    average_coil[motor] = (average_coil[motor] * 255 + (motor_coil[motor] << 16) + 128) >> 8;
    volatile uint16_t coil_threshold = average_coil[motor] >> 16;
    volatile uint16_t min_thresh = (motor == 0) ? 50 : 50;
    if (coil_threshold < min_thresh) {
        coil_threshold = min_thresh;
    }
    if (coil_high[motor] && motor_coil[motor] <= coil_threshold) {
        coil_high[motor] = false;
        change_after_delay[motor] = true;
        int32_t new_interval = (now_ticks - last_zero_ticks[motor]) << 16;
        state_change_interval[motor] = (state_change_interval[motor] * 255 + new_interval + 128) >> 8;
        last_zero_ticks[motor] = now_ticks;
    } else if (!coil_high[motor] && motor_coil[motor] > coil_threshold) {
        coil_high[motor] = true;
        change_after_delay[motor] = true;
        int32_t new_interval = (now_ticks - last_zero_ticks[motor]) << 16;
        state_change_interval[motor] = (state_change_interval[motor] * 255 + new_interval + 128) >> 8;
        last_zero_ticks[motor] = now_ticks;
    }

    if (target_duty_cycle[motor] > 0 && motor_hz[motor] > 0 && sync_done[motor]) {
        if (change_after_delay[motor]) {// && (now_ticks - last_zero_ticks[motor]) > state_change_interval[motor] >> 17) {
            should_advance_state = true;
            change_after_delay[motor] = false;
        }
    } else {
        // ramp up one hz at a time from sync start to end hz
        if (motor_hz[motor] == 0) {
            sync_start_ticks[motor] = now_ticks;
            sync_done[motor] = false;
        }
        if (now_ticks - sync_start_ticks[motor] > SYNCHRONOUS_STEP_TICKS) {
            sync_done[motor] = true;
            // prime the pid to avoid jolt in derivative term
            pid_next(&current_pid[motor], curr);
        }
        motor_min_hz[motor] = SYNCHRONOUS_HZ;
    }

    uint16_t ticks_per_transition = PWM_FREQ / motor_min_hz[motor] / 42;
    if ((now_ticks - last_transition_ticks[motor]) >= ticks_per_transition) {
        should_advance_state = true;
    }

    if (should_advance_state) {
        motor_state[motor] = (motor_state[motor] + 1) % 6;
        update_motor_state(motor);

        last_transition_ticks[motor] = now_ticks;
        motor_count[motor]++;
    }

    if ((now_ticks - last_hz_ticks[motor]) >= PWM_FREQ / HZ_MEASUREMENT_HZ) {
        uint32_t new_hz = (motor_count[motor] * HZ_MEASUREMENT_HZ + 21) / 42;
        //if (motor_hz[motor] > 85 && new_hz < 80) {
            //volatile int c = new_hz + 1;
        //}
        motor_hz[motor] = new_hz;
        motor_count[motor] = 0;
        last_hz_ticks[motor] = now_ticks;
        if (sync_done[motor]) {
            //motor_min_hz[motor] = (motor_hz[motor] * 15) >> 4;
            if (motor_min_hz[motor] < SYNCHRONOUS_HZ) {
                motor_min_hz[motor] = SYNCHRONOUS_HZ;
            }
        }
    }
}

void configure_pid() {
    // will output current in adc lsb units (that is, bits as read from the adc)
    pid_init(&hz_pid[0], 1.0, 1.0, 0, 0, HZ_ADJUST_HZ);
    pid_init(&hz_pid[1], 1.0, 1.0, 0, 0, HZ_ADJUST_HZ);
    pid_set_output_limits(&hz_pid[0], 0.0, 20.0);
    pid_set_output_limits(&hz_pid[1], 0.0, 20.0);
    // will directly output duty cycle
    pid_init(&current_pid[0], 0.003*PWM_PERIOD, 0.0, 0.01*PWM_PERIOD, 0, CURRENT_ADJUST_HZ);
    pid_init(&current_pid[1], 0.003*PWM_PERIOD, 0.0000*PWM_PERIOD, 0.00*PWM_PERIOD, 0.0000*PWM_PERIOD, CURRENT_ADJUST_HZ);
    pid_set_output_limits(&current_pid[0], MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
    pid_set_output_limits(&current_pid[1], MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
    pid_set_integral_limits(&current_pid[0], -MAX_DUTY_CYCLE/4, MAX_DUTY_CYCLE/4);
    pid_set_integral_limits(&current_pid[1], -MAX_DUTY_CYCLE/4, MAX_DUTY_CYCLE/4);
    //pid_set_output_filter(&current_pid[0], 20);
    //pid_set_output_filter(&current_pid[1], 20);
}

int main() {
    system_init();

    configure_adc();
    configure_i2c();
    configure_pwm();
    configure_pid();

    volatile uint32_t update_count = 0;
    while (1) {
        volatile uint16_t update_rate = (uint64_t)update_count * PWM_FREQ / ticks_pwm_cycles;

        update_motor(0);
        update_motor(1);

        adc_motor0_float_coil = MOTOR0_STATE_TO_FLOATING_ADC[motor_state[0]];
        adc_motor1_float_coil = MOTOR1_STATE_TO_FLOATING_ADC[motor_state[1]];
        update_count++;
    }
    return 0;
}
