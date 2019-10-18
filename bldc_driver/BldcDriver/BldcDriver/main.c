/*
 * BldcDriver.c
 *
 * Created: 5/1/2018 10:46:23 PM
 * Author : Acshi
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#ifndef _BV
#define _BV(bit) (1 << bit)
#endif

#define CLOCK_FREQ 20000000
#define PWM_FREQ 20000 // 16000
#define PWM_PERIOD (CLOCK_FREQ/PWM_FREQ)
#define PWM_REST 80 // min cycles for the high side mosfet-driver charge pumps to recover
#define COMP_DELAY 20 // cycles of delay after start of rest for analog comparators to start running
#define MAX_DUTY_CYCLE (PWM_PERIOD - PWM_REST)
#define MIN_DUTY_CYCLE (PWM_PERIOD / 20) // (PWM_PERIOD / 16)
// index-0 will be for 1hz
//#define SYNC_DUTY_N 7
//#define SYNC_DUTY_BY_3EHZ ((uint16_t[]){ PWM_PERIOD / 16, PWM_PERIOD / 15, PWM_PERIOD / 14, PWM_PERIOD / 13, PWM_PERIOD / 12, PWM_PERIOD / 11, PWM_PERIOD / 10})
//#define SYNC_DUTY_BY_EHZ ((uint16_t[]){ PWM_PERIOD / 12})
//#define SYNC_DUTY_BY_EHZ ((uint16_t[]){ PWM_PERIOD / 12})
//#define SYNC_DUTY_N (sizeof(SYNC_DUTY_BY_EHZ) / sizeof(SYNC_DUTY_BY_EHZ[0]))
#define MAX_DUTY_INC_ABS 20
#define MAX_DUTY_INC_PERCENT (256 / 16) // (256 / 32) // "percentage" of 256
#define MAX_DUTY_DEC_PERCENT (256 / 4) // "percentage" of 256
#define OVER_CURRENT_CHANGE 5

// synchronous speed for start up
#define SYNCHRONOUS_EHZ_INC 1
#define SYNCHRONOUS_EHZ_START (10*6)
#define SYNCHRONOUS_EHZ_END (64*6) //(48*6) // 24*6
#define SYNCHRONOUS_STEP_SEC 0.002 // 0.002 0.003 0.004
#define SYNCHRONOUS_STEP_TICKS (int16_t)(PWM_FREQ * SYNCHRONOUS_STEP_SEC)

#define MOTOR_RUNNING_CURRENT_THRESH 20

// Alignment of motor phases for start up
#define ALIGN_STEP_SEC 0.0 // seconds to assert an alignment step
#define ALIGN_STEP_TICKS (int16_t)(PWM_FREQ * ALIGN_STEP_SEC)

// rate at which speed measurement is updated
#define HZ_MEASUREMENT_HZ 100

#define DUTY_ADJUST_HZ 100
#define DUTY_ADJUST_TICKS (PWM_FREQ / DUTY_ADJUST_HZ)
#define MIN_MOTOR_CURRENT 10

#define STATUS_OFF(); PORTD &= ~_BV(PORTD5);
#define STATUS_ON(); PORTD |= _BV(PORTD5);

#define MOTOR_ENABLE_OFF(); PORTD &= ~_BV(PORTD3);
#define MOTOR_ENABLE_ON(); PORTD |= _BV(PORTD3);

#define TRIGGER_OFF(); PORTB &= ~_BV(PORTB0);
#define TRIGGER_ON(); PORTB |= _BV(PORTB0);

#define TRIGGER2_OFF(); PORTE &= ~_BV(PORTE3);
#define TRIGGER2_ON(); PORTE |= _BV(PORTE3);

#define READ_FAULT() (PIND & _BV(PORTD4))

#define MOT_HA_COUNT OCR1B
#define MOT_HB_COUNT OCR1A
#define MOT_HC_COUNT OCR3A
#define COMP_CHECK_COUNT OCR3B

// this version is actually synced to pwm output
uint16_t set_duty_cycle = 0;

#define MOT_LA_OFF(); PORTB &= ~_BV(PORTB4);
#define MOT_LB_OFF(); PORTB &= ~_BV(PORTB3);
#define MOT_LC_OFF(); PORTD &= ~_BV(PORTD1);
#define MOT_LA_ON(); PORTB |= _BV(PORTB4);
#define MOT_LB_ON(); PORTB |= _BV(PORTB3);
#define MOT_LC_ON(); PORTD |= _BV(PORTD1);

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

#define CURRENT_ADC 3
#define MOTOR_V_ADC 4
// MOTOR_A is on AIN1, and not used through an ADC mux
#define MOTOR_B_ADC 1
#define MOTOR_C_ADC 2
uint8_t adc_aux_i = 0;

// used for inductive position sensing
// and has a different filter alpha than motor_current_raw
volatile uint16_t startup_current_raw = 0; // raw adc value

volatile uint32_t motor_current_raw = 0; // 16-bit fixed point
volatile uint32_t motor_voltage_raw = 0; // 16-bit fixed point
volatile uint16_t temp_raw = 0;
volatile uint32_t motor_ehz_raw = 0; // 16-bit fixed point, by transition (divide by 6 for true electrical hz)
volatile uint32_t motor_ehz = 0; // by transition (divide by 6 for true electrical hz)
volatile uint32_t motor_count = 0; // state change count
volatile uint32_t last_motor_ehz_raw; // from last measurement period

volatile bool has_scheduled_tick = false; // enables the below scheduled tick
volatile uint32_t scheduled_transition_ticks = 0; // when ticks_pwm_cycles reaches this value, we increment the motor state
volatile bool has_reported_tick = false; // enables reporting scheduled ticks when they are not used/executed

#define MIN_TICKS_PER_PHASE (PWM_FREQ / SYNCHRONOUS_EHZ_START)
volatile uint32_t ticks_per_phase_raw; // 16-bit fixed point
volatile uint32_t ticks_per_phase;
volatile uint32_t last_zero_ticks;
volatile uint32_t last_zero_subticks;

const uint8_t notes_to_freqs_offset = 43;
const uint16_t notes_to_freqs[42] = {196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093};

uint8_t music_track = 1;
uint16_t note_on = 0;
bool note_delay_done = false;
uint32_t note_event_ticks = 0;

// for a last_ticks filter
#define LTICKS_FILTER_MIN_EHZ (5*6*7)
#define LTICKS_FILTER_MAX_EHZ (100*6*7)
#define LTICKS_FILTER_MIN_N 2
#define LTICKS_FILTER_MAX_N 32
uint32_t last_subticks_buffer[LTICKS_FILTER_MAX_N] = { 0 };
//uint16_t last_ticks_sorted[LTICKS_FILTER_N] = { 0 };

#define PREEMPT_MAX_CHANGE (((uint32_t)1 << 16) * 3 / HZ_MEASUREMENT_HZ)
#define PREEMPT_MIN_CHANGE (((uint32_t)1 << 16) / 20 / HZ_MEASUREMENT_HZ)
#define PREEMPT_MAX_PREEMPT 28 // out of 128 for a full commutation
#define PREEMPT_MAX_PREEMPT_NEG -8 // out of 128 for a full commutation
int16_t preemption_factor = 0;

// current state of each motor, in terms of which coil is power, ground, and floating
uint8_t motor_state = 0;

volatile uint16_t current_limit = 2048;

int16_t user_duty_cycle = 0; // this is the one directly set by commands
uint16_t target_duty_cycle = 0;
uint16_t start_duty_cycle = 120;

uint16_t motor_min_ehz = 0; // minimum hz for transitions in case we miss commutation, also for synchronous start up
uint32_t last_transition_ticks = 0; // for timing transitions
uint32_t last_duty_adjust_ticks = 0; // for current pid
uint32_t last_hz_ticks = 0; // for hz measurement
uint32_t sync_start_ticks = 0; // synchronous starting
bool sync_done = false;
bool align_done = false;

// incremented at whatever our pwm frequency
volatile uint32_t ticks_pwm_cycles = 0;
volatile uint16_t update_rate;

uint8_t i2c_read_buf[16];
uint8_t i2c_read_buf_i = 0;
uint8_t i2c_out_buf[16];
uint8_t i2c_out_buf_i = 0;
uint8_t i2c_out_bytes = 0;

static void configure_directions(void) {
    // bits set for output directions
    DDRB = _BV(1) | _BV(2) | _BV(3) | _BV(4); // HB, HA, LB, LA
    DDRC = 0;
    DDRD = _BV(0) | _BV(1) | _BV(5); // HC, LC, STATUS
    DDRE = 0;
}

static void start_aux_adc_read(void) {
    // first make sure we aren't already doing a conversion
    if ((ADCSRA & _BV(ADSC)) == 0) {
        adc_aux_i = (adc_aux_i + 1) % 2;
        if (adc_aux_i == 0) {
            ADMUX = _BV(REFS0) | MOTOR_V_ADC; // w/ AVCC reference
        } else {
            ADMUX = _BV(REFS0) | CURRENT_ADC; // w/ AVCC reference
        }
        ADCSRA |= _BV(ADEN); // adc enable
        ADCSRA |= _BV(ADSC); // start conversion
    }
}

ISR(ADC_vect) {
    uint32_t adc_val = ADC;
    if (adc_aux_i == 0) {
        // on the order of 10-20Hz low pass
        motor_voltage_raw = ((uint64_t)motor_voltage_raw * 511 + (adc_val << 16) + 256) >> 9;
    } else {
        motor_current_raw = ((uint64_t)motor_current_raw * 511 + (adc_val << 16) + 256) >> 9;
        // no filtering here
        startup_current_raw = adc_val;
    }
}

static void configure_adc(void) {
    PRR0 &= ~_BV(PRADC); // enable in power reduction register
    ADCSRA = 0;
    ADCSRA |= _BV(ADIE); // interrupt enable
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // 1/64 clock prescaler, 20mhz to ~300khz

    ADMUX = _BV(REFS0) | MOTOR_V_ADC; // AVCC reference

    DIDR0 = 0xff; // disable all digital input buffers; use PORTC only for analog! (and not using PORTE2,3)
}

static void update_motor_state(void);
static void delay_increment_motor_state(void);
static void increment_motor_state(void);

ISR(ANALOG_COMP_vect) {
    delay_increment_motor_state();

    //static void comparator_coil_a_callback(struct ac_module *const module_inst) {
    //// coil a is floating in states 2 (falling below zero), 5 (rising back)
    //uint8_t status = AC1->STATUSA.bit.STATE0;
    //if ((motor_state == 2 && !status) || (motor_state == 5 && status)) {
    //delay_increment_motor_state();
    //}
    //}
    //
    //static void comparator_coil_b_callback(struct ac_module *const module_inst) {
    
    //uint8_t status = AC->STATUSA.bit.STATE0;
    //if ((motor_state == 1 && !status) || (motor_state == 4 && status)) {
    //delay_increment_motor_state();
    //}
    //}
    //
    //static void comparator_coil_c_callback(struct ac_module *const module_inst) {

    //uint8_t status = AC->STATUSA.bit.STATE1;
    //if ((motor_state == 0 && !status) || (motor_state == 3 && status)) {
    //delay_increment_motor_state();
    //}
    //}
}

static void configure_comparators(void) {
    // disabled (but still on!) to begin with
    // we want to carefully control when we are looking for which kind of edge
    ACSR = 0;

    ADCSRB |= ACME; // enable ADC multiplexed input
    // ADMUX = _BV(REFS0) | MOTOR_B_ADC; // leaving the AVCC reference, MOTOR_B to the negative input

    DIDR1 = _BV(AIN0D) | _BV(AIN1D); // disable digital input buffer, since we only use this for analog
}

static void set_send_reply(uint8_t *msg, uint16_t val) {
    msg[0] = MSG_MARK_BYTE;
    msg[1] = (val >> 8) & 0xff;
    msg[2] = val & 0xff;
}

// finished reading the write request, prepare the write here
static void i2c_process_read(void) {
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
            set_send_reply(i2c_out_buf, preemption_factor);
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

ISR(TWI1_vect) {
    switch (TWSR1) {
    case 0x60: // controller is going to write to us
        i2c_read_buf_i = 0;
        break;
    case 0x80: // byte written by controller
        i2c_read_buf[i2c_read_buf_i] = TWDR1;
        i2c_read_buf_i++;
        i2c_process_read();
        break;
    case 0xA8: // initial byte requested by controller
        i2c_out_buf_i = 0;
        // fall through
    case 0xB8: // following byte requested by controller
        if (i2c_out_bytes > i2c_out_buf_i) {
            TWDR1 = i2c_out_buf[i2c_out_buf_i];
            i2c_out_buf_i++;
        } else {
            TWDR1 = 0;
        }
        break;
    }
}

static void configure_i2c(void) {
    TWBR1 = 0; // unused bitrate divider
    TWSR1 = 0; // unused prescaler to zero
    TWAR1 = I2C_ADDR << 1;
    TWCR1 = 0;
    TWCR1 |= _BV(TWEA); // acknowledge enable
    TWCR1 |= _BV(TWEN); // enable
    TWCR1 |= _BV(TWIE); // interrupt enable
}

ISR(TIMER1_OVF_vect) {
    // motor outputs about to go high! don't want to pay attention to the analog comparator
    // during this noisy time!
    ACSR = 0;

    ticks_pwm_cycles++;
    if (has_scheduled_tick && ticks_pwm_cycles >= scheduled_transition_ticks) {
        if (!has_reported_tick) {
            has_reported_tick = true;
            // toggle a debug output
        }
        if (sync_done) {
            increment_motor_state();
        }
    }
}

ISR(TIMER3_COMPB_vect) {
    // time to check for zero crossing
    // we set which rising edge and enabled the interrupt

    // coil a is floating in states 2 (falling below zero), 5 (rising back
    // coil b is floating in states 1 (rising), 4 (falling)
    // coil c is floating in states 0 (falling), 3 (rising)

    switch (motor_state % 3) {
    case 0: // coil c
        ADCSRA &= ~_BV(ADEN); // disable ADC
        ADMUX = _BV(REFS0) | MOTOR_C_ADC; // w/ AVCC reference
        break;
    case 1: // coil b
        ADCSRA &= ~_BV(ADEN); // disable ADC
        ADMUX = _BV(REFS0) | MOTOR_B_ADC; // w/ AVCC reference
        break;
    case 2: // coil a
        // coil a on the default comparator negative input!
        // we can use the adc now!
        start_aux_adc_read();
        break;
    }

    if (motor_state % 2 == 0) {
        ACSR = _BV(ACIS1) | _BV(ACIE); // interrupt on falling edge
    } else {
        ACSR = _BV(ACIS0) | _BV(ACIE); // interrupt on rising edge
    }
}

static void configure_pwm(void) {
    TCCR1A = _BV(COM1A1) | _BV(COM1B1); // clear on compare match, set on bottom
    TCCR1A |= _BV(WGM11); // mode 14, fast pwm, with the next line
    TCCR1B = _BV(WGM12) | _BV(WGM13);
    TCCR1B |= _BV(CS10); // clock / 1 prescaler
    ICR1 = PWM_PERIOD; // in this mode, this it TOP, setting the overall frequency
    OCR1A = 0; // control duty cycle
    OCR1B = 0; // control duty cycle
    TIMSK1 = _BV(TOIE1); // overflow interrupt enable

    // same settings for timer 3A for the third coil
    TCCR3A = _BV(COM3A1); // clear on compare match, set on bottom
    TCCR3A |= _BV(WGM31); // mode 14, fast pwm, with the next line
    TCCR3B = _BV(WGM32) | _BV(WGM33);
    TCCR3B |= _BV(CS30); // clock / 1 prescaler
    ICR3 = PWM_PERIOD; // in this mode, this it TOP, setting the overall frequency
    OCR3A = 0; // control duty cycle
    TIMSK3 = _BV(OCF3B); // counter 3 interrupt enable

    // time the check on the appropriate comparator during the off part of the period
    OCR3B = MAX_DUTY_CYCLE / 2 + COMP_DELAY;
}

static void update_motor_state(void) {
    if (target_duty_cycle == 0) {
        // turn everything off
        MOT_HA_COUNT = 0;
        MOT_HB_COUNT = 0;
        MOT_HC_COUNT = 0;
        MOT_LA_OFF();
        MOT_LB_OFF();
        MOT_LC_OFF();
        return;
    }

    // high goes by A, A, B, B, C, C
    // low goes by C, B, B, A, A, C
    switch (motor_state) {
    case 0:
        MOT_HA_COUNT = set_duty_cycle;
        MOT_LC_ON();
        break;
    case 1:
        MOT_HA_COUNT = set_duty_cycle;
        MOT_LB_ON();
        break;
    case 2:
        MOT_HB_COUNT = set_duty_cycle;
        MOT_LB_ON();
        break;
    case 3:
        MOT_HB_COUNT = set_duty_cycle;
        MOT_LA_ON();
        break;
    case 4:
        MOT_HC_COUNT = set_duty_cycle;
        MOT_LA_ON();
        break;
    case 5:
        MOT_HC_COUNT = set_duty_cycle;
        MOT_LC_ON();
        break;
    }
}

// how big of a filter should we use for our current speed?
// linear interpolation between min and max
static uint8_t lticks_filter_size(void) {
    if (motor_ehz < LTICKS_FILTER_MIN_EHZ) {
        return LTICKS_FILTER_MIN_N;
    } else if (motor_ehz > LTICKS_FILTER_MAX_EHZ) {
        return LTICKS_FILTER_MAX_N;
    }
    uint16_t rounding = (LTICKS_FILTER_MAX_N - LTICKS_FILTER_MIN_N) / (LTICKS_FILTER_MAX_EHZ - LTICKS_FILTER_MIN_EHZ) / 2;
    return LTICKS_FILTER_MIN_N + (motor_ehz - LTICKS_FILTER_MIN_EHZ + rounding)
                               * (LTICKS_FILTER_MAX_N - LTICKS_FILTER_MIN_N)
                               / (LTICKS_FILTER_MAX_EHZ - LTICKS_FILTER_MIN_EHZ);
}

// how many degrees ahead of time to schedule the tick
// the units are 1/128 of a commutation, or about 2.8 degrees
// can be both positive or negative (for deceleration)
// linear interpolation between values
static int16_t degree_preemption_factor(void) {
    // we use the full raw values to see small changes
    // and we are looking at percentage change here
    int32_t ehz_change = ((int32_t)motor_ehz_raw - (int32_t)last_motor_ehz_raw) / (int32_t)motor_ehz;
    uint32_t abs_change = (ehz_change > 0) ? ehz_change : -ehz_change;

    if (abs_change > PREEMPT_MAX_CHANGE) {
        if (ehz_change > 0) {
            return PREEMPT_MAX_PREEMPT;
        }
        return PREEMPT_MAX_PREEMPT_NEG;
    }
    if (abs_change < PREEMPT_MIN_CHANGE) {
        return 0;
    }
    //int16_t rounding = PREEMPT_MAX_PREEMPT / (PREEMPT_MAX_CHANGE - PREEMPT_MIN_CHANGE) / 2;
    int16_t factor = (abs_change - PREEMPT_MIN_CHANGE)
                     * PREEMPT_MAX_PREEMPT
                     / (PREEMPT_MAX_CHANGE - PREEMPT_MIN_CHANGE);
    if (factor < PREEMPT_MAX_PREEMPT_NEG) {
        return PREEMPT_MAX_PREEMPT_NEG;
    }
    return factor;
}

static void delay_increment_motor_state(void) {
    if (!has_scheduled_tick) {
        // read our sub-tick position
        uint32_t now_subticks = TCNT1;

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
        // advance the time interval by some number of degrees
        // this is necessary for fast acceleration/deceleration
        preemption_factor = degree_preemption_factor();
        subticks_per_phase = ((subticks_per_phase * (128 - preemption_factor)) >> 7);

        ticks_per_phase = subticks_per_phase / PWM_PERIOD;
        scheduled_transition_ticks = last_transition_ticks + ticks_per_phase;
        has_scheduled_tick = true;
        has_reported_tick = false;
        last_zero_ticks = now_ticks;
        last_zero_subticks = now_subticks;
        //PORT->Group[0].OUTTGL.reg = 1 << TRIGGER_OUT;
    }
}

static void increment_motor_state(void) {
    // toggle debug2 pin
    motor_state = (motor_state + 1) % 6;
    update_motor_state();

    motor_count++;
    last_transition_ticks = ticks_pwm_cycles;
    has_scheduled_tick = false;
}

static void update_motor(void) {
    uint16_t motor_current = motor_current_raw >> 16;
    bool restart_sync = sync_done && (motor_current < MIN_MOTOR_CURRENT || motor_ehz < SYNCHRONOUS_EHZ_START);
    if (((user_duty_cycle == 0 || current_limit == 0) && (!sync_done || target_duty_cycle <= MIN_DUTY_CYCLE)) || restart_sync) {
        // turn off all the outputs
        target_duty_cycle = 0;
        set_duty_cycle = 0;
        update_motor_state();
        STATUS_OFF();

        // reset state to not-moving
        motor_ehz_raw = 0;
        motor_ehz = 0;
        align_done = false;
        sync_done = false;
        motor_min_ehz = 0;
        last_duty_adjust_ticks = 0;
        note_on = 0;
        return;
    }

    // turn status light on
    STATUS_ON();

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
            int16_t diff_percent = (diff << 8) / (int16_t)target_duty_cycle;
            if (diff_percent > MAX_DUTY_INC_PERCENT) {
                diff_percent = MAX_DUTY_INC_PERCENT;
            }
            if (diff_percent < -MAX_DUTY_DEC_PERCENT) {
                diff_percent = -MAX_DUTY_DEC_PERCENT;
            }
            diff = (diff_percent * (int16_t)target_duty_cycle + 127) >> 8;
            if (diff > MAX_DUTY_INC_ABS) {
                diff = MAX_DUTY_INC_ABS;
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
        set_duty_cycle = target_duty_cycle;
        last_duty_adjust_ticks = now_ticks;
    }

    // asynchronous commutation managed by the comparator callbacks
    if (sync_done) {
        return;
    }

    // Music modes!
    if (user_duty_cycle > 0 && user_duty_cycle <= 4) {
        music_track = user_duty_cycle - 1;
        if (note_delay_done) {
            //if (now_ticks - note_event_ticks > note_times[music_track][note_on] * PWM_FREQ / 50 / 8) {
                //note_on++;
                //if (note_on > notes_n[music_track]) {
                    //note_on = 0;
                    //user_duty_cycle = 0;
                //}
//
                //note_delay_done = false;
                //note_event_ticks = now_ticks;
//
                //motor_min_ehz = 0;
                //target_duty_cycle = 0;
                //tcc_set_compare_value(&tcc_mod, 0, target_duty_cycle);
            //}
        } else {
            //if (now_ticks - note_event_ticks > note_delays[music_track][note_on] * PWM_FREQ / 50 / 8) {
                //note_delay_done = true;
                //note_event_ticks = now_ticks;
//
                //motor_min_ehz = notes_to_freqs[notes[music_track][note_on] - notes_to_freqs_offset];
//
                //target_duty_cycle = MIN_DUTY_CYCLE;
                //tcc_set_compare_value(&tcc_mod, 0, target_duty_cycle);
            //} else {
                //target_duty_cycle = 0;
                //tcc_set_compare_value(&tcc_mod, 0, target_duty_cycle);
            //}
        }
    } else if (now_ticks - sync_start_ticks > SYNCHRONOUS_STEP_TICKS) {
        if ((motor_current_raw >> 16) > MOTOR_RUNNING_CURRENT_THRESH) {
            // condition above to make sure the motor has started turning
            // otherwise we stay in slow synchronous state for debugging
            motor_min_ehz = motor_min_ehz + SYNCHRONOUS_EHZ_INC;
            if (motor_min_ehz >= SYNCHRONOUS_EHZ_END) {
                motor_min_ehz = SYNCHRONOUS_EHZ_END;
                sync_done = true;
                motor_min_ehz = SYNCHRONOUS_EHZ_START;

                target_duty_cycle = MIN_DUTY_CYCLE;
                //tcc_set_compare_value(&tcc_mod, 0, target_duty_cycle);
                last_duty_adjust_ticks = now_ticks;
            }
        }
        sync_start_ticks = now_ticks;
    }

    uint16_t ticks_per_transition = (PWM_FREQ + motor_min_ehz / 2) / motor_min_ehz;
    if ((now_ticks - last_transition_ticks) >= ticks_per_transition) {
        increment_motor_state();
    }
}

static void update_speed_estimate(void) {
    uint32_t now_ticks = ticks_pwm_cycles;
    if ((now_ticks - last_hz_ticks) >= PWM_FREQ / HZ_MEASUREMENT_HZ) {
        last_motor_ehz_raw = motor_ehz_raw;

        uint32_t new_ehz = (motor_count * HZ_MEASUREMENT_HZ + 3);
        motor_ehz_raw = ((uint64_t)motor_ehz_raw * 15 + (new_ehz << 16) + 7) >> 4;
        motor_ehz = motor_ehz_raw >> 16;
        motor_count = 0;
        last_hz_ticks = now_ticks;
    }
}

int main(void) {
    configure_directions();
    configure_adc();
    configure_comparators();

    configure_i2c();
    configure_pwm();

    sei(); // global interrupt enable

    user_duty_cycle = 100; //TODO REMOVE WHEN DONE DEBUGGING THIS!

    volatile uint32_t update_count = 0;
    while (1) {
        update_rate = (uint64_t)update_count * PWM_FREQ / ticks_pwm_cycles;
        update_motor();
        update_speed_estimate();
        update_count++;
    }
    return 0;
}

