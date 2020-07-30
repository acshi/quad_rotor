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

#define CLOCK_FREQ 8000000
#define PWM_FREQ 25000 // 16000
#define PWM_PERIOD (CLOCK_FREQ/PWM_FREQ) // 500 for 16000, 400 for 20000
#define PWM_REST 40 // min cycles for the high side mosfet-driver charge pumps to recover
#define COMP_DELAY 20 // cycles of delay after start of rest for analog comparators to start running
#define MAX_DUTY_CYCLE (PWM_PERIOD - PWM_REST)
#define MIN_DUTY_CYCLE (PWM_PERIOD / 60) // (PWM_PERIOD / 16)
// index-0 will be for 1hz
//#define SYNC_DUTY_N 7
//#define SYNC_DUTY_BY_3EHZ ((uint16_t[]){ PWM_PERIOD / 16, PWM_PERIOD / 15, PWM_PERIOD / 14, PWM_PERIOD / 13, PWM_PERIOD / 12, PWM_PERIOD / 11, PWM_PERIOD / 10})
//#define SYNC_DUTY_BY_EHZ ((uint16_t[]){ PWM_PERIOD / 12})
//#define SYNC_DUTY_BY_EHZ ((uint16_t[]){ PWM_PERIOD / 12})
//#define SYNC_DUTY_N (sizeof(SYNC_DUTY_BY_EHZ) / sizeof(SYNC_DUTY_BY_EHZ[0]))
#define MAX_DUTY_INC_ABS 20
#define MAX_DUTY_INC_PERCENT (64 / 32) // "percentage" of 64
#define MAX_DUTY_DEC_PERCENT (64 / 4) // "percentage" of 64
#define OVER_CURRENT_CHANGE 5

// synchronous speed for start up
#define SYNCHRONOUS_EHZ_INC 1
#define SYNCHRONOUS_EHZ_START (4*6) // 10*6
#define SYNCHRONOUS_EHZ_END (16*6) //(48*6) // 24*6
#define SYNCHRONOUS_STEP_SEC 0.004 // 0.002 0.003 0.004
#define SYNCHRONOUS_STEP_TICKS (int16_t)(PWM_FREQ * SYNCHRONOUS_STEP_SEC)
#define RETRY_MIN_EHZ (1*6)

#define MOTOR_RUNNING_CURRENT_THRESH 0 // 5 // 20

// Alignment of motor phases for start up
#define ALIGN_STEP_SEC 0.3 // seconds to assert an alignment step
#define ALIGN_STEP_TICKS (int16_t)(PWM_FREQ * ALIGN_STEP_SEC)

// rate at which speed measurement is updated
#define HZ_MEASUREMENT_HZ 64

#define DUTY_ADJUST_WAIT_SEC 0.6
#define DUTY_ADJUST_WAIT_TICKS (int16_t)(PWM_FREQ * DUTY_ADJUST_WAIT_SEC)

#define DUTY_ADJUST_HZ 64
#define DUTY_ADJUST_TICKS (PWM_FREQ / DUTY_ADJUST_HZ)
#define MIN_MOTOR_CURRENT 0

#define STATUS_OFF(); PORTD &= ~_BV(PORTD5);
#define STATUS_ON(); PORTD |= _BV(PORTD5);

#define MOTOR_ENABLE_OFF(); PORTD &= ~_BV(PORTD3);
#define MOTOR_ENABLE_ON(); PORTD |= _BV(PORTD3);

#define TRIGGER_OFF(); PORTB &= ~_BV(PORTB0);
#define TRIGGER_ON(); PORTB |= _BV(PORTB0);
#define TRIGGER_TOGGLE(); PINB |= _BV(PORTB0);

#define TRIGGER2_OFF(); PORTE &= ~_BV(PORTE3);
#define TRIGGER2_ON(); PORTE |= _BV(PORTE3);
#define TRIGGER2_TOGGLE(); PINE |= _BV(PORTE3);

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

volatile uint16_t motor_current = 0; // 11-bit . 5-bit fixed point
volatile uint16_t motor_voltage = 0; // 11-bit . 5-bit fixed point
volatile uint16_t temp_raw = 0;
uint16_t motor_ehz = 0; // by transitions, first times 2, then divide by 6 for true electrical hz, and then by the number of motor poles for physical hz
int16_t motor_delta_ehz = 0; // for preemption calculation
volatile uint16_t motor_count = 0; // state change count
uint16_t last_motor_count = 0;

volatile bool has_scheduled_tick = false; // enables the below scheduled tick
volatile uint16_t scheduled_transition_ticks = 0; // when ticks_pwm_cycles reaches this value, we increment the motor state
volatile bool has_reported_tick = false; // enables reporting scheduled ticks when they are not used/executed

#define MIN_TICKS_PER_PHASE (PWM_FREQ / SYNCHRONOUS_EHZ_START)
//volatile uint16_t ticks_per_phase_raw; // 11-bit . 5-bit fixed point
uint16_t ticks_per_phase;
uint16_t last_zero_ticks;
//volatile uint16_t last_zero_subticks;

const uint8_t notes[255] = {76, 76, 76, 72, 76, 79, 67, 72, 67, 64, 69, 71, 70, 69, 67, 76, 79, 81, 77, 79, 76, 72, 74, 71, 72, 67, 64, 69, 71, 70, 69, 67, 76, 79, 81, 77, 79, 76, 72, 74, 71, 48, 79, 78, 77, 75, 60, 76, 53, 68, 69, 72, 60, 69, 72, 74, 48, 79, 78, 77, 75, 55, 76, 84, 84, 84, 55, 48, 79, 78, 77, 75, 60, 76, 53, 68, 69, 72, 60, 69, 72, 74, 48, 75, 74, 72, 55, 55, 48, 48, 79, 78, 77, 75, 60, 76, 53, 68, 69, 72, 60, 69, 72, 74, 48, 79, 78, 77, 75, 55, 76, 84, 84, 84, 55, 48, 79, 78, 77, 75, 60, 76, 53, 68, 69, 72, 60, 69, 72, 74, 48, 75, 74, 72, 55, 55, 48, 72, 72, 72, 72, 74, 76, 72, 69, 67, 43, 72, 72, 72, 72, 74, 76, 55, 48, 43, 72, 72, 72, 72, 74, 76, 72, 69, 67, 43, 76, 76, 76, 72, 76, 79, 67, 72, 67, 64, 69, 71, 70, 69, 67, 76, 79, 81, 77, 79, 76, 72, 74, 71, 72, 67, 64, 69, 71, 70, 69, 67, 76, 79, 81, 77, 79, 76, 72, 74, 71, 76, 72, 67, 55, 68, 69, 77, 53, 77, 69, 60, 53, 71, 81, 81, 81, 79, 77, 76, 72, 55, 69, 67, 60, 55, 76, 72, 67, 55, 68, 69, 77, 53, 77, 69, 60, 53, 71, 77, 77, 77, 76, 74, 72, 64, 55, 64, 60};
const uint8_t delays[255] = {0, 0, 50, 52, 0, 50, 152, 152, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 0, 0, 0, 0, 0, 52, 50, 0, 51, 50, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 103, 101, 101, 0, 51, 50, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 0, 0, 0, 0, 0, 52, 50, 0, 51, 50, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 103, 101, 101, 0, 51, 50, 0, 50, 52, 0, 50, 0, 50, 0, 51, 50, 0, 50, 52, 0, 0, 0, 101, 103, 50, 0, 50, 52, 0, 50, 0, 50, 0, 51, 50, 0, 50, 52, 0, 50, 152, 152, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 0, 50, 0, 51, 50, 0, 0, 0, 0, 0, 0, 50, 34, 34, 34, 34, 26, 34, 0, 0, 0, 0, 0, 0, 50, 0, 50, 0, 51, 50, 0, 0, 0, 0, 0, 0, 50, 0, 50, 0, 34, 34, 34, 0, 0, 0, 0};
const uint8_t times[255] = {52, 50, 50, 50, 50, 52, 52, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 52, 50, 50, 50, 50, 50, 52, 50, 52, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 52, 50, 52, 50, 52, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 52, 50, 50, 50, 50, 50, 52, 50, 52, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 52, 50, 52, 50, 52, 50, 50, 50, 50, 52, 50, 50, 52, 50, 52, 50, 50, 50, 50, 50, 52, 50, 50, 52, 50, 50, 50, 50, 52, 50, 50, 52, 50, 52, 50, 50, 50, 50, 52, 52, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 52, 50, 52, 50, 50, 50, 52, 50, 50, 35, 34, 33, 35, 41, 33, 52, 50, 50, 50, 52, 50, 50, 52, 50, 50, 52, 50, 52, 50, 50, 50, 52, 50, 50, 52, 50, 50, 35, 34, 33, 52, 50, 50, 50, 52};

//const uint8_t notes[4] = {76, 64, 76, 64};
//const uint8_t delays[4] = {50, 50, 50, 50};
//const uint8_t times[4] = {200, 100, 200, 100};

//const uint8_t notes[211] = {66, 66, 66, 66, 66, 71, 55, 64, 60, 55, 60, 62, 61, 60, 60, 67, 71, 72, 69, 71, 69, 64, 65, 62, 64, 60, 55, 60, 62, 61, 60, 60, 67, 71, 72, 69, 71, 69, 64, 65, 62, 76, 75, 74, 71, 72, 64, 65, 67, 60, 64, 65, 76, 75, 74, 71, 72, 79, 79, 79, 0, 76, 75, 74, 71, 72, 64, 65, 67, 60, 64, 65, 68, 65, 64, 0, 76, 75, 74, 71, 72, 64, 65, 67, 60, 64, 65, 76, 75, 74, 71, 72, 79, 79, 79, 0, 76, 75, 74, 71, 72, 64, 65, 67, 60, 64, 65, 68, 65, 64, 0, 68, 68, 68, 68, 70, 67, 64, 64, 60, 68, 68, 68, 68, 70, 67, 0, 68, 68, 68, 68, 70, 67, 64, 64, 60, 66, 66, 66, 66, 66, 71, 55, 64, 60, 55, 60, 62, 61, 60, 60, 67, 71, 72, 69, 71, 69, 64, 65, 62, 64, 60, 55, 60, 62, 61, 60, 60, 67, 71, 72, 69, 71, 69, 64, 65, 62, 72, 69, 64, 64, 65, 72, 72, 65, 67, 77, 77, 77, 76, 77, 72, 69, 65, 64, 72, 69, 64, 64, 65, 72, 72, 65, 67, 74, 74, 74, 72, 71, 67, 48};
//const uint8_t delays[211] = {0, 0, 50, 52, 0, 50, 152, 152, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 204, 0, 0, 0, 50, 52, 0, 0, 52, 0, 0, 103, 0, 0, 0, 50, 52, 50, 0, 255, 0, 0, 0, 0, 50, 52, 0, 0, 52, 0, 0, 103, 103, 101, 255, 205, 0, 0, 0, 50, 52, 0, 0, 52, 0, 0, 103, 0, 0, 0, 50, 52, 50, 0, 255, 0, 0, 0, 0, 50, 52, 0, 0, 52, 0, 0, 103, 103, 101, 255, 102, 0, 50, 52, 0, 50, 0, 50, 0, 152, 0, 50, 52, 0, 0, 255, 154, 0, 50, 52, 0, 50, 0, 50, 0, 152, 0, 50, 52, 0, 50, 152, 152, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 0, 50, 103, 50, 0, 50, 0, 152, 34, 34, 34, 34, 34, 34, 0, 50, 0, 152, 0, 50, 103, 50, 0, 50, 0, 152, 0, 50, 0, 34, 34, 34, 152};
//const uint8_t times[211] = {52, 50, 50, 50, 50, 52, 52, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 50, 50, 52, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 52, 50, 50, 50, 50, 52, 0, 50, 50, 52, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 52, 0, 50, 50, 52, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 52, 50, 50, 50, 50, 52, 0, 50, 50, 52, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 52, 0, 52, 50, 50, 50, 50, 52, 50, 50, 52, 52, 50, 50, 50, 50, 50, 0, 52, 50, 50, 50, 50, 52, 50, 50, 52, 52, 50, 50, 50, 50, 52, 52, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 50, 52, 50, 50, 52, 35, 34, 33, 35, 34, 33, 52, 50, 50, 52, 52, 50, 50, 50, 52, 50, 50, 52, 52, 50, 50, 35, 34, 33, 52, 52};

//const uint8_t notes[156] = {50, 50, 50, 50, 50, 67, 0, 55, 52, 48, 53, 55, 54, 53, 52, 60, 64, 65, 62, 64, 60, 57, 59, 55, 55, 52, 48, 53, 55, 54, 53, 52, 60, 64, 65, 62, 64, 60, 57, 59, 55, 0, 55, 0, 60, 53, 52, 60, 77, 77, 77, 0, 55, 0, 60, 53, 56, 58, 60, 0, 0, 55, 0, 60, 53, 52, 60, 77, 77, 77, 0, 55, 0, 60, 53, 56, 58, 60, 0, 44, 51, 56, 55, 48, 44, 51, 56, 0, 44, 51, 56, 55, 48, 50, 50, 50, 50, 50, 67, 0, 55, 52, 48, 53, 55, 54, 53, 52, 60, 64, 65, 62, 64, 60, 57, 59, 55, 55, 52, 48, 53, 55, 54, 53, 52, 60, 64, 65, 62, 64, 60, 57, 59, 55, 48, 54, 60, 53, 60, 50, 53, 55, 74, 55, 60, 48, 54, 60, 53, 60, 55, 55, 55, 57, 59, 60};
//const uint8_t delays[156] = {0, 0, 50, 52, 0, 50, 255, 102, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 255, 0, 255, 104, 103, 204, 154, 52, 50, 0, 255, 51, 255, 104, 103, 154, 103, 101, 255, 255, 1, 255, 104, 103, 204, 154, 52, 50, 0, 255, 51, 255, 104, 103, 154, 103, 101, 255, 102, 101, 103, 50, 101, 204, 101, 103, 255, 205, 101, 103, 50, 101, 204, 0, 50, 52, 0, 50, 255, 102, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 101, 103, 103, 50, 52, 0, 50, 34, 34, 34, 51, 0, 52, 50, 0, 0, 101, 101, 103, 50, 152, 152, 101, 0, 85, 34, 152, 152, 101, 103, 50, 152, 152, 101, 0, 34, 34, 34};
//const uint8_t times[156] = {52, 50, 50, 50, 50, 52, 0, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 0, 50, 0, 50, 50, 50, 50, 50, 50, 52, 0, 50, 0, 50, 50, 50, 50, 52, 255, 0, 50, 0, 50, 50, 50, 50, 50, 50, 52, 0, 50, 0, 50, 50, 50, 50, 52, 0, 52, 50, 50, 52, 50, 52, 50, 50, 0, 52, 50, 50, 52, 50, 52, 50, 50, 50, 50, 52, 0, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 50, 50, 50, 50, 35, 34, 33, 52, 50, 50, 50, 50, 52, 50, 52, 50, 50, 52, 52, 52, 50, 52, 33, 52, 52, 52, 50, 50, 52, 52, 52, 50, 35, 34, 33, 52};

const uint16_t n_notes = sizeof(notes) / sizeof(notes[0]);

//const uint8_t *note_sets[3] = {notes, notes_1, notes_2};
//const uint8_t *delay_sets[3] = {delays, delays_1, delays_2};
//const uint8_t *time_sets[3] = {times, times_1, times_2};
//const uint16_t n_notes_in_set[3] = {sizeof(notes)/ sizeof(notes[0]), sizeof(notes_1)/ sizeof(notes_1[0]), sizeof(notes_2)/ sizeof(notes_2[0])};

const uint8_t notes_to_ticks_offset = 43;
const uint8_t notes_to_ticks[42] = {137, 129, 122, 115, 108, 102, 97, 91, 86, 81, 77, 72, 68, 64, 61, 57, 54, 51, 48, 45, 43, 40, 38, 36, 34, 32, 30, 28, 27, 25, 24, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12};

int16_t note_on = -1;
bool note_delay_done = false;
uint8_t delay_times_left = 0;
uint8_t note_times_left = 0;
uint16_t note_event_ticks = 0;

// for a last_ticks filter
//#define LTICKS_FILTER_MIN_EHZ (5*6*7)
//#define LTICKS_FILTER_MAX_EHZ (100*6*7)
//#define LTICKS_FILTER_MIN_N 2
//#define LTICKS_FILTER_MAX_N 32
#define FILTER_SIZE 8
#define FILTER_SIZE_LOG2 3
uint8_t last_subticks_buffer[FILTER_SIZE] = { 0 };
uint8_t last_subticks_buffer_i = 0;
//uint8_t filter_size = 6;
//uint16_t last_ticks_sorted[LTICKS_FILTER_N] = { 0 };

#define PREEMPT_MAX_CHANGE (40)
#define PREEMPT_MIN_CHANGE (4)
#define PREEMPT_DEFAULT 6
#define PREEMPT_MAX_PREEMPT 28 // out of 128 for a full commutation
#define PREEMPT_MAX_PREEMPT_NEG 0 //-8 // out of 128 for a full commutation
int16_t preemption_factor = 0;

bool comparator_fired = false;

// current state of each motor, in terms of which coil is power, ground, and floating
uint8_t motor_state = 2;

volatile uint16_t current_limit = 2048;

int16_t user_duty_cycle = 0; // this is the one directly set by commands
uint16_t target_duty_cycle = 0;
uint16_t start_duty_cycle = 20; // 120 for a very fast start when tuned!

uint16_t motor_min_ehz = 0; // minimum hz for transitions in case we miss commutation, also for synchronous start up
uint16_t ticks_per_transition = 100;
uint16_t last_transition_ticks = 0; // for timing transitions
uint16_t last_duty_adjust_ticks = 0; // for current pid
uint16_t last_hz_ticks = 0; // for hz measurement
uint16_t align_start_ticks = 0; // synchronous starting
uint16_t sync_start_ticks = 0; // synchronous starting
uint16_t sync_end_ticks = 0;
bool sync_done = false;
bool duty_adjust_wait_done = false;
bool align_started = false;
bool align_done = false;

// incremented at whatever our pwm frequency
volatile uint16_t ticks_pwm_cycles = 0;
volatile uint16_t update_rate;
uint16_t main_loop_update_count = 0;

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

uint16_t adc_val = 0;
ISR(ADC_vect) {
    adc_val = ADC;
    if (adc_aux_i == 0) {
        // on the order of 20Hz low pass
        //motor_voltage = ((motor_voltage * 3) >> 2) + (adc_val << 3);
        motor_voltage = ((motor_voltage * 7) >> 3) + adc_val;
        //motor_voltage_raw = (motor_voltage_raw * 127 + (adc_val << 8) + 64) >> 7;
    } else {
        motor_current = ((motor_current * 7) >> 3) + (adc_val << 1);
        //motor_current_raw = (motor_current_raw * 127 + (adc_val << 8) + 64) >> 7;
        // no filtering here
        startup_current_raw = adc_val;
    }
}

static void configure_adc(void) {
    PRR0 &= ~_BV(PRADC); // enable in power reduction register
    ADCSRA = 0;
    ADCSRA |= _BV(ADIE); // interrupt enable
    ADCSRA |= _BV(ADPS2) | _BV(ADPS0); // 1/32 clock prescaler, 8mhz to 250khz
    // a conversion while turning ADC on takes 25 ADC cycles, so 250khz/25 = 10khz

    ADMUX = _BV(REFS0) | MOTOR_V_ADC; // AVCC reference

    DIDR0 = 0xff; // disable all digital input buffers; use PORTC only for analog! (and not using PORTE2,3)
}

static void update_motor_state(void);
static void delay_increment_motor_state(void);
static void increment_motor_state(void);

//ISR(ANALOG_COMP_vect) {
    ////delay_increment_motor_state();
    ////TRIGGER_TOGGLE();
//
    //// interrupt off so we don't fire again until the next expected zero crossing
    ////ACSR = 0;
    ////comparator_fired = true;
//}

static void configure_comparators(void) {
    // interrupts disabled (but comparator still on!) to begin with
    // we want to carefully control when we are looking for which kind of edge
    ACSR = 0;
    //ACSRB = _BV(ACOE); // enable ACO analog comparator output

    ADCSRB = _BV(ACME); // enable ADC multiplexed input
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
    if (i2c_read_buf_i < 3) {
        return;
    }

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
            set_send_reply(i2c_out_buf, motor_current);
            //set_send_reply(i2c_out_buf, motor_delta_ehz);
            break;
        case READ_MEASURED_VOLTAGE_MSG:
            set_send_reply(i2c_out_buf, motor_voltage);
            //set_send_reply(i2c_out_buf, preemption_factor);
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
            //set_send_reply(i2c_out_buf, startup_current_raw);
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
    uint8_t twi_status = TWSR1 & 0xF8;

    switch (twi_status) {
    case 0x60: // controller is going to write to us
        ; static int counts1 = 0;
        counts1++;
        i2c_read_buf_i = 0;
        TWCR1 |= _BV(TWINT1); // submit TWI settings
        break;
    case 0x80: // byte written by controller
        ; static int counts2 = 0;
        counts2++;
        i2c_read_buf[i2c_read_buf_i] = TWDR1;
        i2c_read_buf_i++;
        i2c_process_read();
        TWCR1 |= _BV(TWINT1); // submit TWI settings
        break;
    case 0xA8: // initial byte requested by controller
        ; static int counts3 = 0;
        counts3++;
        i2c_out_buf_i = 0;
        // fall through
    case 0xB8: // following byte requested by controller
        ; static int counts4 = 0;
        if (twi_status == 0xB8) {
            counts4++;
        }
        if (i2c_out_bytes > i2c_out_buf_i) {
            TWDR1 = i2c_out_buf[i2c_out_buf_i];
            i2c_out_buf_i++;
            // mark this message as completed!
            if (i2c_out_bytes == i2c_out_buf_i) {
                i2c_out_bytes = 0;
            }
        } else {
            TWDR1 = 0xff; // how we indicate a "no data" byte
        }
        if (i2c_out_bytes == 0) {
            // message is complete with this last byte
            TWCR1 &= ~_BV(TWEA1); // disable acknowledge
            TWCR1 |= _BV(TWINT1); // submit TWI settings
            TWCR1 |= _BV(TWEA1); // re-enable acknowledge
        } else {
            TWCR1 |= _BV(TWINT1); // submit TWI settings
        }
        break;
    case 0xC8:
        ; static int counts5 = 0;
        counts5++;
        // master wants more, but we have no more! whatever!
        TWDR1 = 0xff; // how we indicate a "no data" byte
        TWCR1 &= ~_BV(TWEA1); // disable acknowledge
        TWCR1 |= _BV(TWINT1); // submit TWI settings
        TWCR1 |= _BV(TWEA1); // re-enable acknowledge
        break;
    case 0xC0:
        ; static int counts6 = 0;
        counts6++;
        // Our data byte is sent and we got a not-acknowledge.
        TWCR1 |= _BV(TWINT1); // submit TWI settings
        break;
    case 0xA0:
        ; static int counts7 = 0;
        counts7++;
        // STOP or repeated START condition received
        TWCR1 |= _BV(TWINT1); // submit TWI settings
        break;
    default: ;
        //static volatile uint8_t last_different = 0;
        //last_different = twi_status;
        TWCR1 |= _BV(TWINT1); // submit TWI settings
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
    //ACSR = 0;

    uint16_t ticks = ticks_pwm_cycles + 1;
    ticks_pwm_cycles = ticks;
    if (ticks == 0) {
        main_loop_update_count = 0;
    }
    //if (ticks % ticks_per_phase == 0) {
        //TRIGGER_ON();
    //} else {
        //TRIGGER_OFF();
    //}
    TRIGGER_TOGGLE();
    // This way of writing that ticks >= scheduled_transition_ticks handles wrap-around when scheduled_transition_ticks is close to 65535
    if (has_scheduled_tick && ticks - scheduled_transition_ticks <= 16) {
        if (!has_reported_tick) {
            has_reported_tick = true;
            // toggle a debug output
        }
        if (sync_done) {
            increment_motor_state();
        }
    }
}

static void start_comparator(void) {
    // time to check for zero crossing
    // we set which rising edge and enabled the interrupt

    // high goes by A, A, B, B, C, C
    // low goes by  B, C, C, A, A, B
    // coil c is floating in states 0 (falling below zero), 3 (rising back up)
    // coil b is floating in states 1 (rising), 4 (falling)
    // coil a is floating in states 2 (falling), 5 (rising)

    switch (motor_state) {
    case 0: // coil c
    case 3:
        ADCSRA &= ~_BV(ADEN); // disable ADC
        ADCSRB = _BV(ACME);
        ADMUX = _BV(REFS0) | MOTOR_C_ADC; // w/ AVCC reference
        break;
    case 1: // coil b
    case 4:
        ADCSRA &= ~_BV(ADEN); // disable ADC
        ADCSRB = _BV(ACME);
        ADMUX = _BV(REFS0) | MOTOR_B_ADC; // w/ AVCC reference
        break;
    case 2: // coil a
    case 5:
        // coil a on the default comparator negative input!
        // we can also use the ADC now! (whose enabling enables the correct comparator input)
        // ADCSRA |= ~_BV(ADEN); // enable ADC
        ADCSRB = 0; // disable analog comparator use of ADC MUX
        start_aux_adc_read();
        break;
    }
}

ISR(TIMER3_COMPB_vect) {
    if (target_duty_cycle == 0) {
        // we still want to use the ADC even when it isn't part of the normal commutation cycle
        motor_state = 2;
        start_comparator();
    }
    // sample and hold the analog comparator

    uint8_t coil_high = (ACSR & _BV(ACO)) == 0;
    //if (ACSR & _BV(ACO)) {
        //TRIGGER2_ON();
    //} else {
        //TRIGGER2_OFF();
    //}

    // high goes by A, A, B, B, C, C
    // low goes by  B, C, C, A, A, B
    // coil c is floating in states 0 (falling below zero), 3 (rising back up)
    // coil b is floating in states 1 (rising), 4 (falling)
    // coil a is floating in states 2 (falling), 5 (rising)
    // 0, 1, 0, 1, 0, 1

    if (!comparator_fired) {
        if (motor_state % 2 == 0) {
            // waiting for falling edge
            if (!coil_high) {
                comparator_fired = true;
                delay_increment_motor_state();
            }
        } else {
            // waiting for rising edge
            if (coil_high) {
                comparator_fired = true;
                delay_increment_motor_state();
            }
        }
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
    COMP_CHECK_COUNT = MAX_DUTY_CYCLE / 2 + COMP_DELAY;
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
    // low goes by  B, C, C, A, A, B
    switch (motor_state) {
    case 0:
        MOT_HC_COUNT = 0;
        MOT_HA_COUNT = set_duty_cycle;
        //MOT_LB_ON();
        break;
    case 1:
        //MOT_HA_COUNT = set_duty_cycle;
        MOT_LB_OFF();
        MOT_LC_ON();
        break;
    case 2:
        MOT_HA_COUNT = 0;
        MOT_HB_COUNT = set_duty_cycle;
        //MOT_LC_ON();
        break;
    case 3:
        //MOT_HB_COUNT = set_duty_cycle;
        MOT_LC_OFF();
        MOT_LA_ON();
        break;
    case 4:
        MOT_HB_COUNT = 0;
        MOT_HC_COUNT = set_duty_cycle;
        //MOT_LA_ON();
        break;
    case 5:
        //MOT_HC_COUNT = set_duty_cycle;
        MOT_LA_OFF();
        MOT_LB_ON();
        break;
    }
}

// how big of a filter should we use for our current speed?
// linear interpolation between min and max
//static uint8_t lticks_filter_size(void) {
    //if (motor_ehz < LTICKS_FILTER_MIN_EHZ) {
        //return LTICKS_FILTER_MIN_N;
    //} else if (motor_ehz > LTICKS_FILTER_MAX_EHZ) {
        //return LTICKS_FILTER_MAX_N;
    //}
    //uint16_t rounding = (LTICKS_FILTER_MAX_N - LTICKS_FILTER_MIN_N) / (LTICKS_FILTER_MAX_EHZ - LTICKS_FILTER_MIN_EHZ) / 2;
    //return LTICKS_FILTER_MIN_N + (motor_ehz - LTICKS_FILTER_MIN_EHZ + rounding)
                               //* (LTICKS_FILTER_MAX_N - LTICKS_FILTER_MIN_N)
                               /// (LTICKS_FILTER_MAX_EHZ - LTICKS_FILTER_MIN_EHZ);
//}

// how many degrees ahead of time to schedule the tick
// the units are 1/128 of a commutation, or about 2.8 degrees
// can be both positive or negative (for deceleration)
// linear interpolation between values
uint16_t abs_delta = 0;
static int16_t degree_preemption_factor(void) {
    // we use the full raw values to see small changes
    // and we are looking at percentage change here, out of 64
    abs_delta = (motor_delta_ehz > 0) ? motor_delta_ehz : -motor_delta_ehz;

    if (abs_delta > PREEMPT_MAX_CHANGE) {
        if (motor_delta_ehz > 0) {
            return PREEMPT_MAX_PREEMPT;
        }
        return PREEMPT_MAX_PREEMPT_NEG;
    }
    if (abs_delta < PREEMPT_MIN_CHANGE) {
        return PREEMPT_DEFAULT;
    }
    //int16_t rounding = PREEMPT_MAX_PREEMPT / (PREEMPT_MAX_CHANGE - PREEMPT_MIN_CHANGE) / 2;
    int16_t factor = (abs_delta - PREEMPT_MIN_CHANGE)
                     * PREEMPT_MAX_PREEMPT
                     / (PREEMPT_MAX_CHANGE - PREEMPT_MIN_CHANGE);
    factor = (motor_delta_ehz > 0) ? factor : -factor + PREEMPT_DEFAULT;
    if (factor < PREEMPT_MAX_PREEMPT_NEG) {
        return PREEMPT_MAX_PREEMPT_NEG;
    }
    return factor;
}

static void delay_increment_motor_state(void) {
    if (!has_scheduled_tick) {
        // read our sub-tick position
        //uint16_t now_subticks = TCNT1;

        uint16_t now_ticks = ticks_pwm_cycles;
        uint16_t full_ticks_change = now_ticks - last_zero_ticks;
        if (full_ticks_change > 255) {
            full_ticks_change = 255;
        }
        //int16_t subticks_change = (int16_t)now_subticks - (int16_t)last_zero_subticks;
        //uint16_t total_subticks_change = full_ticks_change * PWM_PERIOD + subticks_change;

        last_subticks_buffer[last_subticks_buffer_i] = full_ticks_change;
        last_subticks_buffer_i = (last_subticks_buffer_i + 1) % FILTER_SIZE;

        scheduled_transition_ticks = last_transition_ticks + ticks_per_phase;
        has_scheduled_tick = true;
        has_reported_tick = false;
        last_zero_ticks = now_ticks;
        //last_zero_subticks = now_subticks;
    }
}

static void increment_motor_state(void) {
    // AVR has no divide instruction, so we can't really do (motor_state + 1) % 6
    motor_state++;
    if (motor_state >= 6) {
        motor_state -= 6;
    }

    comparator_fired = false;
    start_comparator();
    update_motor_state();
    //TRIGGER2_TOGGLE();

    motor_count++;
    last_transition_ticks = ticks_pwm_cycles;
    has_scheduled_tick = false;
}

uint16_t update_now_ticks = 0;
bool restart_sync = false;
static void update_motor(void) {
    //uint16_t motor_current = motor_current_raw >> 8;
    restart_sync = sync_done && (motor_current < MIN_MOTOR_CURRENT || motor_ehz < RETRY_MIN_EHZ);
    if (((user_duty_cycle == 0 || current_limit == 0) && (true || !sync_done || target_duty_cycle <= MIN_DUTY_CYCLE)) || restart_sync) {
        // turn off all the outputs
        target_duty_cycle = 0;
        set_duty_cycle = 0;
        update_motor_state();
        STATUS_OFF();
        MOTOR_ENABLE_OFF();

        // reset state to not-moving
        //motor_ehz_raw = 0;
        motor_state = 2; // so we can use ADC
        align_started = false;
        align_done = false;
        sync_done = false;
        duty_adjust_wait_done = false;
        motor_min_ehz = 0;
        ticks_per_transition = 50000;
        last_duty_adjust_ticks = 0;
        note_on = -1;
        note_times_left = 0;
        delay_times_left = 0;
        return;
    }

    // turn status light on
    STATUS_ON();
    MOTOR_ENABLE_ON();

    // make sure we have locked onto the initial rotor position
    if (!align_started) {
        //determine_rotor_position();
        align_started = true;
        align_start_ticks = ticks_pwm_cycles;
        motor_min_ehz = SYNCHRONOUS_EHZ_START;
        ticks_per_transition = (PWM_FREQ + motor_min_ehz / 2) / motor_min_ehz;
        target_duty_cycle = start_duty_cycle;
    }

    // manage our target duty cycle, and thus the rate at which the motor changes velocity
    update_now_ticks = ticks_pwm_cycles;

    bool time_to_adjust = update_now_ticks - last_duty_adjust_ticks > DUTY_ADJUST_TICKS;
    if ((current_limit > 0 || target_duty_cycle > MIN_DUTY_CYCLE) && time_to_adjust) {
        // additional wait once we get into sync mode for it to stabilize
        if (duty_adjust_wait_done) {
            int16_t original_diff = (int16_t)user_duty_cycle - (int16_t)target_duty_cycle;
            if (original_diff != 0) {
                int16_t diff_percent = (original_diff << 6) / (int16_t)target_duty_cycle;
                if (diff_percent > MAX_DUTY_INC_PERCENT) {
                    diff_percent = MAX_DUTY_INC_PERCENT;
                }
                if (diff_percent < -MAX_DUTY_DEC_PERCENT) {
                    diff_percent = -MAX_DUTY_DEC_PERCENT;
                }
                int16_t diff = (diff_percent * (int16_t)target_duty_cycle + 63) >> 6;
                if (diff == 0) {
                    diff = original_diff > 0 ? 1 : -1;
                }
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
        } else if (sync_done) {
            if (update_now_ticks - sync_end_ticks > DUTY_ADJUST_WAIT_TICKS) {
                duty_adjust_wait_done = true;
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
        last_duty_adjust_ticks = update_now_ticks;
    }

    // asynchronous commutation managed by the comparator callbacks
    if (sync_done) {
        return;
    }

    if (!align_done) {
        if (update_now_ticks - align_start_ticks > ALIGN_STEP_TICKS) {
            sync_start_ticks = ticks_pwm_cycles;
            align_done = true;
        }
    }

    // Music modes!
    if (user_duty_cycle == 1) {
        if (update_now_ticks - note_event_ticks > (PWM_FREQ / 58 / 6)) {
            if (note_delay_done || note_on == -1) {
                // PWM_FREQ / 58 / 6 = 45 (w/ 16khz)
                if (note_times_left == 0) { //update_now_ticks - note_event_ticks > times[note_on] * (PWM_FREQ / 50 / 6)) {
                    note_on++;
                    if (note_on >= (int16_t)n_notes) {
                        note_on = -1;
                        user_duty_cycle = 0;
                    }

                    note_times_left = times[note_on];
                    delay_times_left = delays[note_on];

                    note_delay_done = false;

                    motor_min_ehz = 0;
                    ticks_per_transition = 50000;
                    target_duty_cycle = 0;
                    set_duty_cycle = target_duty_cycle;
                } else {
                    if (target_duty_cycle > start_duty_cycle / 2) {
                        static uint8_t countour_count = 0;
                        if (countour_count == 4) {
                            countour_count = 0;
                            target_duty_cycle--;
                            set_duty_cycle = target_duty_cycle;
                        } else {
                            countour_count++;
                        }
                    }
                    note_times_left--;
                }
            } else {
                //if (update_now_ticks - note_event_ticks > delays[note_on] * (PWM_FREQ / 50 / 6)) {
                if (delay_times_left == 0) {
                    note_delay_done = true;

                    // motor_min_ehz = notes_to_freqs[notes[note_on] - notes_to_freqs_offset];
                    // ticks_per_transition = (PWM_FREQ + motor_min_ehz / 2) / motor_min_ehz;

                    ticks_per_transition = notes_to_ticks[notes[note_on] - notes_to_ticks_offset];

                    target_duty_cycle = start_duty_cycle;
                    set_duty_cycle = target_duty_cycle;
                } else {
                    delay_times_left--;
                    target_duty_cycle = 0;
                    set_duty_cycle = target_duty_cycle;
                }
            }
            note_event_ticks = update_now_ticks;
        }

        //uint8_t test_note = 84;
        //uint16_t test_ticks = notes_to_ticks[test_note - notes_to_ticks_offset];
        //if (ticks_per_transition != test_ticks) {
            ////motor_min_ehz = test_ehz;
            //ticks_per_transition = test_ticks;
            //target_duty_cycle = MIN_DUTY_CYCLE;
            //set_duty_cycle = target_duty_cycle;
        //}
    } else if (align_done && update_now_ticks - sync_start_ticks > SYNCHRONOUS_STEP_TICKS) {
        if (motor_current >= MOTOR_RUNNING_CURRENT_THRESH) {
            // condition above to make sure the motor has started turning
            // otherwise we stay in slow synchronous state for debugging
            motor_min_ehz = motor_min_ehz + SYNCHRONOUS_EHZ_INC;

            if (motor_min_ehz >= SYNCHRONOUS_EHZ_END) {
                //motor_min_ehz = SYNCHRONOUS_EHZ_END;

                sync_done = true;
                motor_min_ehz = SYNCHRONOUS_EHZ_START;

                last_duty_adjust_ticks = update_now_ticks;
                sync_end_ticks = update_now_ticks;
            }

            ticks_per_transition = (PWM_FREQ + motor_min_ehz / 2) / motor_min_ehz;
        }
        sync_start_ticks = update_now_ticks;
    }

    if ((update_now_ticks - last_transition_ticks) >= ticks_per_transition) {
        increment_motor_state();
    }
}

void update_ticks_per_phase(void) {
    uint16_t subticks_sum = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        subticks_sum += last_subticks_buffer[i];
    }
    uint16_t subticks_per_phase = subticks_sum / FILTER_SIZE;
    // advance the time interval by some number of degrees
    // this is necessary for fast acceleration/deceleration
    preemption_factor = degree_preemption_factor();
    subticks_per_phase = ((subticks_per_phase * (128 - preemption_factor)) >> 7);

    ticks_per_phase = subticks_per_phase;
}

int main(void) {
    configure_directions();
    configure_adc();
    configure_comparators();

    configure_i2c();
    configure_pwm();

    sei(); // global interrupt enable

    //user_duty_cycle = 100; //TODO REMOVE WHEN DONE DEBUGGING THIS!

    bool waiting_for_update_rate = true;

    while (true) {
        uint16_t now_ticks = ticks_pwm_cycles;
        if (waiting_for_update_rate && now_ticks >= 32767) {
            waiting_for_update_rate = false;
            update_rate = (main_loop_update_count >> 8) * (PWM_FREQ >> 7);
        } else if (now_ticks < 32767) {
            waiting_for_update_rate = true;
        }

        if ((now_ticks - last_hz_ticks) >= PWM_FREQ / HZ_MEASUREMENT_HZ) {
            //last_motor_ehz_raw = motor_ehz_raw;
            //uint32_t new_ehz = (motor_count * HZ_MEASUREMENT_HZ + 3);
            //motor_ehz_raw = (motor_ehz_raw * 15 + (new_ehz << 8) + 7) >> 4;
            //motor_ehz = motor_ehz_raw >> 8;
            //motor_count = 0;
            //last_hz_ticks = now_ticks;

            // motor_ehz * 7 / 8 + new_estimate * 1 / 8, so motor_count * 4 == new_estimate * 1 / 8
            // and thus, new_estimate = motor_count * 4 * 8 = motor_count * HZ_MEASUREMENT_HZ / 2
            motor_ehz = ((motor_ehz * 7) >> 3) + (motor_count << 2); // motor_ehz = motor_count * HZ_MEASUREMENT_HZ / 2 and filtered
            int16_t delta_ehz = (int16_t)motor_count - (int16_t)last_motor_count;
            motor_delta_ehz = ((motor_delta_ehz * 7 + 3) >> 3) + (delta_ehz << 3); // plus 3 helps with rounding for negative numbers
            last_motor_count = motor_count;
            motor_count = 0;
            last_hz_ticks = now_ticks;
        }

        //update_rate = main_loop_update_count / (uint8_t)(now_ticks >> 7); //  * (PWM_FREQ >> 7) (i.e. 125)
        update_motor();
        update_ticks_per_phase();
        main_loop_update_count++;
    }
    return 0;
}

