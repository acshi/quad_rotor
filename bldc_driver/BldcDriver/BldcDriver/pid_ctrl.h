#pragma once

#include <stdint.h>
#include <stdbool.h>

// we use 16-bit fixed point to get rid of floating-point math
typedef struct pid_ctrl {
    int32_t kb; // Proportional set point (bias) Gain
    int32_t kp; // Proportional error Gain
    int32_t ki; // Integral Gain
    int32_t kd; // Derivative Gain

    int32_t filtered_input;
    int32_t pid_output;

    int32_t b_term;   // proportional bias term (based on set point)
    int32_t p_term;   // proportional term of output (based on error)
    int32_t i_term;   // integral term
    int32_t d_term;   // derivative term

    // derivative debugging
    int32_t d_raw;
    int32_t d_out;

    int32_t set_point; // Target value to reach

    // for filtering
    int32_t input_alpha;
    int32_t derivative_alpha;
    int32_t output_alpha;

    int32_t i_term_min, i_term_max; // integral saturation
    int32_t output_min, output_max; //pid output saturation
    uint32_t update_hz; // rate in Hz of pid loop update
} pid_ctrl_t;


//Initialize the PID controller with gains
void pid_init(pid_ctrl_t *pid, float kb, float kp, float ki, float kd, uint32_t update_hz);

void pid_destroy(pid_ctrl_t *pid);

void pid_set_input_filter(pid_ctrl_t *pid, float cutoff_hz);
void pid_set_derivative_filter(pid_ctrl_t *pid, float cutoff_hz);
void pid_set_output_filter(pid_ctrl_t *pid, float cutoff_hz);

int32_t pid_next(pid_ctrl_t *pid, int32_t new_val);
void pid_set_setpoint(pid_ctrl_t *pid, int32_t set_point);

void pid_set_tunings(pid_ctrl_t *pid, float kb, float kp, float ki, float kd);
void pid_set_output_limits(pid_ctrl_t *pid, int32_t min, int32_t max);
void pid_set_integral_limits(pid_ctrl_t *pid, int32_t min, int32_t max);
void pid_reset_integrator(pid_ctrl_t *pid);
