#include "pid_ctrl.h"

#include <stdlib.h>
#include <math.h>

void pid_init(pid_ctrl_t *pid, float kb, float kp, float ki, float kd, uint32_t update_hz) {
    pid->update_hz = update_hz;
    pid_set_tunings(pid, kb, kp, ki, kd);
}

void pid_set_input_filter(pid_ctrl_t *pid, float cutoff_hz) {
    float factor = 2 * M_PI * cutoff_hz / pid->update_hz;
    pid->input_alpha = (int32_t)(factor / (factor + 1) * 65536);
}

void pid_set_derivative_filter(pid_ctrl_t *pid, float cutoff_hz) {
    float factor = 2 * M_PI * cutoff_hz / pid->update_hz;
    pid->derivative_alpha = (int32_t)(factor / (factor + 1) * 65536);
}

void pid_set_output_filter(pid_ctrl_t *pid, float cutoff_hz) {
    float factor = 2 * M_PI * cutoff_hz / pid->update_hz;
    pid->output_alpha = (int32_t)(factor / (factor + 1) * 65536);
}

static int32_t constrain(int32_t in, int32_t min, int32_t max) {
    if (in < min) {
        return min;
    } else if (in > max) {
        return max;
    }
    return in;
}

int32_t pid_next(pid_ctrl_t *pid, int32_t new_val) {
    int32_t new_input = new_val << 16;
    if (pid->input_alpha > 0) {
        new_input = ((int64_t)pid->input_alpha * new_val + (int64_t)((1 << 16) - pid->input_alpha) * pid->filtered_input) >> 16;
    }

    int32_t error = pid->set_point - new_input;

    pid->b_term = ((int64_t)pid->kb * pid->set_point) >> 16;
    pid->p_term = ((int64_t)pid->kp * error) >> 16;
    pid->i_term += ((int64_t)pid->ki * error) >> 16;
    pid->i_term = constrain(pid->i_term, pid->i_term_min, pid->i_term_max);
    if (pid->kd != 0) {
        int32_t prevError = pid->set_point - pid->filtered_input;
        pid->d_raw = error - prevError;
        if (pid->derivative_alpha > 0) {
            pid->d_out = ((int64_t)pid->derivative_alpha * pid->d_raw + (int64_t)((1 << 16) - pid->derivative_alpha) * pid->d_out) >> 16;
        } else {
            pid->d_out = pid->d_raw;
        }
        pid->d_term = ((int64_t)pid->kd * pid->d_out) >> 16;
    } else {
        pid->d_term = 0;
    }

    pid->filtered_input = new_input;
    int32_t raw_out = (pid->b_term + pid->p_term + pid->i_term + pid->d_term) >> 16;
    int32_t out_value = constrain(raw_out, pid->output_min, pid->output_max);
    if (pid->output_alpha > 0) {
        pid->pid_output = ((int64_t)pid->output_alpha * pid->pid_output + (int64_t)((1 << 16) - pid->output_alpha) * out_value) >> 16;
    } else {
        pid->pid_output = out_value;
    }
    return pid->pid_output;
}

void pid_set_setpoint(pid_ctrl_t *pid, int32_t set_point) {
    pid->set_point = set_point << 16;
}

void pid_set_tunings(pid_ctrl_t *pid, float kb, float kp, float ki, float kd) {
    //scale gains by update rate in seconds for proper units
    //note we are using hz and not dt.
    pid->kb = (int32_t)(kb * (1 << 16));
    pid->kp = (int32_t)(kp * (1 << 16));
    pid->ki = (int32_t)(ki * (1 << 16) / pid->update_hz);
    pid->kd = (int32_t)(kd * (1 << 16) * pid->update_hz);
}

void pid_set_output_limits(pid_ctrl_t *pid, int32_t min, int32_t max) {
    pid->output_min = min;
    pid->output_max = max;
}

void pid_set_integral_limits(pid_ctrl_t *pid, int32_t min, int32_t max) {
    pid->i_term_min = min << 16;
    pid->i_term_max = max << 16;
}

void pid_reset_integrator(pid_ctrl_t *pid) {
    pid->i_term = 0;
}
