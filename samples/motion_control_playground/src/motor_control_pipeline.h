#ifndef __MOTOR_CONTROL_PIPELINE_H
#define __MOTOR_CONTROL_PIPELINE_H

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "control_law.h"

struct motor_control_pipeline {
    const struct device *hw;
    struct siso_control_law *position_cl;
    float  last_time;
    float dt;
    float target_position;
    float current_position;
    float control_effort;
    int ticks;
    int sample_ratio;
    int reading_error;
    int writing_error;
};

int motor_control_pipeline_add_hw(struct motor_control_pipeline* cp, const struct device *hw);
int motor_control_pipeline_add_control(struct motor_control_pipeline* cp, struct siso_control_law *cl);
int motor_control_pipeline_remove_control(struct motor_control_pipeline* cp);
int motor_control_pipeline_set_position(struct motor_control_pipeline* cp, float target_position);
int motor_control_pipeline_register(struct motor_control_pipeline* cp, int sample_ratio);

#endif
