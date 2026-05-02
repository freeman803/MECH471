/**
 * @file PID.h
 * @brief This will be the file defines our PID Library
 */
#pragma once
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/


 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/
typedef struct {
    float kp;
    float ki;
    float kd;
    float proportional;
    float prev_measurement;
    float integral;
    float integrated_error;
    float prev_error;
    float prev_derivative;
    float derivative;
    float output_min;
    float output_max;
    float dt;
    float integral_max =  25;
    float integral_min = -25;
    float alpha = 0.5; //tuning parameter update step 
} PID_t;

void PID_init(PID_t *pid, float kp, float ki, float kd);
float PID_compute(PID_t *pid, float setpoint, float measurement,struct time_differencePID *time);
 