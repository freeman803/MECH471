/**
 * @file PID.h
 * @brief This will be the file defines our PID Library
 */
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/

 #include <Arduino.h>
 #include <avr/io.h> 
 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;

    float output_min;
    float output_max;

    float dt;
} PID_t;

void PID_init(PID_t *pid, float kp, float ki, float kd, float dt);
float PID_compute(PID_t *pid, float setpoint, float measurement);
 