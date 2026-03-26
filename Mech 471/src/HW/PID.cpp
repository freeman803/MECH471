/**
 * @file PID.cpp
 * @brief Module source that defines PID library
 */

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include "PID.h"
#include "HW\timer.h"

/******************************************************************************
 *                         P R I V A T E  V A R S
 ******************************************************************************/


/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
void PID_init(PID_t *pid, float kp, float ki, float kd)
{
    //define k values and set them equal to struct
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    //set errors and current terms to 0
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->derivative = 0.0f;
    pid->integrated_error = 0.0f;

    pid->output_min = 0.0f;
    pid->output_max = 100.0f; 
}

float PID_compute(PID_t *pid, float setpoint, float measurement,struct time_differencePID *time){
    float error = setpoint-measurement;

    pid->proportional = pid->kp*error;

    update_dt(time);
    float dt = time->dt / 1000.0f;

    pid->integrated_error += error*dt;
    pid->integral = pid->integrated_error*pid->ki;

    pid->derivative = ((error-pid->prev_error)/dt)*pid->kd;
    pid->prev_error = error;
    
    return (pid->proportional+pid->integral+pid->derivative);
}

