 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <Arduino.h>
#include <HW\digital.h>
#include<HW\timer.h>
#include <HW\ADC_Buffer.h>
#include <HW\ADC.h>
#include <HW\define.h>
#include <HW\PID.h>

float kp = 0.5;
float ki = 0.2;
float kd = 0.1;
float target_slip_ratio = 0.1;

PID_t rpm_pid;  // declare your instance
time_differencePID wheelrpm_time; 

//steering input from profs code 
int st_input = 1750;
long int freq = 50; // 50hz
int period = (int)(1/freq);
int duty_cycle = (int)(st_input/1000)/period;
int max_motorV = 12;

void setup() {
Serial.begin(9600);
// inits
init_buffer();
timer0_init();
ADC_INIT();
PID_init(&rpm_pid, kp, ki,kd);
}
void loop() {
//float PID_compute(PID_t *pid, float setpoint, float measurement,struct time_differencePID *time)
uint16_t whl_speed_fr = buffer0_avg();
uint16_t whl_speed_rr = buffer1_avg();
float slip_ratio = whl_speed_fr/(whl_speed_fr+whl_speed_rr);
float output_motorV = PID_compute(&rpm_pid,target_slip_ratio,slip_ratio,&wheelrpm_time);
init_fastPWM(freq, duty_cycle, pin9);//steering PWM straight line
init_fastPWM(freq, (int)(output_motorV/max_motorV), pin10); // motor output
delay_ms(100);
}