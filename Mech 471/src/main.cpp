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
float st_input = 1750;
float freq = 50; // 50hz
float period = (1.0f/freq);
int duty_cycle = (int)((st_input/1000.0f)/period);
float max_motorV = 12.0f;
uint32_t k = 0;

void setup() {
Serial.begin(9600);
// inits
init_buffer(); // here we call adc init so dont need to call it in setup;
timer0_init();
PID_init(&rpm_pid, kp, ki,kd);
init_dt(&wheelrpm_time);

}
void loop() {
k++;
//float PID_compute(PID_t *pid, float setpoint, float measurement,struct time_differencePID *time)
float whl_speed_rear = buffer1_avg()*1.0f/1023.0f*5.0f;
float whl_speed_fr = buffer3_avg()*1.0f/1023.0f*5.0f;
float whl_speed_fl = buffer5_avg()*1.0f/1023.0f*5.0f;
float average_front = (whl_speed_fl+whl_speed_fr)/2.0f;
float slip_ratio = 0;
if (average_front>0) slip_ratio = (average_front-whl_speed_rear)/(average_front);
float output_motorV = PID_compute(&rpm_pid,target_slip_ratio,slip_ratio,&wheelrpm_time);
float duty = output_motorV / (float)max_motorV;
duty = constrain(duty, 0.0f, 100.0f);
init_fastPWM(freq, (int)duty, pin10); // motor output
if (k%1000 == 1){
    Serial.print("output_motorV: "); Serial.println(output_motorV);
    Serial.print("average_front: "); Serial.println(average_front);
    Serial.print("whl_speed_rear: "); Serial.println(whl_speed_rear);
    Serial.print("slip_ratio: "); Serial.println(slip_ratio);
    Serial.println();
}

delay_ms(1);
}