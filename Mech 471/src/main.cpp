/*
 * MECH 471 / 6621 - Autonomous RC Car Controller
 * Description: Main control loop featuring Cruise Control, Traction Control (TCS), 
 * and a Maneuver Sequencer (including Donut mode) via a 50Hz priority stack.
 */

#include <Arduino.h>
#include <HW\digital.h>
#include <HW\timer.h>
#include <HW\ADC_Buffer.h>
#include <HW\define.h>
#include <HW\PID.h>
#include <math.h>

#define BIT(a) (1UL << (a))

// --- Constants ---
static const float ADC_TO_V   = 5.0f / 1023.0f;  
static const float V_BAT      = 12.0f;            
static const float PW_NEUTRAL = 1500.0f;          
static const float PW_RANGE   = 500.0f;           
static const float PW_MIN     = 1000.0f;          
static const float PW_MAX     = 2000.0f;          

// Slip ratio thresholds 
static const float TRACTION_SLIP_THRESHOLD = -0.20f; 
static const float TARGET_TRACTION_SLIP    = -0.15f; 
static const float TARGET_BRAKE_SLIP       =  0.15f; 
static const float LAUNCH_SPEED_THRESHOLD = 300.0f;   // rad/s — exit launch once up to speed scaled for HIL3, tested through trial and error
static const float LAUNCH_Va = 6.0f;                // open-loop throttle during launch, 7 would cause alot of slip
static const uint32_t HIL_READY_DELAY_MS = 3000UL;  //delay time to allow Profs HIL to be ready and not provide bad readings at the start of the test

// --- Global Variables ---
volatile bool control_flag = false;
bool launch_active = true;

PID_t speed_pid;
PID_t traction_pid;
PID_t brake_pid;

time_differencePID speed_time;
time_differencePID traction_time;
time_differencePID brake_time;

uint8_t log_tick = 0;

// ---------------------------------------------------------
// Interrupt Service Routines (50 Hz Timer 1)
// ---------------------------------------------------------
ISR(TIMER1_OVF_vect) {
    PORTD |=  BIT(PORTD7);   // D7 HIGH: start u1 pulse (drive motor)
    PORTB |=  BIT(PORTB0);   // D8 HIGH: start u2 pulse (steering)
    control_flag = true;     // flag the main loop to run
}

ISR(TIMER1_COMPA_vect) {
    PORTD &= ~BIT(PORTD7);   // D7 LOW: end u1 pulse
}

ISR(TIMER1_COMPB_vect) {
    PORTB &= ~BIT(PORTB0);   // D8 LOW: end u2 pulse
}

// ---------------------------------------------------------
// Signal Helpers
// ---------------------------------------------------------
float adc_to_omega(uint16_t counts) {
    return (counts * ADC_TO_V - 2.5f) * 324.0f; //scale based off HIL3 max speed of 810 rad/s at 5V (2.5V = 0 rad/s)
}

uint16_t va_to_pw(float Va) {
    float pw = (Va / V_BAT) * PW_RANGE + PW_NEUTRAL;
    if (pw < PW_MIN) pw = PW_MIN;
    if (pw > PW_MAX) pw = PW_MAX;
    return (uint16_t)pw;
}

float compute_slip(float w_front, float w_rear) {
    const float MIN_FRONT_SPEED = 30.0f;

    if (fabsf(w_front) < MIN_FRONT_SPEED) {
        return 0.0f;
    }

    float S = (w_front - w_rear) / fabsf(w_front);

    if (S >  3.0f) S =  3.0f;
    if (S < -3.0f) S = -3.0f;

    return S;// computes S but allows us to get more clean values of S given the testing bench without
}

// ---------------------------------------------------------
// Setup 
// ---------------------------------------------------------
void setup() {
    Serial.begin(1000000); // 1 Mbaud for simulator 2
    //Serial.begin(2000000); // 2 Mbaud for simulator 3

    // Initialize Hardware 
    timer2_init();         // Timer2 CTC: 1 ms millis_ clock
    init_servoPWM();       // Timer1 interrupt mode for servos
    init_buffer();         // ADC interrupt buffers (calls sei() internally)

    // Speed controller
    PID_init(&speed_pid, 0.035f, 0.008f, 0.001f);
    speed_pid.integral_min = -25.0f; 

    // Traction controller 
    PID_init(&traction_pid, 6.0f, 0.5f, 0.1f);
    traction_pid.integral_min = -25.0f;

    // Braking controller 
    PID_init(&brake_pid, 6.0f, 0.5f, 0.1f);
    brake_pid.integral_min = -25.0f;

    init_dt(&speed_time);
    init_dt(&traction_time);
    init_dt(&brake_time);

    Serial.println(F("%time_s,y1_V,y2_V,y3_V,pw1_us,w_rear,S_control"));
}

// ---------------------------------------------------------
// Main Control Loop (Executes at 50Hz)
// ---------------------------------------------------------
void loop() {
    
    // Wait for the Timer1 OVF ISR to trigger the control frame
    if (!control_flag) return;
    control_flag = false;

    if (millis_() < HIL_READY_DELAY_MS) {
        set_u1_pulse(1500);  // neutral throttle
        set_u2_pulse(1500);  // straight steering

        launch_active = true;

        update_dt(&speed_time);
        update_dt(&traction_time);
        update_dt(&brake_time);

        return; //holds neutral and delay until HIL is ready
    }
    // 1. Read sensors and convert to rad/s 
    uint16_t raw1 = buffer1_avg(); // Drive
    uint16_t raw3 = buffer3_avg(); // Front Right
    uint16_t raw5 = buffer5_avg(); // Front Left

    if (raw5 < 550 || raw5 > 675) {
        set_u1_pulse(1500);  // neutral throttle
        set_u2_pulse(1500);  // straight steering
        Serial.println(F("NO HIL SIGNAL"));
        return;// debugging tool to check if the car is reading the HIL, checks 3V for y3 which is fixed in HIL
    }

    float w_rear = adc_to_omega(raw1);
    float w_fr   = adc_to_omega(raw3);  
    float w_fl   = adc_to_omega(raw5);  //can be logged still but not used for control in this version

    // Turn-aware control slip: outer wheel provides the best vehicle speed estimate
    //float w_ref = (fabs(w_fr) >= fabs(w_fl)) ? w_fr : w_fl; The profs HIL3 fixes FL to 3V so it would add inaccuracy
    float w_ref = w_fr;   // Professor HIL3: only y2/A3 is dynamic front wheel speed
    float S_control = compute_slip(w_ref, w_rear);

    // 2. Maneuver sequencer (Time-based states)
    uint32_t t = millis_() - HIL_READY_DELAY_MS; // scale time to start after HIL is ready
    float desired_speed = 0.0f;
    uint16_t steer_us = 1500;
    bool donut_mode = false;

    if      (t < 500)   { desired_speed =   0.0f; steer_us = 1500; } // Neutral
    else if (t < 4000)  { desired_speed =  200.0f; steer_us = 1500; } // Forward
    else if (t < 6000)  { desired_speed =  150.0f; steer_us = 1250; } // Turn Left
    else if (t < 8000)  { desired_speed =  200.0f; steer_us = 1500; } // Cruise 1
    else if (t < 9500)  { desired_speed =  150.0f; steer_us = 1750; } // Turn Right
    else if (t < 11000) { desired_speed =  200.0f; steer_us = 1500; } // Cruise 2
    else if (t < 13000) { desired_speed =   50.0f; steer_us = 1500; } // Slow Down
    else if (t < 16000) { desired_speed =   0.0f; steer_us = 1500; } // Brake
    else if (t < 18500) { desired_speed = -150.0f; steer_us = 1500; } // Reverse
    else if (t < 21000) { desired_speed =   0.0f; steer_us = 1500; } // Stop before donut
    else if (t < 25000) { desired_speed =   0.0f; steer_us = 2000; donut_mode = true; } 
    else                { desired_speed =   0.0f; steer_us = 1500; } // Final Stop

    // Output steering directly
    set_u2_pulse(steer_us);

    // 3. Safety priority stack
    
    // Dynamic Traction Threshold: Tighten threshold when steering because lateral grip is lower
    float steer_offset = fabs((float)steer_us - 1500.0f);
    float dynamic_threshold = TRACTION_SLIP_THRESHOLD + (steer_offset / 500.0f) * 0.10f;

    bool braking = (desired_speed <= 0.0f && w_rear > 1.0f);
    float Va = 0.0f;
    const char* mode = "SPEED";

    if (donut_mode) {
        // Priority 1: Donut (Bypass PIDs, full throttle)
        mode = "DONUT";
        Va = V_BAT;
        update_dt(&speed_time);
        update_dt(&traction_time);
        update_dt(&brake_time);
       
    } else if (desired_speed <= 0.0f) {
    // Priority 2: Braking / Reverse / Stop
    // When the maneuver sequence is not asking for forward motion,
    // disable launch logic so the next forward section can launch again.
    launch_active = true;

        if (braking) {
            // Active Braking Control
            // Use brake PID to control positive slip during deceleration.
            mode = "BRAKING";
            Va = PID_compute(&brake_pid, TARGET_BRAKE_SLIP, S_control, &brake_time);

            // Braking should only command neutral/reverse torque.
            // Standard Speed Control for stop/reverse conditions
            if (Va > 0.0f) Va = 0.0f;

            update_dt(&speed_time);
            update_dt(&traction_time);

        
        } else {
            if (desired_speed < 0.0f) {
            mode = "REVERSE";
            } else {
            mode = "STOP";
            }
            
            Va = PID_compute(&speed_pid, desired_speed, w_rear, &speed_time);

            update_dt(&traction_time);
            update_dt(&brake_time);
        }

} else if (launch_active) {
    // Priority 3: Launch Control
    // Open-loop throttle until the rear wheel reaches launch threshold.
    // This avoids the speed PID instantly saturating at startup.

    if (w_rear < LAUNCH_SPEED_THRESHOLD) {
        mode = "LAUNCHING";
        Va = LAUNCH_Va;

        update_dt(&speed_time);
        update_dt(&traction_time);
        update_dt(&brake_time);

    } else {
        // Launch complete: hand off to normal cruise/traction logic.
        mode = "SPEED";
        launch_active = false;

        Va = PID_compute(&speed_pid, desired_speed, w_rear, &speed_time);

        update_dt(&traction_time);
        update_dt(&brake_time);
    }

} else if (desired_speed > 0.0f && fabsf(w_ref) > 30.0f && S_control < dynamic_threshold) {
    // Priority 4: Traction Control Override
    // If rear wheel is spinning faster than the front reference,
    // reduce drive torque to target the desired slip ratio.
    mode = "TRACTION";
    Va = PID_compute(&traction_pid, TARGET_TRACTION_SLIP, S_control, &traction_time);

    // Traction control should reduce throttle, not command reverse.
    if (Va < 0.0f) Va = 0.0f;

    update_dt(&speed_time);
    update_dt(&brake_time);

}else {
        // Priority 5: Standard Cruise Control
        Va = PID_compute(&speed_pid, desired_speed, w_rear, &speed_time);
        update_dt(&traction_time);
        update_dt(&brake_time);
    }

    // 4. Saturate Va and output to drive servo 
    if (Va >  V_BAT) Va =  V_BAT;
    if (Va < -V_BAT) Va = -V_BAT;
    
    uint16_t pw1 = va_to_pw(Va);
    set_u1_pulse(pw1);

    // 5. Serial CSV log every 20 ticks (~400 ms) 
    /*if (++log_tick >= 20) {
        log_tick = 0;
        Serial.print(t * 0.001f, 3);       Serial.print(',');
        Serial.print(raw1 * ADC_TO_V, 3);  Serial.print(',');
        Serial.print(raw3 * ADC_TO_V, 3);  Serial.print(',');
        Serial.print(raw5 * ADC_TO_V, 3);  Serial.print(',');
        Serial.print(pw1);                 Serial.print(',');
        Serial.print(w_rear, 2);           Serial.print(',');
        Serial.println(S_control, 3);
    }*/
   if (++log_tick >= 20) {
    log_tick = 0;

    Serial.print(F("t="));
    Serial.print(t * 0.001f, 2);

    Serial.print(F("s | mode="));
    Serial.print(mode);

    Serial.print(F(" | target="));
    Serial.print(desired_speed, 0);

    Serial.print(F(" actual="));
    Serial.print(w_rear, 0);

    Serial.print(F(" | S="));
    Serial.print(S_control, 3);

    Serial.print(F(" | Va="));
    Serial.print(Va, 1);
    Serial.print(F("V"));

    Serial.print(F(" | pw="));
    Serial.print(pw1);

    Serial.print(F(" | steer="));
    Serial.println(steer_us);
    }
}