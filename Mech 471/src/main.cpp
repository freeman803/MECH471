/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <Arduino.h>
#include <HW\digital.h>
#include <HW\timer.h>
#include <HW\ADC_Buffer.h>
#include <HW\define.h>
#include <HW\PID.h>
#include <math.h>

/******************************************************************************
 *                              C O N S T A N T S
 ******************************************************************************/
static const float ADC_TO_V   = 5.0f / 1023.0f;  // pre-computed: multiply not divide
static const float V_BAT      = 12.0f;            // battery voltage (V)
static const float PW_NEUTRAL = 1500.0f;          // servo neutral (µs)
static const float PW_RANGE   = 500.0f;           // range each side of neutral (µs)
static const float PW_MIN     = 1000.0f;          // full reverse (µs)
static const float PW_MAX     = 2000.0f;          // full forward (µs)

// Slip ratio thresholds (S < 0 = wheelspin, S > 0 = wheel lock)
static const float TRACTION_SLIP_THRESHOLD = -0.20f; // activate traction when S < this
static const float TARGET_TRACTION_SLIP    = -0.15f; // traction setpoint
static const float TARGET_BRAKE_SLIP       =  0.15f; // braking setpoint

// Autonomous maneuver timing (ms from power-on)
static const uint32_t T_FORWARD_START =   500UL;
static const uint32_t T_TURN_L_START  =  4000UL;
static const uint32_t T_CRUISE1_START =  6000UL;
static const uint32_t T_TURN_R_START  =  8000UL;
static const uint32_t T_CRUISE2_START =  9500UL;
static const uint32_t T_SLOW_START    = 11000UL;
static const uint32_t T_BRAKE_START   = 13000UL;
static const uint32_t T_REVERSE_START = 16000UL;
static const uint32_t T_STOP_START    = 18500UL;
static const uint32_t T_DONUT_START   = 21000UL;  // allow 2.5 s to fully stop before donut
static const uint32_t T_DONUT_END     = 25000UL;  // 4 s donut phase

/******************************************************************************
 *                         D R I V E   S T A T E S
 ******************************************************************************/
typedef enum {
    STATE_NEUTRAL = 0,
    STATE_FORWARD,
    STATE_TURN_LEFT,
    STATE_CRUISE,
    STATE_TURN_RIGHT,
    STATE_SLOW_DOWN,
    STATE_BRAKE,
    STATE_REVERSE,
    STATE_STOP,
    STATE_DONUT   // traction control disabled, full throttle + full right steering lock (2000 µs)
} drive_state_t;

/******************************************************************************
 *                     T I M E R 1   I S R s
 * Timer1 runs at 2 MHz (prescaler /8). ICR1=39999 → 20 ms period (50 Hz).
 * OVF fires at BOTTOM (counter wraps): raise D7 and D8 to start servo pulses.
 * COMPA fires at OCR1A ticks: lower D7 to end u1 pulse.
 * COMPB fires at OCR1B ticks: lower D8 to end u2 pulse.
 ******************************************************************************/
volatile bool control_flag = false;

// Timer1 Overflow Interrupt
// Fires at the start of every 20 ms frame (counter wraps to 0 at BOTTOM)
// Raises D7 (u1) and D8 (u2) HIGH to begin both servo pulses
ISR(TIMER1_OVF_vect)
{
    PORTD |=  BIT(PORTD7);   // D7 HIGH: start u1 pulse (drive motor)
    PORTB |=  BIT(PORTB0);   // D8 HIGH: start u2 pulse (steering)
    control_flag = true;      // trigger 50 Hz control loop
}

// Timer1 Compare Match A Interrupt
// Fires when counter reaches OCR1A — ends the u1 (drive motor) pulse
ISR(TIMER1_COMPA_vect)
{
    PORTD &= ~BIT(PORTD7);   // D7 LOW: end u1 pulse
}

// Timer1 Compare Match B Interrupt
// Fires when counter reaches OCR1B — ends the u2 (steering) pulse
ISR(TIMER1_COMPB_vect)
{
    PORTB &= ~BIT(PORTB0);   // D8 LOW: end u2 pulse
}

/******************************************************************************
 *                    S I G N A L   H E L P E R S
 ******************************************************************************/

// ADC counts → angular velocity (rad/s)
// w = (V - 2.5) * 14.0,  where V = counts * ADC_TO_V
// 2.5 V = zero speed, 5.0 V = +wmax (35 rad/s), 0.0 V = -wmax
static inline float adc_to_omega(uint16_t counts)
{
    return (counts * ADC_TO_V - 2.5f) * 14.0f;
}

// Motor voltage Va [-12, +12] V → servo pulse width [1000, 2000] µs
static inline uint16_t va_to_pw(float Va)
{
    float pw = (Va / V_BAT) * PW_RANGE + PW_NEUTRAL;
    if (pw < PW_MIN) pw = PW_MIN;
    if (pw > PW_MAX) pw = PW_MAX;
    return (uint16_t)pw;
}

// Slip ratio S = (w_front - w_rear) / |w_front|
// S < 0: rear faster than front → wheelspin (traction control zone)
// S > 0: rear slower than front → wheel lock (braking control zone)
static inline float compute_slip(float w_front, float w_rear)
{
    if (fabsf(w_front) < 0.5f) return 0.0f;  // guard divide-by-zero at standstill
    return (w_front - w_rear) / fabsf(w_front);
}

/******************************************************************************
 *                      P I D   I N S T A N C E S
 ******************************************************************************/
static PID_t speed_pid;
static PID_t traction_pid;
static PID_t brake_pid;
static time_differencePID speed_time;
static time_differencePID traction_time;
static time_differencePID brake_time;

/******************************************************************************
 *                            S E T U P
 ******************************************************************************/
void setup()
{
    Serial.begin(1000000);   // 1 Mbaud for simulator 2 (change to 115200 for sim 1)

    timer2_init();           // Timer2 CTC: 1 ms millis_ clock
    init_servoPWM();         // Timer1 interrupt mode: servo pulses on D7 (u1) and D8 (u2)

    // Speed controller: track desired angular velocity (rad/s)
    PID_init(&speed_pid, 0.8f, 0.3f, 0.05f);
    speed_pid.integral_min = -25.0f;  // allow bidirectional integral

    // Traction controller: hold slip near TARGET_TRACTION_SLIP during acceleration
    PID_init(&traction_pid, 6.0f, 0.5f, 0.1f);
    traction_pid.integral_min = -25.0f;

    // Braking controller: hold slip near TARGET_BRAKE_SLIP during deceleration
    PID_init(&brake_pid, 6.0f, 0.5f, 0.1f);
    brake_pid.integral_min = -25.0f;

    init_dt(&speed_time);
    init_dt(&traction_time);
    init_dt(&brake_time);

    init_buffer();           // ADC interrupt buffers on A1 (y1), A3 (y2), A5 (y3); also calls sei()

    Serial.println(F("%time_s,y1_V,y2_V,y3_V,pw1_us,w_rear,S_right,S_left,S_ctrl,state"));

    uint8_t log_tick = 0;

    /**************************************************************************
     *                      M A I N   L O O P
     * Runs at 50 Hz (every 20 ms), triggered by Timer1 OVF ISR.
     **************************************************************************/
    while (1)
    {
        if (!control_flag) continue;
        control_flag = false;

        // ---- 1. Read sensors and convert to rad/s ----
        uint16_t raw1 = buffer1_avg();
        uint16_t raw3 = buffer3_avg();
        uint16_t raw5 = buffer5_avg();

        float w_rear = adc_to_omega(raw1);
        float w_fr   = adc_to_omega(raw3);   // right front wheel
        float w_fl   = adc_to_omega(raw5);   // left front wheel

        // Per-wheel slip: detect which individual wheel is losing traction
        float S_right = compute_slip(w_fr, w_rear);
        float S_left  = compute_slip(w_fl, w_rear);

        // Turn-aware control slip: use the faster (outer) front wheel as vehicle speed
        // reference. During cornering the inner wheel decelerates sharply — using the
        // average would give a falsely low reference speed and trigger traction control
        // on the wrong side. The outer wheel is always the better estimate of Vx.
        float w_ref     = (fabsf(w_fr) >= fabsf(w_fl)) ? w_fr : w_fl;
        float S_control = compute_slip(w_ref, w_rear);

        // ---- 2. Maneuver sequencer: set desired speed and steering ----
        uint32_t t = millis_();
        float desired_speed;
        uint16_t steer_us;
        drive_state_t state;

        if      (t < T_FORWARD_START)  { desired_speed =   0.0f; steer_us = 1500; state = STATE_NEUTRAL;    }
        else if (t < T_TURN_L_START)   { desired_speed =  20.0f; steer_us = 1500; state = STATE_FORWARD;    }
        else if (t < T_CRUISE1_START)  { desired_speed =  15.0f; steer_us = 1250; state = STATE_TURN_LEFT;  }
        else if (t < T_TURN_R_START)   { desired_speed =  20.0f; steer_us = 1500; state = STATE_CRUISE;     }
        else if (t < T_CRUISE2_START)  { desired_speed =  15.0f; steer_us = 1750; state = STATE_TURN_RIGHT; }
        else if (t < T_SLOW_START)     { desired_speed =  20.0f; steer_us = 1500; state = STATE_CRUISE;     }
        else if (t < T_BRAKE_START)    { desired_speed =   5.0f; steer_us = 1500; state = STATE_SLOW_DOWN;  }
        else if (t < T_REVERSE_START)  { desired_speed =   0.0f; steer_us = 1500; state = STATE_BRAKE;      }
        else if (t < T_STOP_START)     { desired_speed = -15.0f; steer_us = 1500; state = STATE_REVERSE;    }
        else if (t < T_DONUT_START)    { desired_speed =   0.0f; steer_us = 1500; state = STATE_STOP;       }
        else if (t < T_DONUT_END)      { desired_speed =   0.0f; steer_us = 2000; state = STATE_DONUT;      }
        else                           { desired_speed =   0.0f; steer_us = 1500; state = STATE_STOP;       }

        // Donut mode: traction control disabled, full throttle, full right steering lock
        bool donut_mode = (state == STATE_DONUT);

        // Steering applied directly — no PID needed
        set_u2_pulse(steer_us);

        // ---- 3. Safety priority stack ----
        // Tighten traction threshold under steering: lateral grip margin is lower in corners
        // so intervene sooner. steer_us 1500=straight, 1250/1750=full lock (±250 µs range).
        float steer_offset    = fabsf((float)steer_us - 1500.0f);          // 0–500
        float dynamic_threshold = TRACTION_SLIP_THRESHOLD
                                + (steer_offset / 500.0f) * 0.10f;
        // straight: -0.20  |  quarter lock: -0.15  |  full lock: -0.10

        // Braking: commanded to stop (or reverse) but car is still rolling forward.
        // desired_speed <= 0 covers both STATE_BRAKE and STATE_REVERSE — the brake
        // controller fires until w_rear drops below 1 rad/s, then speed control
        // takes over and drives the motor to the reverse setpoint.
        bool braking = (desired_speed <= 0.0f && w_rear > 1.0f);
        float Va;

        if (donut_mode) {
            // Donut: bypass all controllers — full throttle, traction control disabled.
            // Steering is already at full right lock (steer_us=1750) via the sequencer.
            Va = V_BAT;
            // Keep all three time structs current so PIDs resume cleanly after donut ends
            update_dt(&speed_time);
            update_dt(&traction_time);
            update_dt(&brake_time);
        } else if (braking) {
            // Hold positive slip to maximise deceleration force
            Va = PID_compute(&brake_pid, TARGET_BRAKE_SLIP, S_control, &brake_time);
            if (Va > 0.0f) Va = 0.0f;   // braking only: Va ≤ 0 (neutral or reverse)
            update_dt(&speed_time);
            update_dt(&traction_time);
        } else if (S_control < dynamic_threshold) {
            // Wheelspin detected (turn-aware, outer wheel reference): reduce drive torque
            Va = PID_compute(&traction_pid, TARGET_TRACTION_SLIP, S_control, &traction_time);
            if (Va < 0.0f) Va = 0.0f;   // no reverse during traction control
            update_dt(&speed_time);
            update_dt(&brake_time);
        } else {
            // Normal: speed controller tracks desired_speed from sequencer
            Va = PID_compute(&speed_pid, desired_speed, w_rear, &speed_time);
            update_dt(&traction_time);
            update_dt(&brake_time);
        }

        // ---- 4. Saturate Va and output to drive servo ----
        if (Va >  V_BAT) Va =  V_BAT;
        if (Va < -V_BAT) Va = -V_BAT;
        uint16_t pw1 = va_to_pw(Va);
        set_u1_pulse(pw1);

        // ---- 5. Serial CSV log every 20 ticks (~400 ms) ----
        if (++log_tick >= 20) {
            log_tick = 0;
            Serial.print(t * 0.001f, 3);       Serial.print(',');
            Serial.print(raw1 * ADC_TO_V, 3);  Serial.print(',');
            Serial.print(raw3 * ADC_TO_V, 3);  Serial.print(',');
            Serial.print(raw5 * ADC_TO_V, 3);  Serial.print(',');
            Serial.print(pw1);                 Serial.print(',');
            Serial.print(w_rear, 2);           Serial.print(',');
            Serial.print(S_right, 3);          Serial.print(',');
            Serial.print(S_left, 3);           Serial.print(',');
            Serial.print(S_control, 3);        Serial.print(',');
            Serial.println(state);
        }
    }
}

// Arduino framework requires loop() to be defined; all logic lives in setup().
void loop() {}
