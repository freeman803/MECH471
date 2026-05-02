#include <Arduino.h>

// Extended HIL simulator: adds vehicle body + front wheel model to sim2.
// This makes slip ratio meaningful so traction and braking controllers work.
//
// Added physics vs sim2:
//   v_car  - vehicle longitudinal velocity (m/s), 4th state
//   w_front - front wheel speed = v_car / r_wheel (pure rolling, no front slip)
//   F_tire  - rear tyre force: Pacejka-style saturation on slip ratio
//   T_load  - tyre reaction torque fed back into the motor equation
//
// Front wheel voltages (y2, y3) now encode w_front dynamically.

#define SIGN(a) ( (a) >= 0.0 ? 1.0 : -1.0 )

void sim_step(float &t, float x[], float u[], float dt);
void interrupt1();
void interrupt2();
void task1();

const int N = 3;   // motor states: i, w, theta
const int M = 2;   // inputs: Va, Td

volatile int pw1 = 1500, pw2 = 1500;

const int U1_PIN = 2, U2_PIN = 3;
const int Y1_PIN = 5, Y2_PIN = 6, Y3_PIN = 11;

float t0;

// Vehicle state (outside sim_step so task1 can read it)
static float v_car   = 0.0f;   // vehicle speed (m/s)
static float w_front = 0.0f;   // front wheel speed (rad/s)

void setup()
{
    Serial.begin(1000000);

    pinMode(Y1_PIN, OUTPUT);
    pinMode(Y2_PIN, OUTPUT);
    pinMode(Y3_PIN, OUTPUT);

    pinMode(U1_PIN, INPUT_PULLUP);
    pinMode(U2_PIN, INPUT_PULLUP);

    noInterrupts();
    attachInterrupt(digitalPinToInterrupt(2), interrupt1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), interrupt2, CHANGE);
    interrupts();

    t0 = micros() * 1.0e-6f;

    while (1) {
        task1();
    }

    delay(1000);
    exit(0);
}

void task1()
{
    int output;
    float y1, y2, y3;
    int w1;

    const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
    const float V_bat     = 12.0f;
    const float PW_R      = 1.0f / (PW_MAX - PW_0);
    const float wmax      = 35.0f;
    const float wmax_inv  = 1.0f / wmax;

    float tc;
    float w_rear;
    static float t;
    static float dt;
    static float x[N + 1];
    static float u[M + 1];
    static int   init = 0;

    dt = 0.001f;

    if (!init) {
        sim_step(t, x, u, dt);
        init = 1;
    }

    w1 = pw1;
    if (w1 > PW_MAX) w1 = PW_MAX;
    if (w1 < PW_MIN) w1 = PW_MIN;

    u[1] = (w1 - PW_0) * PW_R * V_bat;   // motor voltage (V)
    u[2] = 0.0f;

    tc = micros() * 1.0e-6f - t0;
    while (t < tc) {
        sim_step(t, x, u, dt);
    }

    w_rear = x[2];

    // --- rear wheel output (y1) ---
    y1 = 2.5f + w_rear * wmax_inv * 2.5f;
    if (y1 < 0.0f) y1 = 0.0f;
    if (y1 > 5.0f) y1 = 5.0f;

    // --- front wheel output (y2, y3) — both equal, symmetric model ---
    y2 = 2.5f + w_front * wmax_inv * 2.5f;
    if (y2 < 0.0f) y2 = 0.0f;
    if (y2 > 5.0f) y2 = 5.0f;
    y3 = y2;

    output = (int)(y1 * 51); analogWrite(Y1_PIN, output);
    output = (int)(y2 * 51); analogWrite(Y2_PIN, output);
    output = (int)(y3 * 51); analogWrite(Y3_PIN, output);

    Serial.print(t);          Serial.print(",");
    Serial.print(w_rear, 3);  Serial.print(",");
    Serial.print(w_front, 3); Serial.print(",");
    Serial.print(u[1], 3);    Serial.print("\n");

    delay(10);
}

void interrupt1()
{
    int input;
    static int input_p = 0;
    static unsigned long int t0 = 0;

    input = digitalRead(U1_PIN);
    if ((input == HIGH) && (input_p == LOW)) t0 = micros();
    if ((input == LOW)  && (input_p == HIGH)) pw1 = micros() - t0;
    input_p = input;
}

void interrupt2()
{
    int input;
    static int input_p = 0;
    static unsigned long int t0 = 0;

    input = digitalRead(U2_PIN);
    if ((input == HIGH) && (input_p == LOW)) t0 = micros();
    if ((input == LOW)  && (input_p == HIGH)) pw2 = micros() - t0;
    input_p = input;
}

void sim_step(float &t, float x[], float u[], float dt)
{
    static float xd[N + 1];
    static float L, R, kb, J, km, b, fc;
    static int   init = 0;

    // Vehicle + tyre parameters
    const float M_car    = 1.0f;    // car mass (kg)
    const float r_wheel  = 0.05f;   // wheel radius (m)
    const float b_car    = 0.5f;    // car body drag (N·s/m)
    const float F_max    = 9.81f;   // peak tyre force = M*g*mu (N), mu≈1
    const float S_opt    = 0.20f;   // slip at peak traction (matches TRACTION_SLIP_THRESHOLD)

    if (!init) {
        L  = 0.03f;
        R  = 0.3f;
        kb = km = 0.3f;
        J  = 0.2f;
        b  = 0.1f;
        fc = 0.0f;

        t    = 0.0f;
        x[1] = 0.0f;   // armature current (A)
        x[2] = 0.0f;   // motor angular velocity (rad/s)
        x[3] = 0.0f;   // shaft angle (rad)

        v_car   = 0.0f;
        w_front = 0.0f;

        init = 1;
        return;
    }

    // ---- Tyre slip ratio (sign convention matches controller) ----
    // S = (w_front - w_rear) / |w_front|
    // S < 0: rear spins faster → wheelspin
    // S > 0: rear slower      → wheel lock
    float slip = 0.0f;
    if (fabsf(w_front) > 0.1f) {
        slip = (w_front - x[2]) / fabsf(w_front);
    }

    // ---- Tyre force (Pacejka-style saturation) ----
    // Peaks at S = ±S_opt, saturates beyond that
    // Negate: with S=(w_front-w_rear)/|w_front|, wheelspin gives S<0.
    // A negative S must produce positive forward force (accelerating the car),
    // so the Pacejka sign must be flipped.
    float S_norm  = slip / S_opt;
    float F_tire  = -F_max * S_norm / sqrtf(1.0f + S_norm * S_norm);

    // ---- Vehicle body dynamics ----
    float v_car_dot = (F_tire - b_car * v_car) / M_car;
    v_car  += v_car_dot * dt;
    w_front = v_car / r_wheel;   // front wheel: pure rolling

    // ---- Motor state equations (tyre reaction feeds back as load torque) ----
    float T_load = F_tire * r_wheel;   // road reaction on rear wheel (Nm)

    xd[1] = (-x[1] * R  - kb * x[2] + u[1]) / L;
    xd[2] = (km * x[1]  - b  * x[2] - fc * SIGN(x[2]) - T_load) / J;
    xd[3] = x[2];

    for (int i = 1; i <= N; i++) x[i] += xd[i] * dt;
    t += dt;
}

void loop() {}
