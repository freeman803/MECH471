#include <Arduino.h>

#define SIGN(a) ( (a) >= 0.0 ? 1.0 : -1.0 )

void sim_step(float &t, float x[], float u[], float dt);
void interrupt1();
void interrupt2();
void task1();

const int N = 3;
const int M = 2;

volatile int pw1 = 1500, pw2 = 1500;

const int U1_PIN = 2, U2_PIN = 3;
const int Y1_PIN = 5, Y2_PIN = 6, Y3_PIN = 11;

float t0;

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

    t0 = micros() * 1.0e-6;

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
    const float V_bat  = 12.0;
    const float PW_R   = 1.0 / (PW_MAX - PW_0);
    const float wmax   = 35.0;
    const float wmax_inv = 1.0 / wmax;

    float tc;
    float w;
    static float t;
    static float dt;
    static float x[N + 1];
    static float u[M + 1];
    static int   init = 0;

    dt = 0.001;

    if (!init) {
        sim_step(t, x, u, dt);
        init = 1;
    }

    w1 = pw1;
    if (w1 > PW_MAX) w1 = PW_MAX;
    if (w1 < PW_MIN) w1 = PW_MIN;

    u[1] = (w1 - PW_0) * PW_R * V_bat;  // motor voltage (V)
    u[2] = 0.0;                           // disturbance torque (Nm)

    tc = micros() * 1.0e-6 - t0;

    while (t < tc) {
        sim_step(t, x, u, dt);
    }

    w  = x[2];
    y1 = 2.5 + w * wmax_inv * 2.5;
    if (y1 < 0.0) y1 = 0.0;
    if (y1 > 5.0) y1 = 5.0;

    // y2 and y3 not yet modelled — fixed voltages
    y2 = 2.0;
    y3 = 3.0;

    output = (int)(y1 * 51); analogWrite(Y1_PIN, output);
    output = (int)(y2 * 51); analogWrite(Y2_PIN, output);
    output = (int)(y3 * 51); analogWrite(Y3_PIN, output);

    Serial.print(t);   Serial.print(",");
    Serial.print(w);   Serial.print(",");
    Serial.print(u[1]); Serial.print("\n");

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

    if (!init) {
        L  = 0.03;
        R  = 0.3;
        kb = km = 0.3;
        J  = 0.2;
        b  = 0.1;
        fc = 0.0;

        t    = 0.0;
        x[1] = 0.0;
        x[2] = 0.0;
        x[3] = 0.0;

        init = 1;
        return;
    }

    xd[1] = (-x[1] * R  - kb * x[2] + u[1]) / L;           // di/dt
    xd[2] = (km * x[1]  - b  * x[2] - fc * SIGN(x[2]) - u[2]) / J; // dw/dt
    xd[3] = x[2];                                            // dtheta/dt

    for (int i = 1; i <= N; i++) x[i] += xd[i] * dt;
    t += dt;
}

void loop() {}
