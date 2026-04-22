#include <Arduino.h>

void interrupt1();
void interrupt2();
void task1();

volatile int pw1, pw2;

const int U1_PIN = 2, U2_PIN = 3;
const int Y1_PIN = 5, Y2_PIN = 6, Y3_PIN = 11;

void setup()
{
    Serial.begin(115200);
    Serial.print("\n%starting program");

    pinMode(Y1_PIN, OUTPUT);
    pinMode(Y2_PIN, OUTPUT);
    pinMode(Y3_PIN, OUTPUT);

    pinMode(U1_PIN, INPUT_PULLUP);
    pinMode(U2_PIN, INPUT_PULLUP);

    noInterrupts();
    attachInterrupt(digitalPinToInterrupt(2), interrupt1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), interrupt2, CHANGE);
    interrupts();

    while (1) {
        task1();
    }

    delay(1000);
    exit(0);
}

void task1()
{
    int output;

    // Fixed test voltages — verify controller reads y1=1V, y2=2V, y3=3V
    output = (int)(1.0 * 51); analogWrite(Y1_PIN, output);
    output = (int)(2.0 * 51); analogWrite(Y2_PIN, output);
    output = (int)(3.0 * 51); analogWrite(Y3_PIN, output);

    Serial.print("\n");
    Serial.print(pw1);
    Serial.print(",");
    Serial.print(pw2);

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

void loop() {}
