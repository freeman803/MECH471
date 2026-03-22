 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <Arduino.h>
#include <HW\ADC.h>

float adc1 = 0;
float adc3 = 0;
float adc5 = 0;

void setup() {
Serial.begin(9600);
ADC_INIT();
analog_i_o(A_1,input);
analog_i_o(A_3,input);
analog_i_o(A_5,input);

}

void loop() {
adc1 = read_analog_ADC(A_1);
adc3 = read_analog_ADC(A_3);
adc5 = read_analog_ADC(A_5);
Serial.print("\n adc1 ,");
Serial.print(adc1);
Serial.print(", adc3 ,");
Serial.print(adc3);
Serial.print(", adc5 ,");
Serial.print(adc5);
Serial.print(",");


delay(10);
}