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

void setup() {
Serial.begin(9600);
init_buffer();
}

void loop() {
uint16_t value = buffer1_avg();
value = (float)value;
Serial.println("A1");
Serial.println(value*5.0f/1023.0f,4);
delay(100);
uint16_t value2 = buffer3_avg();
value2 = (float)value2;
Serial.println("A3");
Serial.println(value2*5.0f/1023.0f,4);
delay(100);
}
