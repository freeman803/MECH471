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
uint16_t value = buffer_avg(USE_Buffer1.buffer);
Serial.print(value);

}