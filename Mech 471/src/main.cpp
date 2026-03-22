 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <Arduino.h>
#include <HW\digital.h>


void setup() {
Serial.begin(9600);
digital_i_o(pin7,output);
digital_i_o(pin8,output);
}

void loop() {
set_digitalHIGH(pin7);
set_digitalHIGH(pin8);
delay(10);
set_digitalLOW(pin7);
set_digitalLOW(pin8);
delay(10);
}