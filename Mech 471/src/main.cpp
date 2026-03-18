 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/
// allow c++ to recognize c files must include all functions in this so c++ compiler can recognize them
// #ifdef __cplusplus
// extern "C" {
// #endif
// void HW_init(void);
// void timer0_init(void);
// #ifdef __cplusplus
// }
// #endif
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <Arduino.h>
#include <HW.h>
#include <PID.h>

void setup() {
  HW_init();
  timer0_init();
}

void loop() {

}