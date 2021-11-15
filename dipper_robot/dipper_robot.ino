/**************************************************************************/
/* Linear motion within two boundaries                                    */
/* Arduino source code for stepper motor control                          */ 
/*                                                                        */
/*                                                                        */
/* Hardware                                                               */
/* Board: Arduino Uno                                                     */
/* Periphery: HC05 Bluetooth                                              */
/* Stepper Driver: DM320T                                                 */
/* Arduino IDE Version 1.8.12                                             */
/* Library used: "AccelStepper" by Mike McCauley V 1.59.0                 */
/*                                                                        */
/* Version: 1.0                                                           */
/* Stand: 15.11.2021                                                      */
/* Author: Carlos Tapia Mancera                                           */
/**************************************************************************/

#include <Arduino.h>
#include <AccelStepper.h>
//https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#a176c5d2e4c2f21e9e92b12e39a6f0e67

//Arduino Pins are defined here
#define DIR_PIN     7 // Stepper direction signal
#define STEP_PIN    6  // Stepper steps signal
#define ENABLE_PIN  8 // Stepper's Enable/disable signal (power on or off)
#define LED 9         // Operational LED 
#define homeSwitch 13  // Reference Position signal (Before it was 3 but pin broke)


// Define the stepper motor and the pins it will use
AccelStepper stepper( AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}