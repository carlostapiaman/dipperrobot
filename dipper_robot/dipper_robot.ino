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

// define a variable for the reference position point
long initial_homing=-1;  // Used to Home Stepper at startup

// Define the stepper motor and the pins it will use, and its setup
AccelStepper stepper( AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
double mode = 1; //the stepping mode {1, 2, 4, 8, 16, 32}
#define MOTOR_FULL_STEPS 200
#define MOTOR_STEPS MOTOR_FULL_STEPS*mode // Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step

// Define some communication variables
int incomingByte;      // a variable to read incoming serial data into

// Define variables for Kinematik/Dynamics
const double Ulin = 25;          //linear transmition power screw IGUS [mm/turn]
const int U = 1;             //Gear ratio: input/output
//const double Utri = 1;     //correction for trinema silent driver ON/OFF/OFF (check if necessary)
const double H_max = 425;     //maximum height in mm(upper physical limit)
const double H_min = 260;      //minimum height in mm(lower physical limit)
double H;                     //A variable to store the current height in mm
double kres = Ulin/(MOTOR_FULL_STEPS*mode*U); //[mm/steps] steps to mm. (Resolution)

//defined default motion parameters
int SPD_lin_def = 30; //linear speed in mm/s
double ACC_time_def = 500; //[ms] the time it takes to accelerate to max speed
int startDelay_def = 200; //[ms] the time delay before an operation begins
int dipTime_def = 500; //[ms] the time it stays on the targeted position (Dipping)
int homingSPD = MOTOR_STEPS*1; //Homing speed in steps per second

//used motion parameters. These variables are actively used.
long SPD_lin; //linear speed in mm/s
double ACC_time;  //[ms] the time it takes to accelerate to the defined speed
int startDelay;   //[ms] 
int dipTime;      //[ms] 

//Unit conversion variables (between mm and steps)
int SPD_steps;// = SPD_lin_def/kres; // Speed in steps per second
int ACC_steps;// = SPD_lin_def/(kres*ACC_time_def*0.001); // Acceleration in steps per second per second

// target position in the operation
double targetPos;

//motion settings {SPD_lin [mm/s], ACC_time [ms], startDelay [ms], dipTime [ms]} Set program settings here
int motionED[] = {30, 1000, 500, 10}; //ASCII 69
int motionCB[] = {30, 1000, 500, 1000}; //ASCII 67
int motionLB[] = {30, 1000, 500, 1000}; //ASCII 76
int motionFKS[] = {100, 250, 500, 10}; //ASCII 75
int motionAD[] = {50, 250, 500, 5000}; //ASCII 65
int motionBT[5];                        //ASCII 83 This is the "free" program where the motion paramters are received externally (not implemented yet)


// Defined some useful functions

void serialFlush(){
  // cleans up the serial buffer
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  

void goHome()
{
  //moves stepper to home position (the upper maximum limit) set by a switch
  delay(5);  // Wait for EasyDriver wake up

   //  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepper.setMaxSpeed(homingSPD);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(ACC_steps);  // Set Acceleration of Stepper

  // Start Homing procedure of Stepper Motor at startup

  //Serial.print("Stepper is Homing . . . . . . . . . . . ");
  stepper.disableOutputs(); //motors ON 
  while (digitalRead(homeSwitch)) 
  {  // Make the Stepper move CCW until the switch is activated
    //stepper.disableOutputs();   //Turns Motor ON
    stepper.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    stepper.run();  // Start moving the stepper
    delay(1);
  }
  
  stepper.setCurrentPosition(0);  // Set the current position as zero for now
  stepper.setMaxSpeed(homingSPD);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(homingSPD);  // Set Acceleration of Stepper
  initial_homing=1;
  delay(50);

  while (!digitalRead(homeSwitch)) { // Make the Stepper move CW until the switch is deactivated
    stepper.moveTo(initial_homing);  
    stepper.run();
    initial_homing++;
    delay(1);
  }
  stepper.enableOutputs(); //Motor is off
  stepper.setCurrentPosition(0);
  //Serial.println("Homing Completed");
  //Serial.println("");
  stepper.setMaxSpeed(SPD_steps);      // Set Max Speed of Stepper (Faster for regular movements)
  stepper.setAcceleration(ACC_steps);  // Set Acceleration of Stepper

}

double getResolution()
{
  //returns the linear motion output resolution in mm/step
  double res;
  res=kres;
  return res;
  
}

double stepsToHeight(long steps)
{
  //converts input steps to its respective height in mm.
  //This considering the phyisical model. Zero steps equals the maximum height.
  double H_out;
  H_out = H_max-kres*steps;
  return H_out;
}

long heightToSteps(double H_in)
{
  //converts input height to its respective absolute number of steps. (some accuracy is lost here)
  //This considering the phyisical model. Zero steps equals the maximum height.
  double steps_out;
  steps_out = (H_max-H_in)*(1/kres);
  return steps_out;
}


long heightToStepsrel(double H_in)
{
  //converts distance input in mm to its steps equivalent. (some accuracy is lost here)
  //This is regardless of the physical model. Only the motor output ratio is considered.
  double steps_out;
  steps_out = (H_in)*(1/kres);
  return steps_out;
}

double getCurrentPos()
{
  //gets current position in mm
  double H_ist = stepsToHeight(stepper.currentPosition());
  return H_ist;
}


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
