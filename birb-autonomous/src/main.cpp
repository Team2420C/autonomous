//----------------------------------------------------------------------------
//                                                                            
//    Module:       main.cpp                                                  
//    Author:       BIRB                                                 
//    Created:      12/9/2023                                                    
//    Description:  IQ project                                                
//                                                                            
//----------------------------------------------------------------------------

#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor leftdriveMotorA = motor(PORT1, true);
motor leftdriveMotorB = motor(PORT2, false);
motor_group leftdrive = motor_group(leftdriveMotorA, leftdriveMotorB);

motor rightdriveMotorA = motor(PORT5, true);
motor rightdriveMotorB = motor(PORT6, false);
motor_group rightdrive = motor_group(rightdriveMotorA, rightdriveMotorB);

controller Controller = controller();

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// Allows for easier use of the VEX Library
// Allows for easier use of the VEX Library
using namespace vex;
bool isAutonomousRunning = false;
float x_pos;
float y_pos;
float error_x;
float error_y;
float target_x;
float target_y;
float gear_ratio = 3;
float target_angle;
float pi = 3.14159265358979323846;
float theta;
float kp = 0.01;
float angle_difference;

void updatePosition() {
  while (true) {
    if (!isAutonomousRunning) {
      theta = BrainInertial.heading(degrees) * pi / 180;
      y_pos = (leftdrive.position(degrees) + rightdrive.position(degrees)) / 2 * gear_ratio;
      x_pos = (leftdrive.position(degrees) + rightdrive.position(degrees)) / 2 * cos(theta) * gear_ratio;
      error_x = target_x - x_pos;
      error_y = target_y - y_pos;
      wait(20, msec);
    } else {
      // Stop updating position while autonomous is running
      wait(20, msec);
    }
  }
}

void autonomous(float target_x, float target_y) {
  isAutonomousRunning = true;
  while (fabs(x_pos - target_x) > 1 || fabs(y_pos - target_y) > 1) {
    theta = BrainInertial.heading(degrees) * pi / 180;
    y_pos = (leftdrive.position(degrees) + rightdrive.position(degrees)) / 2 * gear_ratio;
    x_pos = (leftdrive.position(degrees) + rightdrive.position(degrees)) / 2 * cos(theta) * gear_ratio;
    error_x = target_x - x_pos;
    error_y = target_y - y_pos;
    // Calculate the angle to the target
    target_angle = atan2(error_y, error_x) * 180 / pi;
    // Calculate the difference between the target angle and the current heading
    angle_difference = target_angle - theta;
    leftdrive.setVelocity(angle_difference, percent);
    rightdrive.setVelocity(angle_difference, percent);
    leftdrive.spin(forward);
    rightdrive.spin(forward);
    printf("x_pos: %f, y_pos: %f", x_pos, y_pos);
    printf("\n");
    printf("target_x: %f, target_y: %f", target_x, target_y);
    printf("\n");
    wait(20, msec); // Adjust the wait time as needed
  }
  isAutonomousRunning = false;
  leftdrive.stop();
  rightdrive.stop();
}
int main() {
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrating");
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) { task::sleep(50); }
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrated");
  Brain.playSound(siren);
  leftdrive.setStopping(coast);
  rightdrive.setStopping(coast);
  BrainInertial.setHeading(0, degrees);
  while (!Controller.ButtonEUp.pressing()) {
    wait(5, msec);
  }
  vex::thread positionUpdater(updatePosition);
  autonomous(0, 1000);
  Brain.playSound(siren);
}