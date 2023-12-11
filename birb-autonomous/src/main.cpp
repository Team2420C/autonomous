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
const float gear_ratio = 3;
float target_angle;
float pi = 3.14159265358979323846;
float theta;
float leftWheelDist;
float rightWheelDist;
float deltaTheta;
float deltaDist;
float angle_difference;
float target_distance;
const float wheel_diameter = 7.87402;
const float wheelbase = 4.5;
const float ticksPerRev = 360;
const float DEG_TO_RAD = 2 * pi / ticksPerRev * 1 / gear_ratio;

void updatePosition() {
  while (true) {
    if (!isAutonomousRunning) {
      leftWheelDist = DEG_TO_RAD * leftdrive.position(degrees) * wheel_diameter;
      rightWheelDist = DEG_TO_RAD * rightdrive.position(degrees) * wheel_diameter;
      deltaDist = (leftWheelDist + rightWheelDist) / 2.0;
      deltaTheta = (rightWheelDist - leftWheelDist) / wheelbase;
      x_pos += deltaDist * cos(theta);
      y_pos += deltaDist * sin(theta);
      theta += deltaTheta;
      // Normalize theta to keep it within [0, 2*pi)
      theta = fmod(theta, 2.0 * pi);
      if (theta < 0) {
        theta += 2.0 * pi;
      }
      wait(5, msec);
    } 
    else {
      // Stop updating position while autonomous is running
      wait(5, msec);
    }
  }
}
void driver(){
  while (true) {
    while ((Controller.AxisA.position() != 0) || (Controller.AxisB.position() != 0) || (Controller.AxisC.position() != 0) || (Controller.AxisD.position() != 0)) {
      leftdrive.setVelocity(Controller.AxisA.position(), percent);
      rightdrive.setVelocity(Controller.AxisD.position(), percent);
      leftdrive.spin(forward);
      rightdrive.spin(forward);
      wait(1, msec);
    } 
    wait(5, msec);
    // leftdrive.stop();
    // rightdrive.stop();
  }
}

void autonomous(float target_x, float target_y) {
  isAutonomousRunning = true;
  while ((fabs(x_pos - target_x) > 1 || fabs(y_pos - target_y) > 1) && (Controller.AxisA.position() == 0) && (Controller.AxisB.position() == 0) && (Controller.AxisC.position() == 0) && (Controller.AxisD.position() == 0)) {
    leftWheelDist = DEG_TO_RAD * leftdrive.position(degrees) * wheel_diameter;
    rightWheelDist = DEG_TO_RAD * rightdrive.position(degrees) * wheel_diameter;
    deltaDist = (leftWheelDist + rightWheelDist) / 2.0;
    deltaTheta = (rightWheelDist - leftWheelDist) / wheelbase;
    x_pos += deltaDist * cos(theta);
    y_pos += deltaDist * sin(theta);
    theta += deltaTheta;
    // Normalize theta to keep it within [0, 2*pi)
    theta = fmod(theta, 2.0 * pi);
    if (theta < 0) {
      theta += 2.0 * pi;
    }
    error_x = target_x - x_pos;
    error_y = target_y - y_pos;
    // Calculate the angle to the target
    target_distance = sqrt(error_x * error_x + error_y * error_y);
    target_angle = atan2(error_y, error_x);
    // Calculate the difference between the target angle and the current heading
    angle_difference = target_angle - theta;
    leftdrive.setVelocity((20 + angle_difference) * cos(angle_difference), percent);
    rightdrive.setVelocity((20 - angle_difference) * cos(angle_difference), percent);
    leftdrive.spin(forward);
    rightdrive.spin(forward);
    printf("x_pos: %f, y_pos: %f", x_pos, y_pos);
    printf("\n");
    printf("target_x: %f, target_y: %f", target_x, target_y);
    printf("\n");
    wait(5, msec); // Adjust the wait time as needed
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
  vex::thread drivercontroll(driver);
  autonomous(200, 200);
  Brain.playSound(siren);
}