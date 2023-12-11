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
float DeltaTheta;
float currentL;
float currentR;
float DeltaL;
float DeltaR;
float SideChord;
float DeltaXSide;
float DeltaYSide;
float deltaDist;
float angle_difference;
float target_distance;
float PreviousR;
float PreviousL;
const float wheel_circumference = 2.3622 * pi;
const float wheelbase = 4.5;
const float ticksPerRev = 360;
const float DEG_TO_RAD = 2 * pi / ticksPerRev * 1 / gear_ratio;

void updatePosition() {
  while (true) {
    if (!isAutonomousRunning) {
      // Averages the Left and Right integrated motor encoders since we don't have encoders yet
      currentR = (rightdriveMotorA.position(degrees) + rightdriveMotorB.position(degrees)) / 2.0;
      currentL = (leftdriveMotorA.position(degrees) + leftdriveMotorB.position(degrees)) / 2.0;

      // Creates variables for change in each side info in inches (replace 12.9590697 with your wheel circumference)
      DeltaL = ((currentL - PreviousL) * wheel_circumference) / ticksPerRev;
      DeltaR = ((currentR - PreviousR) * wheel_circumference) / ticksPerRev;

      // Determines the change in angle of the robot using the rotational change in each side
      DeltaTheta = (DeltaR - DeltaL) / wheelbase;

      // Creates an if/else statement to prevent NaN values from appearing and causing issues with calculation
      if (DeltaTheta == 0) {  // If there is no change in angle
        x_pos += DeltaL * cos(theta);
        y_pos += DeltaL * sin(theta);
      } else {  // If the angle changes
        // Calculate the changes in X, Y from chords of an arc/circle
        SideChord = 2 * ((DeltaL / DeltaTheta) + wheelbase) * sin(DeltaTheta / 2);
        DeltaYSide = SideChord * cos(theta + (DeltaTheta / 2));
        DeltaXSide = SideChord * sin(theta + (DeltaTheta / 2));

        // Update the robot's position and orientation
        theta += DeltaTheta;
        x_pos += DeltaXSide;
        y_pos += DeltaYSide;
      }

      // Update the previous values for the next iteration
      PreviousL = currentL;
      PreviousR = currentR;

      wait(5, msec);
    } else {
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
  while ((fabs(x_pos - target_x) > 1 || fabs(y_pos - target_y) > 1) &&
         (Controller.AxisA.position() == 0) && (Controller.AxisB.position() == 0) &&
         (Controller.AxisC.position() == 0) && (Controller.AxisD.position() == 0)) {

    // Update odometry position
    currentR = (rightdriveMotorA.position(degrees) + rightdriveMotorB.position(degrees)) / 2.0;
    currentL = (leftdriveMotorA.position(degrees) + leftdriveMotorB.position(degrees)) / 2.0;
    DeltaL = ((currentL - PreviousL) * wheel_circumference) / ticksPerRev;
    DeltaR = ((currentR - PreviousR) * wheel_circumference) / ticksPerRev;
    DeltaTheta = (DeltaR - DeltaL) / wheelbase;

    if (DeltaTheta == 0) {  // If there is no change in angle
      x_pos += DeltaL * cos(theta);
      y_pos += DeltaL * sin(theta);
    } else {  // If the angle changes
      SideChord = 2 * ((DeltaL / DeltaTheta) + wheelbase) * sin(DeltaTheta / 2);
      DeltaYSide = SideChord * cos(theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin(theta + (DeltaTheta / 2));

      theta += DeltaTheta;
      x_pos += DeltaXSide;
      y_pos += DeltaYSide;
    }

    // Update the previous values for the next iteration
    PreviousL = currentL;
    PreviousR = currentR;

    // Calculate errors for control
    error_x = target_x - x_pos;
    error_y = target_y - y_pos;

    // Calculate the angle to the target
    target_distance = sqrt(error_x * error_x + error_y * error_y);
    target_angle = atan2(error_y, error_x);

    // Calculate the difference between the target angle and the current heading
    angle_difference = target_angle - theta;

    // Set the velocities to move towards the target
    leftdrive.setVelocity(angle_difference * 50, percent);
    rightdrive.setVelocity(angle_difference * 50, percent);

    // Spin the motors
    leftdrive.spin(forward);
    rightdrive.spin(forward);

    // Print debugging information (optional)
    printf("x_pos: %f, y_pos: %f", x_pos, y_pos);
    printf("\n");
    printf("target_x: %f, target_y: %f", target_x, target_y);
    printf("\n");

    // Adjust the wait time as needed
    wait(5, msec);
  }

  // Stop the motors when the target is reached
  leftdrive.stop();
  rightdrive.stop();

  isAutonomousRunning = false;
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
  autonomous(0, 1000);
  Brain.playSound(siren);
}
