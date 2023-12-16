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

bool isAutonomousRunning = false;
bool robotReverse = false;
float x_pos;
float y_pos;
const float pi = 3.14159265358979323846;
const float wheel_circumference = 2.3622 * pi;
const float wheel_distance = 4.5;
const float gear_ratio = 3;
const float kp_x = 0.1;
const float ki_x = 0;
const float kd_x = 0;
const float kp_y = 0.1;
const float ki_y = 0;
const float kd_y = 0;
const float kp_heading = 0.1;
const float ki_heading = 0;
const float kd_heading = 0;
float proportional_x;
float integral_x;
float derivative_x;
float proportional_y;
float integral_y;
float derivative_y;
float proportional_heading;
float integral_heading;
float derivative_heading;
float pid_output_x;
float pid_output_y;
float pid_output_heading;
float left_velocity;
float right_velocity;
float theta;
float prev_theta;
float current_theta;
float leftwheeldist;
float rightwheeldist;
float prev_leftwheeldist;
float prev_rightwheeldist;
float change_leftwheel;
float change_rightwheel;
float target_x;
float target_y;
float error_x;
float error_y;
float prev_error_x;
float prev_error_y;
float prev_error_heading;
float dist_moved;
float change_in_x;
float change_in_y;
float angle_difference;
float target_angle;
float target_angle_odom;
float target_distance;
float heading_error;

void updatePosition() {
  while (true) {
    if (!isAutonomousRunning) {
      prev_theta = theta;
      prev_leftwheeldist = leftwheeldist;
      prev_rightwheeldist = rightwheeldist;
      leftwheeldist = (leftdriveMotorA.position(degrees) + leftdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;
      rightwheeldist = (rightdriveMotorA.position(degrees) + rightdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;      change_rightwheel = rightwheeldist - prev_rightwheeldist;
      change_rightwheel = rightwheeldist - prev_rightwheeldist;
      change_leftwheel = leftwheeldist - prev_leftwheeldist;
      dist_moved = (change_leftwheel + change_rightwheel) / 2;
      current_theta = (change_leftwheel - change_rightwheel) / wheel_distance;
      theta = prev_theta + current_theta;
      if (theta > 180) {
        theta = 0;
        robotReverse = true;
      }
      if (robotReverse == false) {
        change_in_x = dist_moved * sin(theta);
        change_in_y = dist_moved * cos(theta);
      } 
      else {
        change_in_x = dist_moved * cos(theta);
        change_in_y = dist_moved * sin(theta);
      }
      x_pos = x_pos + change_in_x;
      y_pos = y_pos + change_in_y; 
      Brain.Screen.print("x_pos: %f", x_pos);
      Brain.Screen.newLine();
      Brain.Screen.print("y_pos: %f", y_pos);
      Brain.Screen.newLine();
      Brain.Screen.setCursor(1, 1);
      // Adjust the wait time as needed
      wait(5, msec);
      Brain.Screen.clearScreen();   
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

void autonomous(float target_x, float target_y, float target_angle) {
  isAutonomousRunning = true;
  while ((fabs(x_pos - target_x) > .1 || fabs(y_pos - target_y) > .1) && heading_error != 0 && (Controller.AxisA.position() == 0) && (Controller.AxisB.position() == 0) && (Controller.AxisC.position() == 0) && (Controller.AxisD.position() == 0)) {
    prev_error_x = error_x;
    prev_error_y = error_y;
    prev_error_heading = heading_error;
    prev_theta = theta;
    prev_leftwheeldist = leftwheeldist;
    prev_rightwheeldist = rightwheeldist;
    leftwheeldist = (leftdriveMotorA.position(degrees) + leftdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;
    rightwheeldist = (rightdriveMotorA.position(degrees) + rightdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;      change_rightwheel = rightwheeldist - prev_rightwheeldist;
    change_rightwheel = rightwheeldist - prev_rightwheeldist;
    change_leftwheel = leftwheeldist - prev_leftwheeldist;
    dist_moved = (change_leftwheel + change_rightwheel) / 2;
    current_theta = (change_leftwheel - change_rightwheel) / wheel_distance;
    theta = prev_theta + current_theta;
    if (theta > 180) {
      theta = 0;
      robotReverse = true;
    }
    if (robotReverse == false) {
      change_in_x = dist_moved * sin(theta);
      change_in_y = dist_moved * cos(theta);
    } 
    else {
      change_in_x = dist_moved * cos(theta);
      change_in_y = dist_moved * sin(theta);
    }
    x_pos = x_pos + change_in_x;
    y_pos = y_pos + change_in_y; 
    error_x = target_x - x_pos;
    error_y = target_y - y_pos;
    target_distance = sqrt(error_x * error_x + error_y * error_y);
    target_angle_odom = atan2(error_y, error_x);
    // Calculate the difference between the target angle and the current heading
    angle_difference = target_angle_odom - theta;
    if (target_angle_odom != target_angle) {
      angle_difference = target_angle_odom - theta;
      heading_error = angle_difference * (180 / pi);
    }
    else {
      angle_difference = target_angle_odom - theta;
      heading_error = angle_difference * (180 / pi);
    }
    proportional_x = error_x * kp_x;
    integral_x = (integral_x + error_x) * ki_x;
    if (error_x == 0) {
      integral_x = 0;
    } 
    if (integral_x > 25) {
      integral_x = 25;
    }
    derivative_x = (error_x - prev_error_x) * kd_x;
    pid_output_x = proportional_x + integral_x + derivative_x;
    proportional_y = error_y * kp_y;
    integral_y = (integral_y + error_y) * ki_y;
    if (error_y == 0) {
      integral_y = 0;
    } 
    if (integral_y > 25) {
      integral_y = 25;
    }
    derivative_y = (error_y - prev_error_y) * kd_y;
    pid_output_y = proportional_y + integral_y + derivative_y;
    proportional_heading = heading_error * kp_heading;
    integral_heading = (integral_heading + heading_error) * ki_heading;
    if (heading_error == 0) {
      integral_heading = 0;
    } 
    if (integral_heading > 25) {
      integral_heading = 25;
    }
    derivative_heading = (heading_error - prev_error_heading) * kd_heading;
    pid_output_heading = proportional_heading + integral_heading + derivative_heading;
    left_velocity = pid_output_x + pid_output_y + pid_output_heading;
    right_velocity = (pid_output_x + pid_output_y + pid_output_heading) * -1;
    // Set the velocities to move towards the target
    leftdrive.setVelocity(left_velocity, percent);
    rightdrive.setVelocity(right_velocity, percent);
    // Spin the motors
    leftdrive.spin(forward);
    rightdrive.spin(forward);
    Brain.Screen.print("x_pos: %f", x_pos);
    Brain.Screen.newLine();
    Brain.Screen.print("y_pos: %f", y_pos);
    Brain.Screen.newLine();
    Brain.Screen.setCursor(1, 1);
    wait(5, msec);
    Brain.Screen.clearScreen();  
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
  autonomous(0, 4, 0);
  autonomous(0, -2, 0);
  Brain.playSound(siren);
}