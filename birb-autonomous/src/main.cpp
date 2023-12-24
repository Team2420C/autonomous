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
#include <cstdint>

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

motor rightdriveMotorA = motor(PORT7, true);
motor rightdriveMotorB = motor(PORT8, false);
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
// const float wheel_distance = 4.5;
const float gear_ratio = 3;
const float kp_distance = 30;
const float ki_distance = 0;
const float kd_distance = 0;
const float kp_heading = .3;
const float ki_heading = 0;
const float kd_heading = 0;
float proportional_distance;
float integral_distance;
float derivative_distance;
float proportional_heading;
float integral_heading;
float derivative_heading;
float pid_output_distance;
float pid_output_heading;
float left_velocity;
float right_velocity;
float theta;
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
float prev_error_distance;
float prev_error_heading;
float dist_moved;
float angle_difference;
float target_angle;
float target_angle_odom;
float target_distance;
float heading_error;

void updatePosition() {
  while (true) {
    if (!isAutonomousRunning) {
      prev_leftwheeldist = leftwheeldist;
      prev_rightwheeldist = rightwheeldist;
      leftwheeldist = (leftdriveMotorA.position(degrees) + leftdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;
      rightwheeldist = (rightdriveMotorA.position(degrees) + rightdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;
      change_rightwheel = rightwheeldist - prev_rightwheeldist;
      change_leftwheel = leftwheeldist - prev_leftwheeldist;
      dist_moved = (change_leftwheel + change_rightwheel) / 2;
      theta = BrainInertial.heading(degrees) * (pi / 180);
      x_pos += dist_moved * sin(theta);
      y_pos += dist_moved * cos(theta);
      printf("x: %f", x_pos);
      printf("y: %f", y_pos);
      // Adjust the wait time as needed
      wait(2, msec);
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
    constexpr int32_t DEADBAND = 5;
    while ((Controller.AxisA.position() != 0) || (Controller.AxisD.position() != 0)) {
      int32_t left_speed = Controller.AxisA.position();
      int32_t right_speed = Controller.AxisD.position();
      if (std::abs(left_speed) < DEADBAND) left_speed = 0;
      if (std::abs(right_speed) < DEADBAND) right_speed = 0;
      leftdrive.setVelocity(left_speed, percent);
      rightdrive.setVelocity(right_speed, percent);
      leftdrive.spin(forward);
      rightdrive.spin(forward);
      wait(1, msec);
    } 
    if (!isAutonomousRunning) {
      leftdrive.setVelocity(0, percent);
      rightdrive.setVelocity(0, percent);
    }
    wait(5, msec);
  }
}

void autonomous(float target_x, float target_y, float target_angle) {
  isAutonomousRunning = true;
  while (fabs(x_pos - target_x) > 0.1 && fabs(y_pos - target_y) > 0.1) {
    if (Controller.AxisA.position() != 0 || Controller.AxisD.position() != 0) {
      return;
    }
    prev_error_distance = target_distance;
    prev_error_heading = heading_error;
    prev_leftwheeldist = leftwheeldist;
    prev_rightwheeldist = rightwheeldist;
    leftwheeldist = (leftdriveMotorA.position(degrees) + leftdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;
    rightwheeldist = (rightdriveMotorA.position(degrees) + rightdriveMotorB.position(degrees)) / 2 * (wheel_circumference / 360) / gear_ratio;
    change_rightwheel = rightwheeldist - prev_rightwheeldist;
    change_leftwheel = leftwheeldist - prev_leftwheeldist;
    dist_moved = (change_leftwheel + change_rightwheel) / 2;
    theta = BrainInertial.heading(degrees) * (pi / 180);
    x_pos += dist_moved * sin(theta);
    y_pos += dist_moved * cos(theta);
    error_x = target_x - x_pos;
    error_y = target_y - y_pos;
    target_distance = sqrt(error_x * error_x + error_y * error_y);
    target_angle_odom = atan2(error_y, error_x);
    
    if ((target_angle_odom  * (180 / pi)) != target_angle) {
      angle_difference = target_angle_odom - theta;
      heading_error = angle_difference * (180 / pi);
    }
    else {
      angle_difference = (target_angle * (pi / 180)) - theta;
      heading_error = angle_difference * (180 / pi);
    }

    proportional_distance = target_distance * kp_distance;
    integral_distance = (integral_distance + target_distance);
    
    if (target_distance == 0) {
      integral_distance = 0;
    } 
    
    if (integral_distance > 75) {
      integral_distance = 75;
    }

    derivative_distance = (target_distance - prev_error_distance) * kd_distance;
    pid_output_distance = proportional_distance + integral_distance * ki_distance + derivative_distance;
    proportional_heading = heading_error * kp_heading;
    integral_heading = (integral_heading + heading_error);
    
    if (heading_error == 0) {
      integral_heading = 0;
    } 
    
    if (integral_heading > 75) {
      integral_heading = 75;
    }

    derivative_heading = (heading_error - prev_error_heading) * kd_heading;
    pid_output_heading = proportional_heading + integral_heading * ki_heading + derivative_heading;
    left_velocity = pid_output_distance + pid_output_heading;
    right_velocity = pid_output_distance + (pid_output_heading * -1);
    
    // Set the velocities to move towards the target
    leftdrive.setVelocity(left_velocity, percent);
    rightdrive.setVelocity(right_velocity, percent);
    
    // Spin the motors
    leftdrive.spin(forward);
    rightdrive.spin(forward);
    printf("x: %f", x_pos);
    printf("y: %f", y_pos);
    wait(2, msec);
    Brain.Screen.clearScreen();  
  }

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
  ///////////////////////
  // TEST JUST ODOMETRY
  vex::thread positionUpdater(updatePosition);
  vex::thread drivercontroll(driver);
  ///////////////////////
  // TEST PID
  // autonomous(0, 4, 0);
  // Brain.playSound(siren);
  // waitUntil(!isAutonomousRunning);
  // autonomous(0, 2, 0);
  // Brain.playSound(siren);
}