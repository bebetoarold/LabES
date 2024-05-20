/**
 * @file controller.cpp
 * @brief Implementation file for the PIDController class.
 *
 * This file contains the implementation of the PIDController class,
 * which implements a PID (Proportional-Integral-Derivative) controller
 * for controlling the inverted pendulum system.
 *
 * @author [Your Name]
 * @date [Date]
 */

#include "controller.h"
#include <iostream>

PIDController::PIDController() { update_params(kp, kd, ki); }

void PIDController::setClamp(double max, double min) {
  ///@todo Implement setClamp for setting the output limits

  //if (min > max) {
  //  std::swap(min, max);
  //}
  this->min = min;
  this->max = max;
}

double PIDController::output(double error) {
  ///@todo Implement the PID controller output calculation

  a0 = (1 + N * Ts);
  a1 = -(2 + N * Ts);
  a2 = 1;

std::cout << "a0: " << a0 << ", a1: " << a1 << ", a2: " << a2 << std::endl;
 std::cout << "kp: " << kp << ", kd: " << kd << ", ki: " << ki << std::endl;

std::cout << "b0: " << b0 << ", b1: " << b1 << std::endl;
std::cout << "ke0: " << ke0 << ", ke1: " << ke1 << ", ke2: " << ke2 << std::endl;
std::cout << "ku1: " << ku1 << ", ku2: " << ku2 << std::endl;

 

  e0=error; // update error
  u0= (-ku1*u1)-(ku2*u2) + (ke0*e0) + (ke1*e1) + (ke2*e2); // calculate control signal

  std::cout << "Force before clamping--u0: " << u0 << std::endl;

   // Clamp the output
  if (u0 > max) {
    u0 = max;
  } else if (u0 < min) {
    u0 = min;
  }
  
// Print u0 to the console
  //std::cout << "u0: " << u0 << std::endl;
  
  std::cout << "Force after clamping--u0: " << u0 << std::endl;
  return u0;
}

void PIDController::update_params(double kp_, double kd_, double ki_) {
  ///@todo Implement the update_params function for PID controller
  this->kp = kp_;
  this->kd = kd_;
  this->ki = ki_;


b0 = kp * (1 + N * Ts) + ki * Ts * (1 + N * Ts) + kd * N;
  b1 = -(kp * (2 + N * Ts) + ki * Ts + 2 * kd * N);
  b2 = kp + kd * N;

  ku1 = a1 / a0;
  ku2 = a2 / a0;
  ke0 = b0 / a0;
  ke1 = b1 / a0;
  ke2 = b2 / a0;

  e2=e1; e1=e0; u2=u1; u1=u0; // update variables

}

void PIDController::reset() {
    kp = 0.0;
  kd = 0.0;
  ki = 0.0;

  // Reset error and control signal
  e0 = 0.0;
  e1 = 0.0;
  e2 = 0.0;
  u0 = 0.0;
  u1 = 0.0;
  u2 = 0.0;

  // Reset coefficients ???
  a0 = 0.0;
  a1 = 0.0;
  a2 = 0.0;
  b0 = 0.0;
  b1 = 0.0;
  b2 = 0.0;

  // Reset subcalculation variables
  ku1 = 0.0;
  ku2 = 0.0;
  ke0 = 0.0;
  ke1 = 0.0;
  ke2 = 0.0;

}