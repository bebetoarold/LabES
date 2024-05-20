/**
 * @file controller.h
 * @brief Header file for the Controller interface and PIDController class.
 *
 * This file declares the Controller interface, which defines the interface for
 * controllers used in the inverted pendulum simulation. It also declares the
 * PIDController class, which implements a PID
 * (Proportional-Integral-Derivative) controller for controlling the pendulum
 * system.
 *
 * @author Utkarsh Raj
 * @date 10-April-2024
 */

#pragma once

/**
 * @brief Interface for controllers used in the inverted pendulum simulation.
 *
 * The Controller interface defines the common interface for controllers
 * used in the simulation. It provides methods for computing control signals,
 * updating controller parameters, and setting output signal clamping limits.
 */
class Controller {
public:
  /**
   * @brief Computes the control output based on the given error.
   *
   * This method computes the control output based on the error between
   * the desired reference value and the current system state.
   *
   * @param error The error (difference between reference value and current
   * state).
   * @return The control output computed by the controller.
   */
  virtual double output(double error) = 0;
  /**
   * @brief Updates the controller parameters dynamically.
   *
   * This method allows for dynamic updating of controller parameters,
   * such as proportional, integral, and derivative gains.
   *
   * @param kp The proportional gain.
   * @param kd The derivative gain.
   * @param ki The integral gain.
   */
  virtual void update_params(double kp, double kd, double ki) = 0;

  /**
   * @brief Sets the clamping limits for the control output.
   *
   * This method allows setting upper and lower limits for the control output
   * to prevent excessive control action.
   *
   * @param max The maximum allowed control signal.
   * @param min The minimum allowed control signal.
   */
  virtual void setClamp(double max, double min) = 0;
};

/**
 * @brief Implementation of a PID (Proportional-Integral-Derivative) controller.
 *
 * The PIDController class implements a PID controller for controlling the
 * inverted pendulum system. It computes control signals based on proportional,
 * integral, and derivative terms, and provides methods for updating controller
 * parameters and setting clamping limits for the control output.
 */
class PIDController : public Controller {
  //@todo Add private members for PIDController class
public:
  /**
   * @brief Default constructor.
   *
   * Initializes the PID controller with default proportional, derivative,
   * and integral gains.
   */
  double kp = 0.0;
  double kd = 0.0;
  double ki = 0.0;
  ///@todo Intialize paramaters for discrete PID controller
  
    /* data */
 // Clamping limits
  double min = -2048, max = 2048;

  // Error and control signal
  double e2 = 0.0, e1 = 0.0, e0 = 0.0, u2 = 0.0, u1 = 0.0, u0 = 0.0;

  // Filter coefficient
  int N = 100;

  // Sampling time
  double Ts = 0.001;

  // Coefficients
  double  a0 = (1 + N * Ts),
  a1 = -(2 + N * Ts),
  a2 = 1, b0 = 0.0, b1 = 0.0, b2 = 0.0;

  // Subcalculation variables
  double ku1 = 0.0, ku2 = 0.0, ke0 = 0.0, ke1 = 0.0, ke2 = 0.0;


    
  
  

  PIDController();

  /**
   * @brief Computes the control output based on the given error.
   *
   * This method implements the PID control algorithm to compute the control
   * output based on the error between the desired reference value and the
   * current system state.
   *
   * @param error The error (difference between reference value and current
   * state).
   * @return The control output computed by the PID controller.
   */
  double output(double error);
  /**
   * @brief Updates the controller parameters (gains).
   *
   * This method allows for dynamic updating of the PID controller's gains
   * (proportional, derivative, and integral gains).
   *
   * @param kp The new proportional gain.
   * @param kd The new derivative gain.
   * @param ki The new integral gain.
   */
  void update_params(double kp, double kd, double ki);
  /**
   * @brief Sets the clamping limits for the control output.
   *
   * This method allows setting upper and lower limits for the control output
   * to prevent excessive control action.
   *
   * @param max The maximum allowed control signal.
   * @param min The minimum allowed control signal.
   */
  void setClamp(double max, double min);


  /**
 * @brief Resets the internal state of the PID controller.
 *
 * This method clears the internal state of the PID controller, setting all
 * internal variables to zero. This might be necessary when the controller is
 * disabled and then re-enabled, or when the setpoint changes abruptly.
 */
void reset();
};
