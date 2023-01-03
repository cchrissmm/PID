#ifndef PID_H
#define PID_H

class PIDController {
public:
  // Constructor
  PIDController(float kp, float ki, float kd, float windupLimit);

  // Update the PID controller
  float update(float error, float dt);

  // Set the values of kp, ki, and kd
  void setTunings(float kp, float ki, float kd);

  // Set the windup limit
  void setWindupLimit(float windupLimit);

  // Reset the integral and derivative terms
  void reset();

private:
  float kp, ki, kd;
  float windupLimit;
  float integral = 0;
  float previousError = 0;
};

#endif
