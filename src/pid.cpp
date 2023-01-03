#include "pid.h"

// Constructor
PIDController::PIDController(float kp, float ki, float kd, float windupLimit) : kp(kp), ki(ki), kd(kd), windupLimit(windupLimit) {}

// Update the PID controller
float PIDController::update(float error, float dt)
{
    integral += error * dt;

    // Check for windup
    if (integral > windupLimit)
    {
        integral = windupLimit;
    }
    else if (integral < -windupLimit)
    {
        integral = -windupLimit;
    }

    float derivative = (error - previousError) / dt;
    float output = kp * error + ki * integral + kd * derivative;
    previousError = error;
    return output;
}

// Set the values of kp, ki, and kd
void setTunings(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// Set the windup limit
void setWindupLimit(float windupLimit)
{
    this->windupLimit = windupLimit;
}

// Reset the integral and derivative terms
void reset()
{
    integral = 0;
    previousError = 0;
}