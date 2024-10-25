#include "pid_lib/pid_lib.hpp"

// Default constructor
PID::PID() : _Kp(0), _Ki(0), _Kd(0), _err_prev(0), _integral_err(0), _cmd_min(0), _cmd_max(0) {}

// Parameterized constructor
PID::PID(double Kp, double Ki, double Kd, double cmd_min, double cmd_max) 
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _err_prev(0), _integral_err(0), _cmd_min(cmd_min), _cmd_max(cmd_max) {}

// PID step function to calculate control output
double PID::stepPID(double measurement, double setpoint, double dt) {
    double error = setpoint - measurement;
    _integral_err += error * dt;
    double derivative = (error - _err_prev) / dt;
    _err_prev = error;

    // Calculate the control command
    double output = (_Kp * error) + (_Ki * _integral_err) + (_Kd * derivative);

    // Clamp the output within the allowed range
    if (output > _cmd_max) output = _cmd_max;
    if (output < _cmd_min) output = _cmd_min;

    return output;
}

// Destructor
PID::~PID() {}
