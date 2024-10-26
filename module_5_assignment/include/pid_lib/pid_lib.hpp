#ifndef PID_LIB_HPP
#define PID_LIB_HPP

#include <iostream>
using namespace std;

class PID {
  public:
    // default constructor
    PID();
    PID(double Kp, double Ki, double Kd, double cmd_min, double cmd_max);

    // Function to perform the PID calculation
    double stepPID(double measurement, double setpoint, double dt);

    // Setter methods to update Kp, Ki, Kd
    void setKp(double Kp) { _Kp = Kp; }
    void setKi(double Ki) { _Ki = Ki; }
    void setKd(double Kd) { _Kd = Kd; }

    // Getter methods to retrieve Kp, Ki, Kd if needed
    double getKp() const { return _Kp; }
    double getKi() const { return _Ki; }
    double getKd() const { return _Kd; }

    // Destructor
    ~PID();

  private:
    double _Kp;              // Proportional gain constant
    double _Ki;              // Integral gain constant
    double _Kd;              // Derivative gain constant
    double _err_prev;        // Previous error
    double _integral_err;    // Integral term
    double _deriv_err_prev;  // Previous derivative
    double _cmd_min;         // Minimal clamp
    double _cmd_max;         // Maximum clamp
};

#endif
