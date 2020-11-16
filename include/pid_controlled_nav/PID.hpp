//
// Created by andrepascoa on 11/11/20.
//

#ifndef SRC_PID_HPP
#define SRC_PID_HPP

#include <cmath>

class PID {
private:
    double KP;
    double KI;
    double KD;
    double max, min;
    double prev_error = 0;
    double integral = 0;
public:
    PID(double KP, double KI, double KD, double max, double min);

    double calculate(double offset, double value, double DT);

};

#endif //SRC_PID_HPP
