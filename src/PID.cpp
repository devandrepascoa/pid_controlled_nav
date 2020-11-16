//
// Created by andrepascoa on 11/11/20.
//

#include <pid_controlled_nav/PID.hpp>
#include <algorithm>

PID::PID(double KP, double KI, double KD, double max, double min) : KP(KP), KI(KI), KD(KD), min(min), max(max) {}

double PID::calculate(double offset, double currentValue, double DT) {
    double error = currentValue - offset;
    double p_comp = KP * error;
    integral += error * DT;
    double i_comp = KI * integral;
    double d_comp = KD * (error - prev_error) / DT;

    double output = p_comp + i_comp + d_comp;

    prev_error = error;

    return std::max(std::min(max, output), min);
}



