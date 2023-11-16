#ifndef PID_H
#define PID_H

#include <map>
#include "controls_utils.h"

class PID
{
public:
    std::map<PIDGainTypesEnum, double> *pid_gains;
    double integralClamp;
    double cutoffFreq;
    bool angleCorrection;

    PID();
    PID(std::map<PIDGainTypesEnum, double> &pid_gains,
        double integral_clamp = 1000.0,
        double cutoff_freq = 5.0,
        bool angle_correction = true);

    void reset();
    double calcControlEffort(double error, double deltaTime);
};

#endif