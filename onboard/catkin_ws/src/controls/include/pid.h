#ifndef PID_H
#define PID_H

class PID
{
public:
    double *Kp;
    double *Ki;
    double *Kd;
    double *Ff;
    double integralClamp;
    double cutoffFreq;
    bool angleCorrection;

    PID();
    PID(double *Kp,
        double *Ki,
        double *Kd,
        double *Ff,
        double integralClamp = 1000.0,
        double cutoffFreq = 5.0,
        bool angleCorrection = true);

    void reset();
    double calcControlEffort(double error, double deltaTime);
};

#endif