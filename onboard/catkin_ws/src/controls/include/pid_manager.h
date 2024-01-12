#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <memory>
#include <unordered_map>
#include "controls_utils.h"
#include "drc_pid.h"

class PIDManager
{
public:
    std::unordered_map<AxesEnum, PID> pid_controllers;

    PIDManager();
    PIDManager(AxesPIDGainsMap pid_gains_for_axes);
    void run_loops(const std::unordered_map<AxesEnum, double> &errors,
                   const std::unordered_map<AxesEnum, double> &deltaTimes,
                   std::unordered_map<AxesEnum, double> &outputs);
    void reset(AxesEnum axis_to_reset);
};

#endif