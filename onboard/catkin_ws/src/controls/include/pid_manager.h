#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <memory>
#include <unordered_map>

#include "controls_utils.h"
#include "drc_pid.h"

class PIDManager
{
public:
    /**
     * @brief PID controllers for each axis
     */
    std::unordered_map<AxesEnum, PID> pid_controllers;

    /**
     * @brief Default constructor
     */
    PIDManager();

    /**
     * @brief Constructor
     *
     * @param pid_gains_for_axes Pointers to PID gains for each axis
     * @param pid_terms_for_axes Pointers to PID terms for each axis
     */
    PIDManager(AxesPIDGainsMap pid_gains_for_axes, AxesPIDGainsMap pid_terms_for_axes);

    /**
     * @brief Run PID loops
     *
     * @param errors Errors for each axis
     * @param deltaTimes Delta times for each axis
     * @param[out] outputs PID outputs for each axis
     */
    void run_loops(const std::unordered_map<AxesEnum, double> &errors,
                   const std::unordered_map<AxesEnum, double> &deltaTimes,
                   std::unordered_map<AxesEnum, double> &outputs);

    /**
     * @brief Reset PID loop for a given axis
     *
     * @param axis_to_reset Axis to reset
     */
    void reset(AxesEnum axis_to_reset);
};

#endif