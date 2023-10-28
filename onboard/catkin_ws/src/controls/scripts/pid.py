#!/usr/bin/env python3

"""
TODO: Implement a PID controller
The PID controller should be a class that is initialized with the PID gains (Kp, Ki, Kd, and feedforward).

There should be a method that takes in the error value and delta time and returns the control effort. The feedfoward
term should be added to the control effort just before returning. The final returned value must always be
between -1 and 1 (inclusive).

There should be a method that can reset the PID controller; this would be called when a new setpoint is given.

It should be possible for the PID gains to be changed at any time, even while the controller is running.

Context: One instance of this class will be created per axis (x, y, z, roll, pitch, yaw) per control type
(position, velocity). The first method described above will be called every time /state is published. The second
method will be called every time a new setpoint is published to /desired_pose or /desired_twist. The PID gains will
be updated every time a service is called to change them (this service will be advertised by separate node).

IMPORTANT: Make sure to look at the ROS PID implementation at (https://bitbucket.org/AndyZe/pid/src). Pay special
attention to the filter and determine if it is necessary for our implementation. Also determine if it is necessary
to impose a windup limit. See https://wiki.ros.org/pid for more information on the filter and the package in general.
"""


import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd, Ff, intClamp = 1000.0, cutoffFreq = 5, angleCorrection = True):
        self.cutoffFreq = cutoffFreq # 1/4 sampling = 5Hz default
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ff = Ff
        self.intClamp = intClamp
        self.angleCorrection = angleCorrection
        
        # Error/Time Array
        self.prevValues = np.zeros((3, 3)) # Rows are error, time, derivative
        # Integral error
        self.intError = 0
    
    def reset(self):
        '''
        Function to reset all params in memory
        '''
        self.intError = 0
        self.prevValues.fill(0)

    def setCutoff(self, cutoffFreq):
        self.cutoffFreq = cutoffFreq
   
    def setFf(self, Ff):
        self.Ff = Ff
       
    def setWindupClamp(self, intClamp):
        self.intClamp = intClamp
       
    def setKp(self, Kp):
        self.Kp = Kp
        
    def setKi(self, Ki):
        self.Ki = Ki
        
    def setKd(self, Kd):
        self.Kd = Kd
        
    def calcControlEffort(self, error, dt):
        # Update value array
        self.prevValues[0, 0], self.prevValues[0, 1], self.prevValues[0, 2] = error, self.prevValues[0, 0], self.prevValues[0, 1]
        self.prevValues[1, 0], self.prevValues[1, 1], self.prevValues[1, 2] = dt + self.prevValues[1, 0], self.prevValues[1, 0], self.prevValues[1, 1]
        
        # Angle wrap to cast all angles [-180, 180]
        # Not sure if this is needed since the error is passed rather than the PV. If error is clamped to [-180, 180] already, this is redundant.
        if self.angleCorrection:
            error = (error - np.pi) % (2 * np.pi) - np.pi
        
        # Update int.
        self.intError += error * dt
        
        # Integral windup to protect against sudden setpoint changes or unactuatable states
        #self.intError = min(max(self.intError, -self.intClamp), self.intClamp)
        
        self.prevValues[2, 0], self.prevValues[2, 1], self.prevValues[2, 2] = (self.prevValues[0, 1] - self.prevValues[0, 0]) / dt, self.prevValues[2, 0], self.prevValues[2, 1]
        
        # Add filtering of error here if needed to smooth out error and errorDerivative
        
        # Calculate control effort 
        ### TODO: Replace with filtered params ###
        controlOutput = self.Kp * self.prevValues[0, 0] + self.Ki * self.intError + self.Kd * self.prevValues[2, 0] + self.Ff
        
        # Clamp to effort limits
        controlOutput = min(max(controlOutput, -1), 1)
        return controlOutput
