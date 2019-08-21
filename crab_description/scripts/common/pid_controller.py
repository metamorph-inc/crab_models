"""
# Name: pid_controller.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Email: jcoombe@metamorphsoftware.com
# Create Date: 09/07/2017
# Edit Date: 04/07/2018
#
# Simple PID controller
# Based on https://en.wikipedia.org/wiki/PID_controller and
# https://github.com/sezan92/self_balancing_robot/blob/master/src/pidcontrol.py
"""

import math


# General PID controller class
class PIDController(object):

    # Construct a new PID_Controller object
    def __init__(self, kp, ki, kd, windup_min=-20, windup_max=+20):

        # Parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # State Variables
        self.prev_error = 0
        self.integral = 0
        self.time = 0

        self.windup_min = windup_min
        self.windup_max = windup_max

    def set_kp(self, kp_new):
        self.kp = kp_new

    def set_ki(self, ki_new):
        self.ki = ki_new

    def set_kd(self, kd_new):
        self.kd = kd_new

    def tune(self, kp_new, ki_new, kd_new):
        self.set_kp(kp_new)
        self.set_ki(ki_new)
        self.set_kd(kd_new)

    # Return PID correction based on setpoint and measured value
    def get_correction(self, setpoint, measured, dt=1):
        # Update integral and derivative terms
        error = setpoint - measured
        derivative = 0
        if self.time > 0:
            self.integral += error*dt
            derivative = (error - self.prev_error) / dt

        if self.integral < self.windup_min:
            self.integral = self.windup_min
        elif self.integral > self.windup_max:
            self.integral = self.windup_max

        # Update object state variables
        self.time += 1
        self.prev_error = error

        # Calculate correction
        correction = self.kp*error + self.ki*self.integral + self.kd*derivative

        return correction
