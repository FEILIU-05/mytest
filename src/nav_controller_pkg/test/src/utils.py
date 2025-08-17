#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import division
import math
import tf
import rospy

def q_to_yaw(q):
    """
    将 geometry_msgs/Quaternion 转换为欧拉角（偏航角）。
    """
    return tf.transformations.euler_from_quaternion(
        [q.x, q.y, q.z, q.w])[2]

def ang_diff(a, b):
    """
    计算两个角度之间的最小差值。
    """
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d

class PID(object):
    """
    简单的PID控制器实现，用于精调。
    """
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev = 0.0
        self.intg = 0.0

    def step(self, err, dt):
        self.intg += err * dt
        # 避免除以零
        der = (err - self.prev) / dt if dt else 0.0
        self.prev = err
        return self.kp * err + self.ki * self.intg + self.kd * der

    def reset(self):
        self.prev = 0.0
        self.intg = 0.0

class PIDController(object):
    """
    完整的PID控制器实现，用于物体检测微调。
    """
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, output_min=None, output_max=None, integral_max=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max if integral_max is not None else 10.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None # Not used in compute, but kept for consistency if dt isn't passed

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, dt=None):
        if dt is None or dt <= 0:
            dt = 0.1 # Default small dt to prevent division by zero or large jump
        
        P_out = self.Kp * error
        
        self.integral += error * dt
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max
        I_out = self.Ki * self.integral
        
        D_out = 0.0
        if dt > 1e-6: # Prevent division by near zero
            D_out = self.Kd * ((error - self.prev_error) / dt)
        
        self.prev_error = error
        
        output = P_out + I_out + D_out
        
        if self.output_max is not None and output > self.output_max:
            output = self.output_max
            # Anti-windup: if output is saturated, reduce integral to avoid overshoot
            if self.integral != 0:
                self.integral = self.integral * 0.95
        if self.output_min is not None and output < self.output_min:
            output = self.output_min
            # Anti-windup
            if self.integral != 0:
                self.integral = self.integral * 0.95
                
        return output