import numpy as np
from numpy.typing import ArrayLike

from racetrack import RaceTrack

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return np.arctan2(np.sin(angle), np.cos(angle))


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # proportional gain
        self.ki = ki  # integral gain
        self.kd = kd  # derivative gain
        
        self.previous_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        # proportional term
        P = self.kp * error
        
        # integral term
        self.integral += error * dt
        I = self.ki * self.integral
        
        # derivative term
        derivative = (error - self.previous_error) / dt
        D = self.kd * derivative
        
        # update previous error
        self.previous_error = error
        
        return P + I + D
    

def lower_controller(
    state : ArrayLike, desired : ArrayLike, parameters : ArrayLike
) -> ArrayLike:
    pass

def controller(
    state : ArrayLike, parameters : ArrayLike, racetrack : RaceTrack
) -> ArrayLike:
    pass