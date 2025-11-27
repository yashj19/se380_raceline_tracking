import numpy as np
from numpy.typing import ArrayLike

from racetrack import RaceTrack


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return np.arctan2(np.sin(angle), np.cos(angle))


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.previous_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        P = self.kp * error
        
        self.integral += error * dt
        I = self.ki * self.integral
        
        derivative = (error - self.previous_error) / dt
        D = self.kd * derivative
        
        self.previous_error = error
        
        return P + I + D


DT = 0.01
steerPIDController = PIDController(kp=8, ki=50, kd=0.005)
accelPIDController = PIDController(kp=20, ki=2, kd=0)


def lower_controller(
    state: ArrayLike, desired: ArrayLike, parameters: ArrayLike
) -> ArrayLike:
    return np.array([
        steerPIDController.update(desired[0] - state[2], DT),
        accelPIDController.update(desired[1] - state[3], DT)
    ])


LOOKAHEAD_BASE = 12.0
SPEED_CURVATURE_GAIN = 1000.0
CURVATURE_SCAN_RANGE = 50
DISTANCE_DECAY = 0.01  # how quickly the weight diminishes with distance

def controller(
    state: ArrayLike, parameters: ArrayLike, racetrack: RaceTrack
) -> ArrayLike:
    
    state = np.asarray(state, dtype=float)
    parameters = np.asarray(parameters, dtype=float)
    
    px, py = state[0], state[1]
    vel = state[3]
    heading = state[4]
    
    wheelbase = parameters[0]
    steer_min = parameters[1]
    steer_max = parameters[4]
    vel_max = parameters[5]
    
    path = racetrack.centerline
    n = len(path)
    
    # nearest point on path
    dists = np.linalg.norm(path - np.array([px, py]), axis=1)
    nearest = int(np.argmin(dists))
    
    # walk ahead to find lookahead point for steering
    lookahead = LOOKAHEAD_BASE + 0.3 * vel
    traveled = 0.0
    idx = nearest
    while traveled < lookahead:
        next_idx = (idx + 1) % n
        traveled += np.linalg.norm(path[next_idx] - path[idx])
        idx = next_idx
    
    target = path[idx]
    
    # steering calculation
    dx = target[0] - px
    dy = target[1] - py
    dist_to_target = np.hypot(dx, dy)
    angle_to_target = np.arctan2(dy, dx)
    alpha = normalize_angle(angle_to_target - heading)
    
    steer = np.arctan2(2.0 * wheelbase * np.sin(alpha), dist_to_target)
    steer = np.clip(steer, steer_min, steer_max)
    
    # scan for max curvature with distance-based weighting
    # nearby curves matter more than far away curves
    max_weighted_curvature = 0.0
    traveled = 0.0
    
    for offset in range(3, CURVATURE_SCAN_RANGE):
        check_idx = (nearest + offset) % n
        prev_idx = (nearest + offset - 1) % n
        
        # accumulate distance traveled
        traveled += np.linalg.norm(path[check_idx] - path[prev_idx])
        
        p0 = path[(check_idx - 2) % n]
        p1 = path[check_idx]
        p2 = path[(check_idx + 2) % n]
        
        a = np.linalg.norm(p1 - p0)
        b = np.linalg.norm(p2 - p1)
        c = np.linalg.norm(p2 - p0)
        
        if a * b * c > 1e-8:
            area = 0.5 * abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]))
            curvature = 4.0 * area / (a * b * c)
            
            # weight diminishes with distance
            weight = 1.0 / (1.0 + DISTANCE_DECAY * traveled)
            weighted_curvature = curvature * weight
            
            max_weighted_curvature = max(max_weighted_curvature, weighted_curvature)
    
    # slow down for worst upcoming curve
    speed = vel_max / (1.0 + SPEED_CURVATURE_GAIN * max_weighted_curvature**2)
    speed = max(speed, 15.0)
    
    return np.array([steer, speed], dtype=float)
