# import numpy as np
# from numpy.typing import ArrayLike

# from racetrack import RaceTrack

# def normalize_angle(angle):
#     """Normalize angle to [-pi, pi]"""
#     return np.arctan2(np.sin(angle), np.cos(angle))

# def find_closest_reference(state: ArrayLike, racetrack: RaceTrack):
#     """Return the index of closest point on centerline to car position"""
#     car_position = state[0:2]
#     distances = np.linalg.norm(racetrack.centerline - car_position, axis=1)
#     closest_idx = np.argmin(distances)
#     return closest_idx

# # def find_lookahead_point(state: ArrayLike, racetrack: RaceTrack, lookahead_dist: float):
# #     """
# #     Find a point on the centerline that is approximately lookahead_dist ahead of the car.
# #     Ensures we always track FORWARD along the path, never backwards.
    
# #     Returns: (lookahead_index, lookahead_point)
# #     """
# #     car_position = state[0:2]
# #     car_heading = state[4]
# #     num_points = len(racetrack.centerline)
    
# #     # Find closest point first
# #     closest_idx = find_closest_reference(state, racetrack)
    
# #     # Search forward from closest point to find lookahead point
# #     accumulated_dist = 0.0
# #     lookahead_idx = closest_idx
    
# #     for i in range(1, min(100, num_points)):  # Search up to 100 points ahead
# #         curr_idx = (closest_idx + i - 1) % num_points
# #         next_idx = (closest_idx + i) % num_points
        
# #         segment_dist = np.linalg.norm(
# #             racetrack.centerline[next_idx] - racetrack.centerline[curr_idx]
# #         )
# #         accumulated_dist += segment_dist
        
# #         if accumulated_dist >= lookahead_dist:
# #             lookahead_idx = next_idx
# #             break
        
# #         lookahead_idx = next_idx
    
# #     return lookahead_idx, racetrack.centerline[lookahead_idx]

# # def get_cross_track_error(state: ArrayLike, racetrack: RaceTrack, closest_idx: int):
# #     """
# #     Calculate signed cross-track error (lateral distance from path).
# #     Positive = car is to the left of path, Negative = car is to the right.
# #     """
# #     car_position = state[0:2]
# #     closest_point = racetrack.centerline[closest_idx]

# #     # Get path direction at closest point
# #     num_points = len(racetrack.centerline)
# #     next_idx = (closest_idx + 1) % num_points
# #     path_direction = racetrack.centerline[next_idx] - closest_point
# #     path_direction = path_direction / (np.linalg.norm(path_direction) + 1e-6)

# #     # Vector from path to car
# #     to_car = car_position - closest_point

# #     # Cross-track error: component perpendicular to path
# #     cross_track_error = path_direction[0] * to_car[1] - path_direction[1] * to_car[0]

# #     return cross_track_error

# # def get_heading_error(state: ArrayLike, racetrack: RaceTrack, closest_idx: int):
# #     """
# #     Calculate heading error (difference between car heading and path heading).
# #     """
# #     car_heading = state[4]

# #     # Get path heading at closest point
# #     num_points = len(racetrack.centerline)
# #     next_idx = (closest_idx + 1) % num_points
# #     path_direction = racetrack.centerline[next_idx] - racetrack.centerline[closest_idx]
# #     path_heading = np.arctan2(path_direction[1], path_direction[0])

# #     heading_error = normalize_angle(path_heading - car_heading)
# #     return heading_error


# # # ============================================================================
# # # CONTROL APPROACH: Pure Pursuit + Proportional Velocity Control
# # # ============================================================================
# # #
# # # Pure Pursuit Controller (for steering):
# # #   - Geometric path tracking controller
# # #   - Calculates steering to reach a lookahead point on the path
# # #   - More robust than Stanley for sharp turns and large errors
# # #   - Steering: δ = arctan(2 * L * sin(α) / lookahead_dist)
# # #     where α is angle to lookahead point, L is wheelbase
# # #
# # # STATE ORDER: [sx, sy, delta, v, phi]
# # #   state[0] = sx (x position)
# # #   state[1] = sy (y position)  
# # #   state[2] = delta (steering angle)
# # #   state[3] = v (velocity)
# # #   state[4] = phi (heading angle)
# # #
# # # INPUT ORDER: [steering_rate, acceleration] = [v_delta, a]
# # # ============================================================================

# # def lower_controller(
# #     state : ArrayLike, desired : ArrayLike, parameters : ArrayLike
# # ) -> ArrayLike:
# #     """
# #     Lower controller using proportional control.
# #     Outputs [steering_rate, acceleration] to reach desired [steering_angle, velocity].

# #     State: [sx, sy, delta, v, phi]
# #     """
# #     assert(desired.shape == (2,))

# #     desired_steering = desired[0]
# #     desired_velocity = desired[1]

# #     current_steering = state[2]
# #     current_velocity = state[3]

# #     max_steering_rate = parameters[9]   # max_steering_vel
# #     max_acceleration = parameters[10]   # max_acceleration

# #     # Proportional gains - moderate to prevent oscillation
# #     k_steer = 2.0   # Reduced from 4.0
# #     k_vel = 2.0     # Acceleration gain

# #     # Proportional steering rate control
# #     steering_error = desired_steering - current_steering
# #     steering_rate = k_steer * steering_error
    
# #     # Limit steering rate more aggressively to prevent spinning
# #     max_rate = 0.8 * max_steering_rate  # Use only 80% of max
# #     steering_rate = np.clip(steering_rate, -max_rate, max_rate)

# #     # Proportional velocity control
# #     velocity_error = desired_velocity - current_velocity
# #     acceleration = k_vel * velocity_error
# #     acceleration = np.clip(acceleration, -max_acceleration, max_acceleration)

# #     return np.array([steering_rate, acceleration])


# # def controller(
# #     state : ArrayLike, parameters : ArrayLike, racetrack : RaceTrack
# # ) -> ArrayLike:
# #     """
# #     High-level controller using Pure Pursuit for steering.
# #     Outputs desired [steering_angle, velocity].

# #     Pure Pursuit: δ = arctan(2 * L * sin(α) / ld)
# #     where:
# #       - L = wheelbase
# #       - α = angle from car heading to lookahead point
# #       - ld = lookahead distance

# #     State: [sx, sy, delta, v, phi]
# #     """
# #     car_position = state[0:2]
# #     car_velocity = state[3]
# #     car_heading = state[4]
    
# #     wheelbase = parameters[0]
# #     max_steering = parameters[4]
# #     max_velocity = parameters[5]
# #     max_acceleration = parameters[10]
    
# #     num_points = len(racetrack.centerline)
    
# #     # Find closest point for error calculation
# #     closest_idx = find_closest_reference(state, racetrack)
# #     cross_track_error = get_cross_track_error(state, racetrack, closest_idx)
# #     heading_error = get_heading_error(state, racetrack, closest_idx)
    
# #     # -------------------------------------------------------------------------
# #     # Pure Pursuit Steering
# #     # -------------------------------------------------------------------------
    
# #     # Lookahead distance scales with velocity (minimum 5m, max 25m)
# #     min_lookahead = 5.0
# #     max_lookahead = 25.0
# #     lookahead_dist = np.clip(0.5 * abs(car_velocity), min_lookahead, max_lookahead)
    
# #     # Also increase lookahead when off-track (gives more time to correct)
# #     if abs(cross_track_error) > 3.0:
# #         lookahead_dist = max(lookahead_dist, 15.0)
    
# #     # Find lookahead point
# #     lookahead_idx, lookahead_point = find_lookahead_point(state, racetrack, lookahead_dist)
    
# #     # Vector from car to lookahead point
# #     to_lookahead = lookahead_point - car_position
# #     dist_to_lookahead = np.linalg.norm(to_lookahead) + 1e-6
    
# #     # Angle to lookahead point in global frame
# #     angle_to_lookahead = np.arctan2(to_lookahead[1], to_lookahead[0])
    
# #     # Alpha: angle from car heading to lookahead point
# #     alpha = normalize_angle(angle_to_lookahead - car_heading)
    
# #     # Pure Pursuit steering law
# #     # δ = arctan(2 * L * sin(α) / ld)
# #     reference_steering = np.arctan2(2.0 * wheelbase * np.sin(alpha), dist_to_lookahead)
    
# #     # Clamp steering to vehicle limits (use 90% of max for safety margin)
# #     reference_steering = np.clip(reference_steering, -0.9 * max_steering, 0.9 * max_steering)
    
# #     # -------------------------------------------------------------------------
# #     # Velocity control: slow down for curves, errors, and upcoming turns
# #     # -------------------------------------------------------------------------
    
# #     # 1. Curvature-based velocity limit
# #     lookahead_curv = 20  # Look ahead for curvature
# #     prev_idx = (closest_idx - 1) % num_points
# #     next_idx = (closest_idx + lookahead_curv) % num_points

# #     p0 = racetrack.centerline[prev_idx]
# #     p1 = racetrack.centerline[closest_idx]
# #     p2 = racetrack.centerline[next_idx]

# #     v1 = p1 - p0
# #     v2 = p2 - p1
# #     heading1 = np.arctan2(v1[1], v1[0])
# #     heading2 = np.arctan2(v2[1], v2[0])
# #     delta_heading = abs(normalize_angle(heading2 - heading1))

# #     dist = np.linalg.norm(p2 - p0) + 1e-6
# #     curvature = delta_heading / dist

# #     a_lateral_max = 0.4 * max_acceleration
# #     if curvature > 0.001:
# #         v_curvature = np.sqrt(a_lateral_max / curvature)
# #     else:
# #         v_curvature = max_velocity

# #     # 2. Cross-track error velocity limit (slow down when off track)
# #     cross_track_abs = abs(cross_track_error)
# #     if cross_track_abs > 8.0:
# #         v_error_limit = 5.0   # Very off track - crawl
# #     elif cross_track_abs > 4.0:
# #         v_error_limit = 10.0  # Significantly off
# #     elif cross_track_abs > 2.0:
# #         v_error_limit = 20.0  # Moderately off
# #     else:
# #         v_error_limit = max_velocity

# #     # 3. Heading error velocity limit (slow down when pointing wrong way)
# #     heading_error_abs = abs(heading_error)
# #     if heading_error_abs > 1.0:      # ~57 degrees - major misalignment
# #         v_heading_limit = 5.0
# #     elif heading_error_abs > 0.5:    # ~30 degrees
# #         v_heading_limit = 12.0
# #     elif heading_error_abs > 0.25:   # ~15 degrees
# #         v_heading_limit = 25.0
# #     else:
# #         v_heading_limit = max_velocity

# #     # 4. Steering-based velocity limit (slow down during sharp turns)
# #     steering_abs = abs(reference_steering)
# #     if steering_abs > 0.6:           # Sharp turn
# #         v_steering_limit = 15.0
# #     elif steering_abs > 0.3:         # Medium turn
# #         v_steering_limit = 30.0
# #     else:
# #         v_steering_limit = max_velocity

# #     # Take minimum of all limits
# #     reference_velocity = min(
# #         max_velocity,
# #         v_curvature,
# #         v_error_limit,
# #         v_heading_limit,
# #         v_steering_limit,
# #         45.0  # Absolute cap
# #     )

# #     # Ensure minimum velocity
# #     reference_velocity = max(5.0, reference_velocity)

# #     return np.array([reference_steering, reference_velocity])


# # ============================================================================
# # COMMENTED OUT: Original visibility cone approach
# # ============================================================================

# def lower_controller(
#     state : ArrayLike, desired : ArrayLike, parameters : ArrayLike
# ) -> ArrayLike:
#     """
#     Lower controller: outputs [steering_rate, acceleration] to reach desired [steering_angle, velocity]
#     Uses bang-bang control (max effort in the right direction)

#     State: [sx, sy, delta, v, phi]
#     Parameters: [wheelbase, -max_steer, min_vel, -pi, max_steer, max_vel, pi, -max_steer_rate, -max_accel, max_steer_rate, max_accel]
#     """
#     assert(desired.shape == (2,))

#     desired_steering = desired[0]
#     desired_velocity = desired[1]

#     current_steering = state[2]
#     current_velocity = state[3]

#     max_steering_rate = parameters[9]*0.5  # max_steering_vel
#     max_acceleration = parameters[10]*0.5  # max_acceleration

#     # Steering control: bang-bang to reach desired steering angle
#     steering_error = desired_steering - current_steering
#     if steering_error > 0.01:
#         steering_rate = max_steering_rate
#     elif steering_error < -0.01:
#         steering_rate = -max_steering_rate
#     else:
#         steering_rate = 0.0

#     # Velocity control: bang-bang to reach desired velocity
#     velocity_error = desired_velocity - current_velocity
#     if velocity_error > 0.1:
#         acceleration = max_acceleration
#     elif velocity_error < -0.1:
#         # print("HELLO")
#         acceleration = -max_acceleration
#     else:
#         acceleration = 0.0

#     return np.array([steering_rate, acceleration])


# def controller(
#     state : ArrayLike, parameters : ArrayLike, racetrack : RaceTrack
# ) -> ArrayLike:
#     """
#     High-level controller: outputs desired [steering_angle, velocity]

#     State: [sx, sy, delta, v, phi] where phi is heading
#     """
#     car_position = state[0:2]
#     car_heading = state[4]

#     closest_idx = find_closest_reference(state, racetrack)
#     max_index = closest_idx

#     num_points = len(racetrack.centerline)
#     max_lookahead = 10  # Maximum number of indices to look ahead

#     # Set max right/left angle thresholds to infinity originally
#     max_right_threshold = np.inf
#     max_left_threshold = np.inf

#     # Iterate through centerline points starting from closest (limited lookahead)
#     for i in range(closest_idx, closest_idx + max_lookahead):
#         idx = i % num_points  # wrap around for circular track

#         centerline_point = racetrack.centerline[idx]

#         # Calculate angle from car to this centerline point
#         direction_to_point = centerline_point - car_position
#         angle_to_point = np.arctan2(direction_to_point[1], direction_to_point[0])

#         # Angle relative to car's heading (positive = left, negative = right)
#         relative_angle = normalize_angle(angle_to_point - car_heading)

#         # Check if this point is outside our visibility cone
#         if relative_angle > max_left_threshold or relative_angle < -max_right_threshold:
#             break

#         # Update maxIndex since we can see this point
#         max_index = idx

#         # Get boundary points at this index (already computed in racetrack)
#         right_boundary_point = racetrack.right_boundary[idx]
#         left_boundary_point = racetrack.left_boundary[idx]

#         # Calculate angle to right boundary point
#         direction_to_right = right_boundary_point - car_position
#         angle_to_right = np.arctan2(direction_to_right[1], direction_to_right[0])
#         relative_angle_right = normalize_angle(angle_to_right - car_heading)

#         # Calculate angle to left boundary point
#         direction_to_left = left_boundary_point - car_position
#         angle_to_left = np.arctan2(direction_to_left[1], direction_to_left[0])
#         relative_angle_left = normalize_angle(angle_to_left - car_heading)

#         # Update thresholds (shrink the cone based on boundary visibility)
#         # Right threshold: how far right we can look (negative angle)
#         if relative_angle_right < 0:
#             max_right_threshold = min(max_right_threshold, -relative_angle_right)

#         # Left threshold: how far left we can look (positive angle)
#         if relative_angle_left > 0:
#             max_left_threshold = min(max_left_threshold, relative_angle_left)

#     # Calculate reference steering angle: angle to maxIndex point relative to heading
#     target_point = racetrack.centerline[max_index]
#     direction_to_target = target_point - car_position
#     angle_to_target = np.arctan2(direction_to_target[1], direction_to_target[0])
#     reference_steering = normalize_angle(angle_to_target - car_heading)

#     # Clamp steering to vehicle limits
#     max_steering = parameters[4]  # max_steering_angle
#     reference_steering = np.clip(reference_steering, -max_steering, max_steering)

#     # Calculate reference velocity
#     # Goal: reach max velocity but be able to brake to 0 at maxIndex
#     max_velocity = parameters[5]
#     max_acceleration = parameters[10]

#     # Distance to maxIndex point
#     d_remaining = np.linalg.norm(target_point - car_position)

#     # From v^2 = v0^2 + 2*a*d, to stop (v=0) from v_ref with deceleration -a_max:
#     # 0 = v_ref^2 - 2*a_max*d_remaining
#     # v_ref = sqrt(2 * a_max * d_remaining)
#     reference_velocity = min(max_velocity, np.sqrt(2 * max_acceleration * d_remaining))

#     # Ensure non-negative velocity
#     reference_velocity = max(0.0, reference_velocity)

#     return np.array([reference_steering, reference_velocity])