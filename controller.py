import numpy as np
from numpy.typing import ArrayLike

from simulator import RaceTrack

def lower_controller(
    state : ArrayLike, desired : ArrayLike, parameters : ArrayLike
) -> ArrayLike:
    # [steer angle, velocity]
    assert(desired.shape == (2,))

    return np.array([0, 100]).T

def find_closest_reference(state: ArrayLike, racetrack: RaceTrack):
    # return the closest point in centerline of racetrack to state x,y (index 0 and 1)
    car_position = state[0:2]
    distances = np.linalg.norm(racetrack.centerline - car_position, axis=1)
    closest_idx = np.argmin(distances)
    return closest_idx


def controller(
    state : ArrayLike, parameters : ArrayLike, racetrack : RaceTrack
) -> ArrayLike:
    maxIndex = find_closest_reference(state, racetrack)
    # set max right threshold and max left threshold for angles to infinity originally
    
    # starting from closest idx, iterate through racetrack centerline points (i, i + 1, ...)
        # if the angle between the line formed between our current heading and the centerline piont
        # is greater than either of the max left/right thresholds, break

        # update maxIndex to be this index

        # at each centerline point, get the width left and width right of the boundary at that point
        # estimate the position of the left and right boundary points of this centerline point by
        # taking the line perpendicular to the line formed by the current index and previous index
        # (the width of the boundary should be the distance from the center point to each of those 
        # boundary points along this line)

        # take the angle between the heading and the line between our current position and the calculated right boundary point
        # and set max right threshhold to be the min between itself and the value
        # do the same for the max left threshold
    
    # set reference steering angle to be the angle between the current heading and the line bewteen
    # the current position of the car (x, y in state indices 1 and 0) and the point at maxIndex

    # now we want to calculate reference velocity
    # we want to reach highest velocity such that we can slow down at index j

    return np.array([0, 100]).T