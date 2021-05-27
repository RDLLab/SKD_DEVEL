import os, sys

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_trajectories_dir = os.path.dirname(source_path)
skd_python_dir = os.path.dirname(skd_trajectories_dir)

if(skd_python_dir not in sys.path):
    sys.path.append(skd_python_dir)


# Import local libraries
import skd_collision_tests.controllers.pedestrian_controllers as pedestrian_controllers

# Import third party libs
import numpy as np
import copy


""" Get the trajectory's min pos location """
def get_traj_min_longitudinal_point_index(safe_traj):
    LONGIT_INDEX = 0
    HOZ_INDEX = 1

    min_longit = 100000
    min_point_index = 0
    # Iterate over all points and save index of the lowerst
    for point_index in range(len(safe_traj)):
        # Longitudinal point
        point_longit = safe_traj[point_index][LONGIT_INDEX]

        # Check if this is new min point
        if(point_longit < min_longit):
            min_longit =  point_longit
            min_point_index = point_index

    return min_point_index



def is_trajectory_valid(safe_trajectory):
    """ Returns if a given trajectory is valid for the assessment of the car """
    return true


def filter_trajectories(trajectories_db):
    """ Checks if each of the given trajectories are valid against the car's mechanical criteria """
    filtered_trajectories = []

    # Filter valid ones
    for trajectory in filter_trajectories:
        if(self.is_trajectory_valid(trajectory)):
            filtered_trajectories.append(trajectory)

    # Return all accepted trajectories
    return filtered_trajectories



def get_car_starting_pos(safe_trajectory, car_controller):
    """ Calculates the car_starting_pos based on the an offset of the safe trajectory"""

    # MAX C for now 
    C = 1.5
    # Calculate stopping distance for this controller
    car_stop_dist = car_controller.get_unit_stop_dist() * C
    """ Examines the safe trajectory and outputs a starting location for the car controller with the specified
    kinematics parameters. The output starting location should be such that the "unit controller c = 1, can interact with the pedestrian."""
    safe_traj_min_point_index = get_traj_min_longitudinal_point_index(safe_trajectory)
    safe_traj_min_point = safe_trajectory[safe_traj_min_point_index]

    return [safe_traj_min_point[0] - car_stop_dist, -2]

