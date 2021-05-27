import os, sys

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_collision_tests_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_collision_tests_dir)
if(skd_python_dir not in sys.path):
    sys.path.append(skd_python_dir)
    


import numpy as np
import copy


################################# GLOBAL FUNCTIONS ##################################
""" Clamps a value """
def clamp(value, min_val, max_val):
    # Assert parameters are right
    func_min = min_val
    func_max = max_val

    # Swap in case order was reversed
    if(func_min > func_max):
        func_max = min_val
        func_min = max_val

    # Check which to return 
    if (value < func_min):
        return func_min
    elif (value > func_max):
        return func_max

    # Value is not capped. Return as it is
    return value


###################################### CLASS DEFINITIONS ####################################
class PedestrianController:
    SAFE_LONGIT_POS = 0
    SAFE_HOZ_POS = 1

    """ Constructor of the pedestrian class """
    def __init__(self, safe_ped_traj):
        # Assert safe pedestrian trajectory exists
        assert(len(safe_ped_traj) > 0), "No safe pedestrian trajectory associated"
        self.safe_ped_traj = copy.deepcopy(safe_ped_traj)

        # Pedestrian internals
        self.radius = 0.34
        self.height = 1.86
        
        # Record of the controller simulation_time_step initialize to 0
        self.time_step = 0

        # Initiate the controller to the starting point of the ped
        starting_point = copy.deepcopy(safe_ped_traj[0])
    
    
        self.longit_pos = starting_point[self.SAFE_LONGIT_POS]
        self.hoz_pos = starting_point[self.SAFE_HOZ_POS]


    """ Overwrites the starting position of the pedestrian controller """
    def set_ped_position(self, ped_longit, ped_hoz):
        # Reset initial position to position given (ped_longit, ped_hoz)
        self.longit_pos = ped_longit
        self.hoz_pos = ped_hoz

        # Print update
        #print("Pedestrian position set to [%f, %f]" % (self.longit_pos, self.hoz_pos))

    """ Updates one step of the pedestrian """
    def advance_step(self):
        # Increase time step
        self.time_step += 1

        # Move the pedestrian one step in the safe trajectory
        # Update pos
        if(self.time_step < len(self.safe_ped_traj)):
            current_safe_ped_point = copy.deepcopy(self.safe_ped_traj[self.time_step])
        else:
            current_safe_ped_point = copy.deepcopy(self.safe_ped_traj[-1])
        self.longit_pos = current_safe_ped_point[self.SAFE_LONGIT_POS]
        self.hoz_pos = current_safe_ped_point[self.SAFE_HOZ_POS]

       
    """ Get current position """
    def get_current_pos(self):
        return [self.longit_pos, self.hoz_pos]

    """ Get dimensions info """
    def get_dimensions(self):
        return [self.radius, self.height]