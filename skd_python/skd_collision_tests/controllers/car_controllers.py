import os, sys

# Import parent dir
# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_collision_tests_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_collision_tests_dir)
if(skd_python_dir not in sys.path):
    sys.path.append(skd_python_dir)

# Import local libraries
import skd_collision_tests.controllers.pedestrian_controllers as pedestrian_controllers

# Import third party libs
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



""" Models a basic car controller """
class BasicCarController:

    # CAR STATE INDICES
    LONGIT_INDEX = 0
    HOZ_INDEX = 1
    CAR_SPEED_INDEX = 2
    CAR_ACC_INDEX = 3
    CAR_INTENTION_INDEX = 4

    # CAR EXPERIMENTING ADDITIONAL VARIABLES
    CAR_ACCEL_RATE = 1.25 # Car acceleration from stop is assumed to be constant 1.25m/s^2
    SIMULATION_STEP_TIME = 0.3
    """ Constructor of the car class """
    def __init__(self, car_longit_start=100.0, car_horizontal_start=-2.0, max_speed=8.33, braking_rate=-3.5, multiplier=1.0, car_dims = [4.66, 1.68]):
         # Car's internal information
        self.car_length = car_dims[0]
        self.car_width = car_dims[1]

        # Car's state information
        self.longit_pos = car_longit_start
        self.hoz_pos = car_horizontal_start

        # Initialize car to driving and constant velocity
        self.car_acc = 0.0
        self.braking = False
        self.throttle_rate = 1.25
        self.car_vel = max_speed

        # Car mechanical information
        self.car_max_speed = max_speed
        self.braking_rate = braking_rate
        self.multiplier = float(multiplier)

        # Calculate stopping distance for this controller
        self.stop_time = abs(self.car_max_speed / self.braking_rate)
        # Stopping distance of the car based on braking rate
        self.unit_stop_dist = (self.car_max_speed * self.stop_time) + (0.5 * self.braking_rate 
            * self.stop_time * self.stop_time)
        # Add padding to the unit stopping distance "Kappa"
        self.unit_stop_dist += (self.car_length/2) + (self.car_max_speed/2 * self.SIMULATION_STEP_TIME)
        # Stopping distance of the car based on braking rate and multiplier
        self.car_stopping_distance = self.unit_stop_dist * self.multiplier

        # print("CAR USED:")
        # print("Kappa = %f" % (self.unit_stop_dist))
        # print("Multiplier = %f" % (self.multiplier))
        # print("C * Kappa = %f" % (self.car_stopping_distance))
        # print("CAR STOPPING_TIME = %f" % (self.stop_time))
        
        

    """ get dimensions """
    def get_car_dimensions(self):
        return [self.car_length, self.car_width]

    """ Getters """
    def get_current_pos(self):
        return [self.longit_pos, self.hoz_pos]

    def get_car_state(self):
        """ Gets the car state information in the format [longit_pos, hoz_pos, car_vel, car_acc, braking_intention]"""
        return [self.longit_pos, self.hoz_pos, self.car_vel, self.car_acc, self.braking]

    def get_unit_stop_dist(self):
        """ Returns the unitary stopping distance of the basic controller """
        return self.unit_stop_dist


    def get_car_stopping_distance(self):
        """ Returns the threshold value in meters at which the car starts to decelerate for an obstacle"""
        return self.car_stopping_distance


    def get_car_max_speed(self):
        """ Returns the max speed of the car """
        return self.max_speed


    def get_car_braking_rate(self):
        """ Returns the braking rate of the car """
        return self.braking_rate


    def get_car_stop_time(self):
        """ Returns the estimated time in seconds from the car to go from braking to full stop """
        return self.stop_time
        

    def is_braking(self):
        """ Checks if the car is in braking status """
        return self.braking


    def set_car_pos(self, start_pos_longit, start_pos_hoz):
        """ Sets the car to a position """
        self.longit_pos = start_pos_longit
        self.hoz_pos = start_pos_hoz

    
    """" Prints out the debug info of the car """
    def print_debug_info(self):
        print("STOPPING_TIME")
        print(self.stop_time)
        print("UNIT_STOP_DIST (KAPPA)")
        print(self.stop_dist)
        print("MULTIPLIER")
        print(self.multiplier)
        print("CONTROLLER_STOPPING_DIST")
        print(self.car_stopping_distance)



    """Method to update the state information of the car. Only this method 
    can be used to update the information of the car"""
    def advance_car_state(self, pedestrian_controller):
        # Get the pedestrian's info (perfect observation)
        ped_dimensions =  pedestrian_controller.get_dimensions()
        ped_position = pedestrian_controller.get_current_pos()

        # Get current car state
        current_car_state = self.get_car_state()

        # Compute relative distance between ped and car controllers
        rel_ped_from_car = np.subtract(pedestrian_controller.get_current_pos(), self.get_current_pos())

        # Check is brakes are to be applied
        self.braking = self.need_to_brake(rel_ped_from_car)


        # Update car state
        # Check for change to braking
        if (self.braking):
            # Add error to the braking rate of the car controllers
            acc_error = np.random.uniform(-0.1 * self.braking_rate, 0.1 * self.braking_rate, 1)
            self.car_acc = self.braking_rate + float(acc_error)
            # Update velocity based on braking rate
            self.car_vel =  clamp(self.car_vel + (self.car_acc *  self.SIMULATION_STEP_TIME), 0, self.car_max_speed)
        else:
            # Car is driving at constant speed
            self.car_acc = 0
            self.car_vel = self.car_max_speed


        # If velocity is down to zero, stop decelerating
        # if (self.car_vel <= 0):
        #     self.car_acc = 0

        # Update the speed parameter of the car with some error
        speed_error = np.random.uniform(-0.05 * self.car_max_speed, 0.05 * self.car_max_speed)
        self.car_vel = clamp(self.car_vel + speed_error, 0, self.car_max_speed)


        # Update longitudinal position according to (vot + 1/2(a)(t^2)
        step_displacement = (current_car_state[self.CAR_SPEED_INDEX] * self.SIMULATION_STEP_TIME) 
        + (0.5 * self.car_acc * self.SIMULATION_STEP_TIME * self.SIMULATION_STEP_TIME)
       
        # Clamp displacement to avoid negative displacement
        if (step_displacement < 0):
            print("negative displacement")
            print(step_displacement)
        step_displacement = clamp(step_displacement, 0, self.car_max_speed * self.SIMULATION_STEP_TIME)


        self.longit_pos += step_displacement


    """ Method to query if the car should change to "is_braking = True" mode, based on the 
    stopping distance assigned to the car """
    def need_to_brake(self, rel_ped_to_car):
        # Start breaking if condition is met
        if (rel_ped_to_car[self.LONGIT_INDEX]) <= self.car_stopping_distance:
            return True

        # Otherwise, return false
        return False


    """ Method to check if car collided with a particular pedestrian controller """
    def collides(self, pedestrian_controller):
        # PED INFO
        RADIUS_INDEX = 0

        # Compute the difference vector between the centers
        ped_loc = pedestrian_controller.get_current_pos()
        car_loc = self.get_current_pos()
        rel_ped_to_car = np.subtract(ped_loc, car_loc)

        # Compute closest point to the circle from the center of the rectangle
        closest_car_longit = clamp(car_loc[self.LONGIT_INDEX] + rel_ped_to_car[self.LONGIT_INDEX],
                                   car_loc[self.LONGIT_INDEX] - (self.car_length / 2.0),
                                   car_loc[self.LONGIT_INDEX] + (self.car_length / 2.0))

        closest_car_hoz = clamp(car_loc[self.HOZ_INDEX] + rel_ped_to_car[self.HOZ_INDEX],
                                car_loc[self.HOZ_INDEX] - (self.car_width / 2.0),
                                car_loc[self.HOZ_INDEX] + (self.car_width / 2.0))

        closest_point = np.array([closest_car_longit, closest_car_hoz])

        # Compute distance between closest point and center of circle
        diff_vec = np.subtract(closest_point, ped_loc)
        distance = np.linalg.norm(diff_vec)

        # Check if distance is greater than radius of pedestrian circle
        ped_dimensions = pedestrian_controller.get_dimensions()

        if (distance <= ped_dimensions[RADIUS_INDEX]):
            return True

        # Otherwise, return False
        return False


