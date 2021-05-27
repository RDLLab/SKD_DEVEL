import sys,os
import copy
import json
from datetime import datetime
import numpy as np

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_collision_tests_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_collision_tests_dir)
# Append top level library
if(skd_python_dir not in sys.path):
    sys.path.append(skd_python_dir)


# Utils
import skd_core.skd_core_utils.skd_core_utils as skd_core_utils
import skd_trajectories.trajectories_filters as trajs_filters



""" Class to represent a simulation environment. This class serves as a testbed to experiment with 
the collision success rate of a particular controller multiplier against a given pedestrain safe trajectory """
class CollisionEnvironment:
    """ Constructor"""
    # Size of state space
    STATE_SPACE_SIZE=6

    # State space variables indices
    STATE_PED_LONG=0
    STATE_PED_HOZ=1
    STATE_VEH_LONG=2
    STATE_VEH_HOZ=3
    STATE_VEH_SPEED=4
    STATE_VEH_INTENTION=5

    # Constructor of the class
    def __init__(self, output_dir):
        # The logs of the successful experiments
        self.run_success_log = []
        # The logs of the multipliers
        self.multiplier_log = []
        # Entries for all the runs
        self.run_entries = []
        # Output dir
        self.env_outdir =  output_dir 
        
        try:
            # Create outdir
            os.makedirs(self.env_outdir)
        except OSError as error:
            print(error) 
    
    
    # Return output directory
    def get_env_outdir(self):
        return self.env_outdir



    # Start experiment of the environment simulation
    def run_single_collision_experiment(self, controller_id, starting_ped_controller, 
                                    starting_car_controller, experiment_out_dir, run_number, max_num_steps=25):
        LONG_INDEX = 0
        HOZ_INDEX = 1
        OUT_OF_REACH_THRESH = 5
        COLLISION_SEGMENTS_CHECK = 5

        """ Need to add a module that plots and interprets the statistics of the collision expriments """

        # Step count
        step_count = 0
        # Logs the entries of each step
        step_entries = []
        

        # Initiate the controllers
        run_ped_controller = starting_ped_controller
        run_car_controller = starting_car_controller

        # Check if the car is in collision
        # Start with terminal set to collision result at init state
        collision_flag = run_car_controller.collides(run_ped_controller)

        # Run until terminal
        while (step_count < max_num_steps):

            # Step starting states
            ped_step_start_pos = run_ped_controller.get_current_pos()
            car_step_start_pos = run_car_controller.get_current_pos()

            # Append state for logging
            step_entries.append(self.get_env_state(run_ped_controller, run_car_controller))
            # Advance the state of the car controller
            run_car_controller.advance_car_state(run_ped_controller)
            
            # Advance the state of the pedestrian and check if there is a collision
            run_ped_controller.advance_step()

            # Check if out of reach
            ped_step_end_pos = run_ped_controller.get_current_pos()
            car_step_end_pos = run_car_controller.get_current_pos()


            # # Check for terminal conditions
            # out_of_reach = (car_step_end_pos[LONG_INDEX] - ped_step_end_pos[LONG_INDEX]) > OUT_OF_REACH_THRESH

            # Check for collision in this step
            #collision_flag = run_car_controller.collides(run_ped_controller)
            if(not collision_flag):
                collision_flag = self.check_intermediate_collision(ped_step_start_pos, ped_step_end_pos, 
                    car_step_start_pos, car_step_end_pos, run_car_controller, run_ped_controller, COLLISION_SEGMENTS_CHECK)

            # Update step count
            step_count += 1

        # Append the last state
        step_entries.append(self.get_env_state(run_ped_controller, run_car_controller))

        # Append collision flags and multiplier
        self.run_success_log.append(collision_flag)
        self.multiplier_log.append(controller_id)

        # Save entry
        self.run_entries.append(copy.deepcopy(step_entries))
        self.serialize_run(copy.deepcopy(step_entries), run_number, experiment_out_dir, collision_flag, run_car_controller.get_car_dimensions())




    """ Function for checking collisions occuring in the discretized movement in between steps from the agents """
    def check_intermediate_collision(self, ped_step_start_pos, ped_step_end_pos, 
        car_step_start_pos, car_step_end_pos, car_controller, ped_controller, num_steps_check):
        # Interpolate intermediate positions between start and stop
        ped_positions = np.linspace(ped_step_start_pos, ped_step_end_pos, num_steps_check).tolist()
        car_positions = np.linspace(car_step_start_pos, car_step_end_pos, num_steps_check).tolist()

        # Deep copy controllers to use as collision check actors only ("Dont want to actually move them")
        collision_car_controller = copy.deepcopy(car_controller)
        collision_ped_controller = copy.deepcopy(ped_controller)


        # # Test that the controllers are independent after collision check
        for discretized_step in range(num_steps_check):
            step_ped_pos = ped_positions[discretized_step]
            step_car_pos = car_positions[discretized_step]
            
            # Set agents to positions
            collision_car_controller.set_car_pos(step_car_pos[0], step_car_pos[1])
            collision_ped_controller.set_ped_position(step_ped_pos[0], step_ped_pos[1])

            if(collision_car_controller.collides(collision_ped_controller)):
                """ Return true if there was a collision in between """
                return True
     

        return False




    """ Outputs information to the terminal about the current state of the environment, including the position of the 
    controllers in it"""
    def print_env_state(self, run_ped_controller, run_car_controller):
        print("PED POS")
        print(run_ped_controller.get_current_pos())
        print("CAR_POS")
        print(run_car_controller.get_car_state())

    """ queries for the current state of the simulation environment """
    def get_env_state(self, ped_controller, car_controller):
        current_state = []
        current_state.extend(ped_controller.get_current_pos())
        current_state.extend(car_controller.get_car_state())
        return current_state

    # Get the log info of the experiment environment
    def get_logs(self):
        return self.run_success_log, self.multiplier_log

    # Get the success log of the ecperiment environment
    def get_success_log(self):
        return self.run_success_log

    # Get the success statistics of the experiment environment
    def get_success_statistics(self):
        failures = 0
        for attempt in self.run_success_log:
            if (attempt == True):
                failures += 1

        # Summarize statistics
        total_runs = len(self.run_success_log)
        total_failures = failures
        total_success = total_runs - total_failures

        return [total_runs, total_failures, total_success]


    # Log the outout into a yaml
    def serialize_run(self, run_entry, run_number, outdir, status, dimensions):
        outfile = outdir + "/run_%d.yaml" % (run_number)
        output_map = {"DATA_LOG" : run_entry, "COLLIDED" : status, "CAR_DIMENSIONS" : copy.deepcopy(dimensions)}
        skd_core_utils.save_dict_to_yaml(output_map, outfile)






if __name__ == '__main__':
    main()
