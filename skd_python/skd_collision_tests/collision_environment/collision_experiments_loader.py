import sys,os
import numpy as np
import scipy.stats as st
import copy
import json
import time
import glob

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_collision_tests_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_collision_tests_dir)
if(skd_python_dir not in sys.path):
    sys.path.append(skd_python_dir)

# Controllers
import skd_collision_tests.controllers.car_controllers as car_controllers
import skd_collision_tests.controllers.pedestrian_controllers as pedestrian_controllers
import skd_collision_tests.collision_environment.collision_env_utils as collision_utils
import skd_collision_tests.collision_environment.collision_environment as collision_environment

# SKD Core Utils
import skd_core.skd_core_utils.skd_core_utils as skd_core_utils
import skd_trajectories.trajectories_filters as traj_filters


""""" Scenario classes to organize the different groups of simulations used """
class CollisionExperimentLoader:
    """ Object containing the necessary information to initialize an experiment """
    def __init__(self, output_dir, config_file):

        # Output dir
        self.output_dir = output_dir

        # Load experiments info from config file
        self.config_file_info = collision_utils.load_yaml_file(config_file)
        
        # Experiment info 
        self.max_num_steps = self.config_file_info["max_num_steps"]
        self.num_runs = self.config_file_info["num_runs"]
        self.controller_multipliers = self.config_file_info["multiplier_ids"]
        self.max_trajs_per_file = self.config_file_info["max_trajs_per_file"]
        
        # Pedestrian info
        self.safe_ped_traj_files = self.config_file_info["safe_trajectory_files"]
        # Info to create a car controllers
        self.car_controller_type = self.config_file_info["car_controller_type"]
        # Safe trajectories for scenario
        self.safe_trajectories = []

        # Create an environment for running experiments
        self.collision_env = collision_environment.CollisionEnvironment(output_dir)

         # Create outputdirs
        self.loader_summary_dir = self.output_dir + "/collision_experimens_summary"
        self.loader_summary_path = self.loader_summary_dir + "/experiments_summary.yaml"
        
        try:
            os.makedirs(self.loader_summary_dir)
        except OSError as error:
            print(error)


    def run_collision_experiments(self):
        # Experiment summaries
        experiments_summary = {}

        # Iterate over all controller multipliers
        for controller_id in self.controller_multipliers:

            # Store the controller id summaries
            controller_safe_traj_file_summaries = []

            # Iterate over all safe_ped_traj_files
            for safe_traj_filename in self.safe_ped_traj_files:

                safe_traj_file_summary = self.get_safe_traj_file_summary(controller_id, safe_traj_filename)

                # Store 
                controller_safe_traj_file_summaries.append(copy.deepcopy(safe_traj_file_summary))


            # Save a summary of experiments per controller multiplier
            controller_multiplier_summary = {"controller_multiplier" : controller_id, 
                                            "safe_traj_file_summaries" : copy.deepcopy(controller_safe_traj_file_summaries)
                                            }
            # Store controller summary
            experiments_summary[str(controller_id)] = copy.deepcopy(controller_multiplier_summary)
            
        # Save experiments summary and return summary
        skd_core_utils.save_dict_to_yaml(experiments_summary, self.loader_summary_path)



    def get_safe_traj_file_summary(self, controller_id, safe_traj_filename):
        # Store the output directories
        safe_traj_filename_log_dirs = []

        total_trajs_num = skd_core_utils.get_num_safe_trajs(safe_traj_filename)
        # Set the minimum trajs per file
        TRAJS_PER_FILE = min(self.max_num_steps, total_trajs_num)
        
        # Check if max trajs per file is considered
        if(TRAJS_PER_FILE <= 0):
            # Query the number of trajs in file
            TRAJS_PER_FILE = skd_core_utils.get_num_safe_trajs(safe_traj_filename)

        # Iterate for all trajectories considered in safe_traj_file
        for safe_traj_index in range(TRAJS_PER_FILE):
            # Set an output destination for the experiments log files
            safe_filename = os.path.basename(safe_traj_filename).split(".")[0]
            experiments_out_dir = self.output_dir 
            experiments_out_dir += "/controller_m_%s/%s/ST_%d" % (controller_id, safe_filename, safe_traj_index)

            # Make experiments
            try:
                os.makedirs(experiments_out_dir)
            except:
                pass

            # Perform collision experiments for self.num_runs tries
            for run_num in range(self.num_runs):
                self.run_single_experiment(controller_id, safe_traj_filename, 
                        safe_traj_index, run_num, experiments_out_dir)

            safe_traj_filename_log_dirs.append(experiments_out_dir)


        # Store a summary of the safe_traj_file
        safe_traj_file_summary = {"safe_traj_filepath" : safe_traj_filename,
                                "safe_traj_file_log_dirs" : safe_traj_filename_log_dirs}

        return safe_traj_file_summary

    

    def run_single_experiment(self, controller_id, safe_traj_filename, safe_traj_index, run_number, exp_outdir):
        """ Runs a single colllision experiment using the 
        collision env associated with the loader """

        LONGIT_INDEX = 0
        HOZ_INDEX = 1

        # Create a starting ped_controller
        run_safe_traj = skd_core_utils.get_safe_traj_from_file(safe_traj_filename, safe_traj_index)
        run_pedestrian = pedestrian_controllers.PedestrianController(run_safe_traj)
        
        # Create a default basic car controller
        run_car_controller = car_controllers.BasicCarController(controller_id, multiplier=float(controller_id))

        # Change starting position according to safe traj
        car_start_pos = traj_filters.get_car_starting_pos(run_safe_traj, run_car_controller)
        run_car_controller.set_car_pos(car_start_pos[LONGIT_INDEX], car_start_pos[HOZ_INDEX])

        # Run the experiment 
        self.collision_env.run_single_collision_experiment(controller_id, run_pedestrian, run_car_controller, 
                                    exp_outdir, run_number)


    def get_experiment_max_num_steps(self):
        """ Max number of steps per each collision experiment run """
        return self.max_num_steps


    def get_experiment_num_runs(self):
        return self.num_runs


    def get_summary_file_path(self):
        return self.loader_summary_path

        






if __name__ == '__main__':
    main()
