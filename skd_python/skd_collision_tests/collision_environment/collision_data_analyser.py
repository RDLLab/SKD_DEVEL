import sys,os
import numpy as np
import scipy.stats as st

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_collision_tests_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_collision_tests_dir)
sys.path.append(skd_python_dir)

import skd_collision_tests.controllers.car_controllers as car_controllers
import skd_collision_tests.controllers.pedestrian_controllers as pedestrian_controllers
import skd_collision_tests.collision_environment.collision_env_utils as collision_utils
import skd_core.skd_core_utils.skd_core_utils as skd_core_utils

import copy
import json
import time
import glob


class CollisionExperimentDataAnalyser:
    """ Class to analyse and plot the output info of the collision environments """
    def __init__(self, parsing_summary_file, outputdir):
        # Save the top level dir where parsing occurs
        self.parsin_summary_file = parsing_summary_file

        # Location where the analyser outputs
        self.outputdir = outputdir

        # Load the simmary data
        self.parsing_summary_data  = skd_core_utils.load_dict_from_yaml(self.parsin_summary_file)

        # Output directory is created at an outer level to avoid errors of recreating the dir inside 

    def get_analyzer_summary_statistics(self):
        # Database to store data
        summary_data_db = []

        # Iterate over the controller multiplier summaries in the summary file
        for controller_id in self.parsing_summary_data:

            # Load the controler summary
            controller_summary = self.parsing_summary_data[str(controller_id)]
            # Load the top level controller summary
            controller_summary_data = self.parse_controller_summary(controller_id, controller_summary)

            # Save summary for each controller
            summary_data_db.append(controller_summary_data)

        
        # Add headers to database
        np_summary_data_db = np.array(summary_data_db)
        # Get first row to check headers
        first_summary = np_summary_data_db[0]
        # Length of row data including final six statistics and controller id row
        row_data_len = len(first_summary)
        # number of statistics variable
        stats_len = 6
        
        # Add headers per safe_traj_processed. Subract one index for controller id
        header = "controller_id"
        for header_index in range(0, row_data_len - stats_len - 1):
            header += ",ST_%d" % (header_index)

        # Add the stats for header
        header += ",row_data_size,row_data_sum,row_data_mean,row_data_var,row_data_ci_low,row_data_ci_high"

        # Save data to a csv file
        np.savetxt("%s/experiments_statistics.csv" % (self.outputdir), np_summary_data_db, delimiter=",", header=header, comments='')



    def parse_controller_summary(self, controller_id, controller_summary):
        """ Parses through the experiment summary data of a controller summary, and returns
        the experimental statistics associated with the runs in the summary file """

         # Collect all statistic summaries from controller multiplier
        controller_id_data = []

        
        # Iterate over the safe trajectories file summaries
        for safe_traj_file_summary in controller_summary["safe_traj_file_summaries"]:
           
            # Process summary record
            safe_traj_file_data= self.process_safe_traj_file_summary(
                                                            controller_id, safe_traj_file_summary)

            # Save stats
            controller_id_data.append(copy.deepcopy(safe_traj_file_data))


        # Append extra data
        safe_traj_file_collision_rates = []
        safe_traj_file_collision_dirs = []

        for safe_traj_file_data in controller_id_data:
            for safe_traj_file_collision_dir in safe_traj_file_data: 
                # Append collision rates for specific safe trajctory
                single_safe_traj_data = safe_traj_file_data[safe_traj_file_collision_dir]
                safe_traj_file_collision_rates.append(single_safe_traj_data["COLLISION_RATE"])
                safe_traj_file_collision_dirs.append(safe_traj_file_collision_dir)



        # Process controller id data to return statistics in a row fashion
        # start with id tag
        controller_row_entry = [float(controller_id)]
        controller_row_entry.extend(safe_traj_file_collision_rates)
        # Append statistics for collision rates
        controller_row_entry.extend(skd_core_utils.process_general_stats_array(safe_traj_file_collision_rates))

        # Pack headers 

        return controller_row_entry





    def process_safe_traj_file_summary(self, controller_id, safe_traj_file_summary):

        """ Data parsing at the safe traj file level """
        # Grab the information from each traj file
        safe_traj_filepath = safe_traj_file_summary["safe_traj_filepath"]
        safe_traj_file_collision_dirs = safe_traj_file_summary["safe_traj_file_log_dirs"]

        # Collect all single trajectories stats
        safe_traj_file_data = {}
        
        """ Process each safe_trajectory inside the safe trajectory file (safe_file[safe_traj_index]) """
        for safe_traj_dir in safe_traj_file_collision_dirs:
           
            # Adhere output to be of same format as output of kamikaze traj gne
            safe_traj_dirname = os.path.basename(safe_traj_dir)
            # Grab safe trajectory index from dirname
            safe_traj_index = int(safe_traj_dirname.split("_")[-1])

            # Process single safe_trajectory dir
            single_traj_data = self.process_single_safe_trajectory(
                safe_traj_dir, safe_traj_filepath, controller_id, safe_traj_index)

            # Plot as well
            self.plot_single_safe_trajectory(safe_traj_dir, controller_id, safe_traj_index, max_plots = 10)

            # Save stats
            safe_traj_file_data[safe_traj_dir] = copy.deepcopy(single_traj_data)


        return safe_traj_file_data



    def process_single_safe_trajectory(self, safe_traj_dir, safe_traj_filepath,
         controller_id, safe_traj_index): 

        # Parse data from all the data files in the single safe trajectory
        runs_suffix = ".yaml"
        traj_run_files = glob.glob(safe_traj_dir + "/*%s" % (runs_suffix))
        num_run_files = len(traj_run_files)

        # Storeage for collided flags
        safe_traj_runs_collided = []
        
        # Parse all the statistics per run
        for run_num in range(num_run_files):
            # Read from each data and save
            run_file = safe_traj_dir + "/run_%d.yaml" % (run_num)
            data = skd_core_utils.load_dict_from_yaml(run_file)
            collided = data["COLLIDED"]
            # Append run collision status
            safe_traj_runs_collided.append(collided)

        # Pack the single safe traj results in a summary
        total_collided = np.count_nonzero(safe_traj_runs_collided)
        collision_rate = (total_collided/num_run_files) if (num_run_files > 0) else 0
        single_safe_traj_data = {"TOTAL_COLLIDED" : total_collided,
                                    "TOTAL_ATTEMPTS" : num_run_files,
                                    "COLLISION_RATE" : collision_rate,
                                    "SAFE_TRAJ_FILENAME" : safe_traj_filepath,
                                    "SAFE_TRAJ_INDEX" : safe_traj_index,
                                    "CONTROLLER_ID" : controller_id}


        return single_safe_traj_data




    def plot_single_safe_trajectory(self, safe_traj_dir, controller_id, safe_traj_index, max_plots=-1): 

        safe_traj_file_keyname = os.path.basename(os.path.dirname(safe_traj_dir))

        safe_traj_plotdir = self.outputdir + "/%s/plots" % (skd_core_utils.get_kamikaze_config_suffix(
                                                    controller_id, safe_traj_file_keyname, safe_traj_index))

        # Make directory
        try:
            os.makedirs(safe_traj_plotdir)
        except OSError as error:
            print(error)

        # Parse data from all the data files in the single safe trajectory
        runs_suffix = ".yaml"
        traj_run_files = glob.glob(safe_traj_dir + "/*%s" % (runs_suffix))
        num_run_files = len(traj_run_files)

    

        # Set cap on number of plots
        NUM_PLOTS = min(max_plots, num_run_files)
        if(NUM_PLOTS < 0):
            NUM_PLOTS = num_run_files
        
        # Parse all the statistics per run
        for run_num in range(NUM_PLOTS):
            # Read from each data and save
            run_file = safe_traj_dir +  "/run_%d.yaml" % (run_num)
            run_data = skd_core_utils.load_dict_from_yaml(run_file)
            states_data = run_data["DATA_LOG"]
            collided = run_data["COLLIDED"]

            # Append run collision status
            skd_core_utils.save_plot_number(run_num, states_data, safe_traj_plotdir, collided)

       