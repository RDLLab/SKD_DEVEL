import sys, os
import numpy as np
import random
import time
import json, copy
import glob
import matplotlib.pyplot as plt

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_core_dir = os.path.dirname(os.path.dirname(source_path))
if(skd_core_dir not in sys.path):
	sys.path.append(skd_core_dir)


# Import analysers
import skd_core_analysers.oppt_log_analyser as oppt_log_analyser
import skd_core_utils.skd_core_utils as skd_core_utils
import skd_core_metrics.Fretchet as Fretchet


class SKDKamikazeDataAnalyser:
    """ Class to analyse and plot the output info of the collision environments """
    def __init__(self, parsing_summary_file, outputdir):
        # Save the top level dir where parsing occurs
        self.parsing_summary_file = parsing_summary_file

        # Location where the analyser outputs
        self.outputdir = outputdir

        self.parsing_summary_data = skd_core_utils.load_dict_from_yaml(self.parsing_summary_file)
      


    def parse_summary_data(self):
        # Summary data array
        summary_data_array = []

        # Iterate over the controller multiplier summary reposrts
        for controller_id in self.parsing_summary_data:

            # Controller summary
            controller_summary = self.parsing_summary_data[controller_id]

            controller_row_data = self.parse_controller_summary(controller_id, controller_summary)

            # Save controller data to summary
            summary_data_array.append(controller_row_data)


        # Save summary data as a table with headers
        summary_data_headers = "controller_id"
        summary_data_headers += ",frechet_sample_size,fretchet_sum,frechet_means,fretchet_var,frechet_95_ci_low_,frechet_95_ci_high"
        summary_data_headers += ",timings_sample_size,fretchet_sum,timings_means,fretchet_var,timings_95_ci_low_,timings_95_ci_high"

        # Use numpy to save array
        np_summary_data = np.array(summary_data_array)
        summary_outfile_path = self.outputdir + "/summary_statistis.csv"

        # Save as csv
        np.savetxt(summary_outfile_path, np_summary_data, delimiter=",", header=summary_data_headers, comments='')



    def parse_controller_summary(self, controller_id, controller_summary):
        # Collect all statistic summaries from controller multiplier
        controller_multiplier_frechet_stats = []
        controller_multiplier_timing_stats = []
        
        # Iterate over the safe trajectories file summaries
        for safe_traj_file_summary in controller_summary["safe_traj_file_summaries"]:
           
            # Process summary record
            traj_file_fretchet_stats, traj_file_timing_stats = self.process_safe_traj_file_summary(
                                                            controller_id, safe_traj_file_summary)

            # Save stats
            controller_multiplier_frechet_stats.append(copy.deepcopy(traj_file_fretchet_stats))
            controller_multiplier_timing_stats.append(copy.deepcopy(traj_file_timing_stats))

        # Process top level statistics here
        controller_fretchet_distances = []
        controller_fretchet_timings = []

        # Gather all the controller distances statistics
        for frechet_stat in controller_multiplier_frechet_stats:    
            # Accumulate all distance stats
            for traj_file_stat_record in frechet_stat:
                # add all the distances in each record
                record_distances = traj_file_stat_record["DATA_ARRAY"] 
                controller_fretchet_distances.extend(copy.deepcopy(record_distances))


        # Gather all the controller timings statistics
        for timing_stat in controller_multiplier_timing_stats:   
            # Accumulate all timing stats
            for traj_file_timing_record in timing_stat:
                # add all the distances in each record
                record_timings = traj_file_timing_record["DATA_ARRAY"] 
                controller_fretchet_timings.extend(record_timings)


        controller_summary_data = [float(controller_id)]
        controller_summary_data.extend(copy.deepcopy(skd_core_utils.process_general_stats_array(controller_fretchet_distances)))
        controller_summary_data.extend(copy.deepcopy(skd_core_utils.process_general_stats_array(controller_fretchet_timings)))

        return controller_summary_data


    


    def process_safe_traj_file_summary(self, controller_id, safe_traj_file_summary):

        """ Data parsing at the safe traj file level """
        # Grab the information from each traj file
        safe_traj_filepath = safe_traj_file_summary["safe_traj_filepath"]
        safe_traj_dirs = safe_traj_file_summary["safe_traj_file_log_dirs"]

        # Collect all single trajectories stats
        safe_traj_file_fretchet_stats = []
        safe_traj_file_timings_stats = []
        
        """ Process each safe_trajectory inside the safe trajectory file (safe_file[safe_traj_index]) """
        for safe_traj_dir in safe_traj_dirs:
           
            # Adhere output to be of same format as output of kamikaze traj gne
            safe_traj_file_keyname = os.path.basename(os.path.dirname(safe_traj_dir))
            safe_traj_dirname = os.path.basename(safe_traj_dir)
            # Grab safe trajectory index from dirname
            safe_traj_index = int(safe_traj_dirname.split("_")[-1])

            # Process single safe_trajectory dir
            single_traj_fretcht_stats, single_traj_timings_stats = self.process_single_safe_trajectory(
                safe_traj_dir, safe_traj_filepath, controller_id, safe_traj_file_keyname, safe_traj_index)

            # Save stats
            safe_traj_file_fretchet_stats.append(copy.deepcopy(single_traj_fretcht_stats))
            safe_traj_file_timings_stats.append(copy.deepcopy(single_traj_timings_stats))


        return safe_traj_file_fretchet_stats, safe_traj_file_timings_stats





    def process_single_safe_trajectory(self, safe_traj_dir, safe_traj_filepath,
         controller_id, safe_traj_file_keyname, safe_traj_index, augmented=True):  
        # Match controller multiplier hierarchy naming match
        safe_traj_outdir = self.outputdir + "/%s" % (skd_core_utils.get_kamikaze_config_suffix(
                                                    controller_id, safe_traj_file_keyname, safe_traj_index))

        # Retrive analysers associated with logfiles
        safe_traj_file_analysers = self.get_safe_traj_file_analysers(safe_traj_dir, safe_traj_outdir)

        # Get all successful data records associated with the particular safe trajectory (id by safe_traj_file and index)
        safe_traj_data_records = []

        

        # Examine analysers
        for analyser in safe_traj_file_analysers:
            sucessful_ped_trajs, successful_veh_trajs = analyser.get_successful_ped_veh_trajectories()

            # Create trajectory data pairs
            data_records = self.get_traj_data_records(sucessful_ped_trajs, successful_veh_trajs,
                                     safe_traj_filepath, safe_traj_index)

            safe_traj_data_records.extend(copy.deepcopy(data_records))


        # Examine single safe trajectory data records
        #print("COMPUTE STATS HEADER")
        records_fretchet_dists, records_fretchet_times = self.compute_traj_records_fretchet_stats(
                                        safe_traj_data_records, safe_traj_outdir)
        #print("COMPUTE STATS END")

        # Record stats
        single_traj_fretcht_stats = skd_core_utils.process_general_stats(copy.deepcopy(records_fretchet_dists))
        single_traj_timings_stats = skd_core_utils.process_general_stats(copy.deepcopy(records_fretchet_times))

        return single_traj_fretcht_stats, single_traj_timings_stats

        


    def compute_traj_records_fretchet_stats(self, traj_records, outputdir, augmented=True, save_plots=True):
        """ Takes as input trajectory records (Trajectory Data Record), and computes
        the fretchet distances and fretchet timings associated with the records """
        safe_traj_fretchet_distances = []
        safe_traj_fretchet_timings = []

        
        # Compute fretchet distances and timing for computations
        for record_index in range(len(traj_records)):
            record = traj_records[record_index]
            # Record time 
            t_single_calc_start = time.clock()
            record_frechet = 0
            # Check for augmented path computation
            if(augmented):
                augmented_traj = self.get_augmented_trajectory(record.get_kamikaze_ped_traj(), record.get_safe_traj())
                record_frechet = Fretchet.frechetDist(augmented_traj, record.get_safe_traj())
                # Save fretchet distances
                safe_traj_fretchet_distances.append(record_frechet)
            else:
                # Save non-augmented fretchet dist
                record_frechet = Fretchet.frechet(record.get_kamikaze_ped_traj()), record.get_safe_traj()
                safe_traj_fretchet_distances.append(record_frechet)

            t_single_calc_end = time.clock()
            # Save timing
            safe_traj_fretchet_timings.append((t_single_calc_end - t_single_calc_start) * 1000)

            # Check for plotting
            if(save_plots):
                plot_output_dir = outputdir + "/plots"

                try:
                    os.makedirs(plot_output_dir)
                except OSError as error:
                    # Can use to debug by printing number of times dirs are attempted to be created
                    # print(error)
                    pass 

                plot_title =  "SKD Interaction experiment number %d, Estimated SKD = %f" % (record_index, record_frechet)
                skd_core_utils.save_trajectories_plot(record_index, record.get_safe_traj(), record.get_kamikaze_ped_traj(),
                                     record.get_kamikaze_veh_traj(), plot_title, plot_output_dir) 

        return safe_traj_fretchet_distances, safe_traj_fretchet_timings




    def get_traj_data_records(self, ped_trajectories, veh_trajectories, safe_trajectory_filepath, safe_traj_index):
        data_records = []

        # Check for right input
        uneven_len_err_msg = "Error get_traj_data_records(): uneven number of trajectories between sets"
        assert(len(ped_trajectories) == len(veh_trajectories)), uneven_len_err_msg

        # Retrieve safe trajectory from file
        safe_trajectory = skd_core_utils.get_safe_traj_from_file(safe_trajectory_filepath, safe_traj_index)

        # Iterate through all trajectories and add records
        for traj_index in range(len(ped_trajectories)):
            kamikaze_ped_traj = ped_trajectories[traj_index]
            kamikaze_veh_traj = veh_trajectories[traj_index]
            new_traj_pair = skd_core_utils.TrajectoryDataRecord(kamikaze_ped_traj, kamikaze_veh_traj, safe_trajectory)
            data_records.append(new_traj_pair)


        return data_records



    def get_safe_traj_file_analysers(self, safe_traj_file_dir, safe_traj_file_outdir):
        """ Parses the oppt experiment log files associated with the given safe_traj_file_dir, 
            and returns a list of OPPTLogAnalysers associated with the log files """
        safe_traj_file_analysers = []
        
        # Grab all the log files in the directory (usually 1)
        log_files = glob.glob(safe_traj_file_dir + "/*.log")

        # Iterate over the log files and create and assocaited analyser
        for log_file in log_files:
            # Local outputdir for each log analyser
            logfile_analyser_outdir = safe_traj_file_outdir + "/%s" % (os.path.basename(log_file).split(".")[0])
            log_analyser = oppt_log_analyser.OPPTLogAnalyser(logfile_analyser_outdir , log_file)

            # Initialize analysers by creating output dir and splitting runs in log files
            log_analyser.split_runs()
          
            # Save analyser
            safe_traj_file_analysers.append(log_analyser)


        return safe_traj_file_analysers



   
    def get_augmented_trajectory(self, collision_traj, safe_traj):
        """ 
        Function to compute the augmented trajectory from a successful collision path.
        The function augments the collision traj by appending equal sized_steps in the 
        from the collision point to the end point of the safe traj 
        """

        MAX_DISPLACEMENT = 0.75

        # Compare end points
        collision_traj_end = collision_traj[-1]
        safe_traj_end = safe_traj[-1]

        # Extended traj
        extend_traj = copy.deepcopy(collision_traj)

        # Check if endpoints are the same
        if(collision_traj_end != safe_traj_end):
            # Compute the augmented section with numpy
            np_augmented_traj = np.array([])
            np_collision_end = np.array(collision_traj_end)
            np_safe_traj_end = np.array(safe_traj_end)

            # Calc difference vector
            np_diff_vec = np_safe_traj_end - np_collision_end
            np_diff_vec_norm = np.linalg.norm(np_diff_vec)

            # Estimate step size
            num_steps = np.ceil(np_diff_vec_norm / MAX_DISPLACEMENT)
            #Step vec is step_size * (unitary_diff_ve)
            step_vec = (np_diff_vec/np_diff_vec_norm) * (np_diff_vec_norm / num_steps)


            # Append
            for step in range(1, int(num_steps) + 1):
                next_point = np_collision_end + (step * step_vec)
                extend_traj.append(next_point.tolist())

        return copy.deepcopy(extend_traj)






















    




