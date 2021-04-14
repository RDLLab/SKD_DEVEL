import os
import numpy as np
import scipy.stats as st
import random
import time


# Local class to parse oppt log files
import json
import OPPTLogAnalyser
import matplotlib.pyplot as plt
import copy
import math
# Metric computation
import Fretchet as Fretchet


""" Class to represent an experimental scenario analysis for OPPT experiments 
	Each scenario consists of a particular controlled setting (i.e vehicle controller),
	a reference safe trajectory, or safe pedestrain beheviour that was used to generates experiments
	of the scenario, and a set of log files, with a particular format, describing the results of experiment within
	the scenario """

class  OPPTGeneralGroupedScenarioAnalyser:
	# Constructor of the class
	def __init__(self, situations, output_dir, scenario_info):
		self.output_dir = output_dir
		self.situations = copy.deepcopy(situations)
		self.scenario_info = scenario_info
		self.analysers = []
		self.processed_data = False
		self.plot_outdir = self.output_dir + "/scenario_plots"
		self.traj_pairs_outdir = self.output_dir + "/trajectory_pairs"
		self.dump_outdir = self.output_dir + "/json_dumps"



	# Compute the statistics of the scenario based on the informaion inside the log files
	def process_scenario(self):
		#print(self.__str__())
		# Verify there are situations
		assert (len(self.situations) > 0), "No log file information for scenario available"
		# Create an output directory defined by the output dir
		try:
			# Create target directory for scenario analysis
			os.makedirs(self.plot_outdir)
			# Create target directory for scenario data pair dumps
			os.makedirs(self.traj_pairs_outdir)
			# Create dump outdir
			# Create target directory for scenario data pair dumps
			os.makedirs(self.dump_outdir)

		except OSError as error:
			print("ERROR WITH SCENARIO PLOT DIR")
			print(error)

		# Iterate over goal keys to create Log analyser with associated safe trajs
		for situation in self.situations:
			log_file = situation.get_log_file()
			# Log analyser output dir based on name
			log_analyser_outdir = situation.get_log_output_dir()
			
			log_analyser = OPPTLogAnalyser.OPPTLogAnalyser(log_analyser_outdir, log_file)
			log_analyser.split_runs()
			self.analysers.append(log_analyser)
			
			
		# Data is already processed
		self.processed_data = True



	####################### FUNCTIONS TO RETRIEVE PEDESTRIAN TRAJECTORIES FROM SCENARIO ########################
	""" Extract the pedestrian trajectories from all the log files in the scenario as 
		a list of trajectories (represented as a list of points_lists) """
	def get_all_scenario_traj_data_pairs(self):
		# Process scenairo information if needed
		if(not self.processed_data):
			self.process_scenario()

		# Get the trajectories for all runs in all log files in the scenario
		data_pairs = []
		# Extend list with all trajectories from all analysers
		for oppt_analyser_index in range(len(self.analysers)):
			current_situation = self.situations[oppt_analyser_index]
			analyser_trajs = self.analysers[oppt_analyser_index].get_all_ped_trajectories()
			for kamikaze_traj in analyser_trajs:
				# Create a safe traj obj
				new_pair = TrajectoryDataPair(kamikaze_traj, current_situation.get_safe_traj())
				data_pairs.append(new_pair)

		return data_pairs

	""" Extract the pedestrian trajectories from all the log files in the scenario as 
		a list of trajectories (represented as a list of points_lists) """
	def get_scenario_successful_traj_data_pairs(self, sample_limit=-1):
		# Process scenairo information if needed
		if(not self.processed_data):
			self.process_scenario()

		# Get the trajectories for all runs in all log files in the scenario
		# Collect successful traj data pairs according to safe trajectory used (indicated by g an k))
		data_map = {}
		data_pairs = []
		total_data_pairs = 0
		# Extend list with all trajectories from all analysers
		for oppt_analyser_index in range(len(self.analysers)):
			# Current and situation go in link on inde
			current_situation = self.situations[oppt_analyser_index]
			analyser_trajs, veh_trajs = self.analysers[oppt_analyser_index].get_successful_ped_veh_trajectories()
			situation_safe_traj_key = current_situation.get_safe_traj_key_id()
			situation_traj_data_pairs = []

			# Store all pairs for this iteration inside the map under the same key
			for kamikaze_traj_index in range(len(analyser_trajs)):
				kamikaze_traj = analyser_trajs[kamikaze_traj_index]
				run_time_veh_traj = veh_trajs[kamikaze_traj_index]
				# Create a safe traj obj
				new_pair = TrajectoryDataPair(kamikaze_traj, current_situation.get_safe_traj(), run_time_veh_traj)
				situation_traj_data_pairs.append(new_pair)

			# If key is here then extend. Otherwise extend list
			if(situation_safe_traj_key in data_map):
				data_map[situation_safe_traj_key].extend(situation_traj_data_pairs)
			else:
				# New entry
				data_map[situation_safe_traj_key] = situation_traj_data_pairs

			# Compute the total amount of points available
			total_data_pairs += len(situation_traj_data_pairs)

		# print("DEBUGGING MAP HERE")
		# for key in data_map:
		# 	print("KEY IS" + str(key))
		# 	print(len(data_map[key]))
		# 	print(data_map[key])
		# 	input()

		

		# Cap the limit by sicing for now
		if(sample_limit > 0):
			# Ensure data is enough
			assert (total_data_pairs >= sample_limit), "NOT ENOUGH DATA POINTS IN SCENARIO LOG FILES"

			# Force it to gather enough points to the ceiling
			SAMPLES_PER_KEY = math.ceil(sample_limit / len(data_map.keys()))

			for safe_key in data_map:
				print("HERE")
				print(data_map)
				print(len(data_map[safe_key]))
				# Check enough samples available
				assert(len(data_map[safe_key]) >= SAMPLES_PER_KEY), "NOT ENOUGHT SAMPLES FOR KEY %s" % (safe_key)
				# Shuffle pairs within this key
				random.shuffle(data_map[safe_key])
				data_map[safe_key] = data_map[safe_key][0 : SAMPLES_PER_KEY]
			
		return data_map



	########################## FUNCTION TO COMPUTE THE FRECHET DISTANCES #####################################
	""" Compute the statistics of the scenario based on the informaion inside the log files """
	def get_scenario_frechet_stats(self, augmented = True, save_plot = False, sample_limit = -1):
		# Get list of frechet distances associated with scenario
		frechet_dists_map, frechet_time_map = self.get_scenario_fretchet_dists(augmented, save_plot, sample_limit)
		# Sample total runs
		scenario_success, scenario_total_runs = self.get_scenario_run_stats()

		# Process frecht distance map
		result_statistics = [(scenario_success/scenario_total_runs)]
		result_statistics.extend(self.process_fretchet_dists_map(frechet_dists_map))
		#result_statistics.extend(self.process_fretchet_time_map(frechet_time_map))
		#result_statistics.extend(self.process_scenario_kamikaze_stats())
		# Output the statistics in a numpy array form for easy data saving
		return result_statistics


	""" Function to return the kamikaze timing statistics for the trials in this scenario """
	def process_scenario_kamikaze_stats(self):
		# Timing statistics for successfully collecting kamikaze trajs
		scenario_kamikaze_timings = self.get_scenario_kamikaze_timings()
		np_scenario_kamikaze_timings = np.array(scenario_kamikaze_timings)
		scenario_kamikaze_timings_size = len(np_scenario_kamikaze_timings)
		scenario_kamikaze_timings_mean = np.mean(np_scenario_kamikaze_timings)
		scenario_kamikaze_timings_var = np.var(np_scenario_kamikaze_timings, ddof=1)
		scenario_kamikaze_timings_ci = st.norm.interval(alpha=0.95, loc=scenario_kamikaze_timings_mean, scale=st.sem(np_scenario_kamikaze_timings))

		# Output the statistics in a numpy array form for easy data saving
		return [scenario_kamikaze_timings_size, scenario_kamikaze_timings_mean, scenario_kamikaze_timings_var, 
		scenario_kamikaze_timings_ci[0], scenario_kamikaze_timings_ci[1]]



	""" Funtion to process a dictionary map a safe_trajectory_key unique key to 
	a set of frechet distances calculated for the sample considered. The function 
	returns the statistics as a list of variables, of the fretchet distances included in 
	the map """
	def process_fretchet_dists_map(self, dists_map):
		frechet_map_means = []
		fretchet_map_vars = []
		fretchet_map_keys = []
		distances = []

		# Populate means and vars map
		for safe_traj_key in dists_map:
			distances.extend(dists_map[safe_traj_key])
			frechet_map_means.append(np.mean(dists_map[safe_traj_key]))
			fretchet_map_vars.append(np.var(dists_map[safe_traj_key], ddof=1))
			fretchet_map_keys.append(safe_traj_key)


		# Organize all variables to be outputted
		np_frechet_dists = np.array(distances)
		sample_size = len(np_frechet_dists)
		frechet_sum = np.sum(np_frechet_dists)
		# Sample mean
		frechet_mean = np.mean(np_frechet_dists)
		# Sample variance
		frechet_var = np.var(np_frechet_dists, ddof=1)
		# Compute 95% Confidence interval
		frechet_ci = st.norm.interval(alpha=0.95, loc=frechet_mean, scale=st.sem(np_frechet_dists))
		# Other grouped_var
		frechet_grouped_var = np.mean(fretchet_map_vars) + np.var(frechet_map_means, ddof=1)


		# Display total number of attempts and success 
		total_successful, total_attempts = self.get_scenario_run_stats()


		# Produce a dump here of the means and vars
		dump_map = {"MULTIPLIER" : self.scenario_info, "MEANS" : frechet_map_means, "VARS" : fretchet_map_vars, "SAFE_TRAJ_ORDER" : fretchet_map_keys} 
		dump_file = open(self.dump_outdir + "/controller_dump.json", "w")
		json.dump(dump_map, dump_file, indent = 4)
		dump_file.close()

		return [sample_size, frechet_sum, frechet_mean, frechet_var, frechet_grouped_var, frechet_ci[0], frechet_ci[1], total_attempts, total_successful]



	""" Funtion to process a dictionary map a safe_trajectory_key unique key to 
	a set of frechet timing calculated for the sample considered. The function 
	returns the statistics as a list of variables, of the fretchet distances included in 
	the map """
	def process_fretchet_time_map(self, time_map):
		frechet_map_timing_means = []
		fretchet_map_timing_vars = []
		times = []
		# Populate means and vars map
		for safe_traj_key in time_map:
			times.extend(time_map[safe_traj_key])
			frechet_map_timing_means.append(np.mean(time_map[safe_traj_key]))
			fretchet_map_timing_vars.append(np.var(time_map[safe_traj_key], ddof=1))

		# Organize all variables to be outputted
		np_times = np.array(times)
		# Sum
		frechet_time_sum = np.sum(np_times)
		# Sample mean
		frechet_time_mean = np.mean(np_times)
		# Sample variance
		frechet_time_var = np.var(np_times, ddof=1)
		# Compute 95% Confidence interval
		frechet_time_ci = st.norm.interval(alpha=0.95, loc=frechet_time_mean, scale=st.sem(np_times))
		# Other grouped_var
		frechet_grouped_var_time = np.mean(fretchet_map_timing_vars) + np.var(frechet_map_timing_means, ddof=1)


		return [frechet_time_sum, frechet_time_mean, frechet_time_var, frechet_grouped_var_time, frechet_time_ci[0], frechet_time_ci[1]]



	""" Computes a list of the fretchet distances between the set of trajectories associated with
	the scenario "trajs" against the associted safe trajectory. The function returns a list of frechet distances
	[Fretch(traj[i], ref_traj)] for all trajectories, and the time in (ms) that it took to compute the distances without the plots """
	def get_scenario_fretchet_dists(self, augmented = True, save_plot = False, sample_limit = -1):
		traj_fretchet_dists = []
		traj_fretchet_calcs_timings = []
		scenario_successful_map = self.get_scenario_successful_traj_data_pairs(sample_limit)
	

		pair_count = 0

		# Save trajectory_data
		self.save_trajectory_data_pairs(scenario_successful_map)

		# Compute fretchet for set in each trajectory dataset
		frechet_dists_map = {}
		frechet_timing_map = {}

		for safe_traj_key in scenario_successful_map:
			# Set of successful trajectory data pairs linked to the safe trajectory
			safe_traj_set = copy.deepcopy(scenario_successful_map[safe_traj_key])
			safe_traj_distances = []
			safe_traj_timings = []

			# Compute frechet for each trajectory pair
			for pair in safe_traj_set:
				# Record the CPU TIME OF THIS SECTION COMPUTING THE FRETCHET TRAJS
				t_single_calc_start = time.clock()
				# Check for augmented path computation
				if(augmented):
					augmented_traj = self.get_augmented_trajectory(pair.get_kamikaze_traj(), pair.get_safe_traj())
					augmented_fretchet = Fretchet.frechetDist(augmented_traj, pair.get_safe_traj())
					traj_fretchet_dists.append(augmented_fretchet)
					safe_traj_distances.append(augmented_fretchet)
				else:
					traj_fretchet_dists.append(Fretchet.frechet(pair.get_kamikaze_traj()), pair.get_safe_traj())
					safe_traj_distances.append(Fretchet.frechet(pair.get_kamikaze_traj()), pair.get_safe_traj())

				t_single_calc_end = time.clock()
				# Done clocking. Append timing
				single_calc = (t_single_calc_end - t_single_calc_start) * 1000
				safe_traj_timings.append(single_calc)

				#Check if a plot is to be saved
				if(save_plot):
					self.save_traj_plot(pair.get_kamikaze_traj(), pair.get_safe_traj(), pair.get_veh_traj(), augmented_fretchet, pair_count)
					pair_count += 1


			# Save individual statistics to corresponding map
			frechet_dists_map[safe_traj_key] = copy.deepcopy(safe_traj_distances)
			frechet_timing_map[safe_traj_key] = copy.deepcopy(safe_traj_timings)


		return frechet_dists_map, frechet_timing_map




	""" Generates a dump of all the trajectory data pairs with a list of trajectory data pairs, into a json file"""
	def save_trajectory_data_pairs(self, scenario_successful_map):
		outfile_root = self.traj_pairs_outdir
		json_result_map = {}

		for safe_traj_key in scenario_successful_map:
			# Output map for the safe traj key
			outmap_json = {}
			# Save all pairs under a map
			pair_count = 0
			for pair in scenario_successful_map[safe_traj_key]:
				# Create an empyt 
				pair_map = {}
				pair_map["KAMIKAZE"] = copy.deepcopy(pair.get_kamikaze_traj())
				pair_map["SAFE"] = copy.deepcopy(pair.get_safe_traj())
				outmap_json[pair_count] = copy.deepcopy(pair_map)
				pair_count += 1


			# Save full outmap in result map
			json_result_map[safe_traj_key] = outmap_json


		with open(outfile_root + "/latest_traj_pair_dump.json", "w") as outfile_json:
			json.dump(json_result_map, outfile_json, indent = 4)



	""" Function to compute the augmented trajectory from a successful collision path.
		The function augments the collision traj by appending equal sized_steps in the 
		from the collision point to the end point of the safe traj """
	def get_augmented_trajectory(self, collision_traj, safe_traj):
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


	""" Loads a csv table represetnation of the run tracker data and plots the trajectory of 
	    both the pedestrian and the car involved in the data """
	def save_traj_plot(self, kamikaze_traj, safe_traj, veh_traj, distance, plot_id):		
		# Plot data here
		fig = plt.figure()
		axes = plt.axes()

		safe_ped_hoz_points = []
		safe_ped_lon_points = []
		kamikaze_ped_hoz_points = []
		kamikaze_ped_lon_points = []
		veh_hoz_points = []
		veh_lon_points = []

		# Points in the kamikaze trajectory
		for point in kamikaze_traj:
			kamikaze_ped_lon_points.append(point[0])
			kamikaze_ped_hoz_points.append(-point[1])

		# Points in the safe trajectory
		for safe_point in safe_traj:
			safe_ped_lon_points.append(safe_point[0])
			safe_ped_hoz_points.append(-safe_point[1])

		# points for veh trajectory
		for veh_point in veh_traj:
			veh_lon_points.append(veh_point[0])
			veh_hoz_points.append(-veh_point[1])


		NUM_POINTS_KAMIKAZE = len(kamikaze_traj)
		NUM_POINTS_SAFE = len(safe_traj)
		NUM_POINTS_VEH = len(veh_traj)


		# Color settings
		alphas_ped = np.linspace(0.1, 1, NUM_POINTS_SAFE)
		alphas_kamikaze = np.linspace(0.1, 1, NUM_POINTS_KAMIKAZE)
		alphas_veh = np.linspace(0.1, 1, NUM_POINTS_VEH)
		rgba_colors_ped = np.zeros((NUM_POINTS_SAFE, 4))
		rgba_colors_kamikaze = np.zeros((NUM_POINTS_KAMIKAZE, 4))
		rgba_colors_veh = np.zeros((NUM_POINTS_VEH, 4))
		# for red the first column needs to be one
		rgba_colors_ped[:, 0] = 1.0
		# the fourth column needs to be your alphas
		rgba_colors_ped[:, 3] = alphas_ped
		# Plot the trajectory of the pedestrian
		rgba_colors_kamikaze[:, 2] = 1.0
		rgba_colors_kamikaze[:, 3] = alphas_kamikaze
		# Colors for veh
		rgba_colors_veh[:, 1] = 1.0
		rgba_colors_veh[:, 3] = alphas_veh
		# Plot the trajectory of the pedestrian
		axes.scatter(kamikaze_ped_hoz_points, kamikaze_ped_lon_points, color=rgba_colors_kamikaze)
		axes.scatter(safe_ped_hoz_points, safe_ped_lon_points, color=rgba_colors_ped)

		# Plot veh trajectory
		axes.scatter(veh_hoz_points, veh_lon_points, color=rgba_colors_veh)


		# Annotate plot start and end
		axes.annotate("S_K", (kamikaze_ped_hoz_points[0], kamikaze_ped_lon_points[0]))
		axes.annotate("E_K", (kamikaze_ped_hoz_points[-1], kamikaze_ped_lon_points[-1]))
		axes.annotate("S_S", (safe_ped_hoz_points[0], safe_ped_lon_points[0]))
		axes.annotate("E_S", (safe_ped_hoz_points[-1], safe_ped_lon_points[-1]))


		# Set options for plot
		axes.grid(axis="both")
		axes.set_title("Pedestrian Safe vs Kamikaze Trajs and Fretchet distance = %f" % (distance))
		axes.set_xlabel("Horizontal section of road")
		axes.set_ylabel("Longitudinal section of road")

		x_min = int(min(min(safe_ped_hoz_points), min(kamikaze_ped_hoz_points), min(veh_hoz_points))) - 5
		x_max = int(max(max(safe_ped_hoz_points), max(kamikaze_ped_hoz_points), max(veh_hoz_points))) + 5
		y_min = int(min(min(safe_ped_lon_points), min(kamikaze_ped_lon_points), min(veh_lon_points))) - 5
		y_max = int(max(max(safe_ped_lon_points), max(kamikaze_ped_lon_points), max(veh_lon_points))) + 5

		plt.xticks(range(x_min, x_max, 1))
		plt.yticks(range(y_min, y_max, 1))

		plt.savefig(self.plot_outdir + "/traj_%s_plot%d.png" % (self.scenario_info, plot_id))
		plt.close()


	""" Computes the total number of experiment runs and the total number of successful experiments
	associated with this scenario """
	def get_scenario_run_stats(self):
		total_runs_processed = 0
		total_successful = 0

		# Compile data from all the analysers associated with this scenario
		for experiment_analyser in self.analysers:
			# print("EXAMINING ANALYSER FROM")
			# print(experiment_analyser.get_analyser_filepath())
			analyser_total = experiment_analyser.get_num_runs()
			analyser_success = experiment_analyser.get_num_successful()
			#print("TOTAL=%d 	SUCCESS=%d" % (analyser_total, analyser_success))
			# Add it to the tally
			total_runs_processed += analyser_total
			total_successful += analyser_success



		# print("PRINTING SCENARIO STATS:%s " % (self.scenario_info))
		# print("TOTAL =  %d and SUCCESS =  %d" % (total_runs_processed, total_successful))


		return total_successful, total_runs_processed



	""" Computes the total time to compute all the successful kamikaze trajectories within the runs in the
	scenario associated with this object """
	def get_scenario_kamikaze_timings(self):
		successful_times_info = []


		# Compiled data from all the analysers associated with this scenario
		for experiment_analyser in self.analysers:
			print("EXAMINING TIMING DATA")
			print(experiment_analyser.get_analyser_filepath())
			analyser_timings = experiment_analyser.get_successful_run_timings()
			successful_times_info.extend(analyser_timings)

		# Return list of timings
		return successful_times_info



class TrajectoryDataPair:

	""" Constructor of the class. A Tracjectory Data Pair contains a kamikaze trajectory, and its equivalent 
	safe trajectory ended as a safeTrajectory object """
	def __init__(self, kamikaze_traj, safe_traj, vehicle_traj):
		self.kamikaze_traj = copy.deepcopy(kamikaze_traj)
		self.safe_traj = copy.deepcopy(safe_traj)
		self.vehicle_traj = copy.deepcopy(vehicle_traj)


	""" Returns the kamikaze traj associated with this object """
	def get_kamikaze_traj(self):
		return self.kamikaze_traj


	""" Returns the safe traj as a list associated with this object """
	def get_safe_traj(self):
		return self.safe_traj


	def get_veh_traj(self):
		return self.vehicle_traj





if __name__ == '__main__':
	test_scenario_plotting()


