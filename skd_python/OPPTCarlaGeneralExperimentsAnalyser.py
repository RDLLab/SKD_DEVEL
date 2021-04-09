import os
import shutil 
import tkinter
from tkinter import filedialog
import numpy as np
import glob
import scipy.stats as st
import json
import copy
# Local class to parse oppt log files
import OPPTGeneralGroupedScenarioAnalyser

# Metric computation
import Fretchet as Fretchet


class ScenarioSituation(safe_traj_db_prefix):
	
	def __init__(self, controller_id, goal_id, traj_key_index, logfile, output_dir):
		assert (logfile != None), "ScenarioSituation Error. Logfile is None"
		self.goal_id = goal_id
		self.traj_key_index = traj_key_index
		self.logfile = logfile
		self.controller_id = controller_id
		self.output_dir = output_dir
		self.safe_traj_db_prefix = safe_traj_db_prefix


	def get_controller_id(self):
		return self.controller_id


	def get_goal_id(self):
		return self.goal_id


	def get_traj_key(self):
		return self.traj_key_index


	def get_log_file(self):
		return self.logfile


	""" Returns an unique id tied to the goal and traj index of the current situation"""
	def get_safe_traj_key_id(self):
		return "_g_" + self.goal_id + "_k_" + self.traj_key_index


	def get_log_output_dir(self):
		# Use name of log as key for output dir
		key_file = os.path.basename(self.logfile)
		key_items = key_file.split(".")
		key = key_items[0]
		return (self.output_dir  + "/analyser_m_" + key)

	def get_safe_traj_filepath(self):
		return (self.safe_traj_db_prefix + self.goal_id + "_safe_trajs.json")

	def get_output_dir(self):
		return self.output_dir

	""" Returns the safe ped trajectory associated with this object as a list of lists(2D points) """
	def get_safe_traj(self):
		data_file = open(self.get_safe_traj_filepath())
		ped_traj_data_json = json.load(data_file)
		data_file.close()
		return ped_traj_data_json[self.traj_key_index]

	def __str__(self):
		return "GOAL ID:" + self.goal_id + "	M:" + self.controller_id + "	K:" + self.traj_key_index







""" Class to represent an experiments organizer. The prupose of this class is to retrieve 
copies of log files from different experiments and pool all scenario files from different experiments 
into the same location """
class OPPTExperimentsOrganizer:
	SITUATION_SYNTHETIC_DATA = 0
	SITUATION_REAL_DATA = 1

	# Constructor of the class
	def __init__(self, controller_ids, safe_trajs_map, output_dir, situation_type):
		self.controller_ids = controller_ids
		self.safe_trajs_map = copy.deepcopy(safe_trajs_map)
		self.output_dir = output_dir
		self.data_dirs = []
		self.processed = False
		self.situation_type = situation_type

		# Check valid situation
		assert  ((self.situation_type == self.SITUATION_REAL_DATA) or 
			(self.situation_type == self.SITUATION_SYNTHETIC_DATA)), "SITUATION TYPE IS NOT VALID"

		# Create an output directory defined by the output dir
		try:
			# Create target directory for scenario analysis
			os.makedirs(self.output_dir)
		except OSError as error:
			print("ERROR WITH OUTPUT DIR")
			print(error)

	""" Organizes the log files according to the files """
	def process_organizer(self):
		# Select directories considered by experiment
		self.data_dirs = self.retrieve_directories()
		self.create_controller_directories()
		self.copy_log_files()
		# processed is true
		self.processed = True



	""" Creates Scenario Analyzers for each of the values in the controller ids, the goal ids, and the safe keys """
	def generate_controller_scenarios(self):
		# Results
		experiments_data = []
		controller_scenarios = []

		
		# Check if organizer has been processed
		if(not self.processed):
			self.process_organizer();


		# Create a scenario for each of the Goals
		for controller_id in self.controller_ids:
			scenario_situations = []
			scenario_output_dir = self.output_dir + "/experiment_m_" + controller_id
			# append a scenario situation per goal id and traj key
			for goal_id in self.safe_trajs_map.keys():
				scenario_experiment_dir = (self.output_dir + "/experiment_m_" + controller_id +
					 "/experiment_g_" + goal_id)
				id_scenario_logfiles_path = scenario_experiment_dir + "/logfiles"

				for safe_key in self.safe_trajs_map[goal_id]:
					# Get all the log files associated with the scenario, and create a situation with each
					logfiles = glob.glob(id_scenario_logfiles_path + "/*_g_" + goal_id + "*k_" + safe_key + "_m_" + controller_id + "*")
					for logfile in logfiles:
						if(self.situation_type == self.SITUATION_SYNTHETIC_DATA):
							situation = ScenarioSituation(controller_id, goal_id, safe_key, 
								logfile, scenario_experiment_dir)
						elif(self.situation_type == self.SITUATION_REAL_DATA):
							situation = RealDataScenarioSituation(controller_id, goal_id, safe_key, 
								logfile, scenario_experiment_dir)
						# Append situation
						scenario_situations.append(situation)

			id_scenario = OPPTGeneralGroupedScenarioAnalyser.OPPTGeneralGroupedScenarioAnalyser(scenario_situations, scenario_output_dir, controller_id)
			# Save scenario analyser
			controller_scenarios.append(id_scenario)

		return controller_scenarios




	""" Creates a scenario analyzer for each controller id examined """
	def analyze_experiments_scenarios(self, outfilepath, gen_plots = False, sample_size = -1):
		# Check if organizer has been processed
		if(not self.processed):
			self.process_organizer();

		# Results
		experiments_data = []
		controller_scenarios = []
		
		# ANALYZE THE SYNTHETIC DATA FROM CARLA
		controller_scenarios = self.generate_controller_scenarios()

		# We can extract anything from this scenario analysers here
		assert (len(controller_scenarios) == len(self.controller_ids)) , "NOT ENOUGH SCENARIO CONTORLLERS"
		for controller_index in range(len(controller_scenarios)):
			scenario_data = [float(self.controller_ids[controller_index])]
			scenario_data.extend(controller_scenarios[controller_index].get_scenario_frechet_stats(augmented = True, 
				sample_limit = sample_size, save_plot = gen_plots))
			experiments_data.append(scenario_data)
			
			

		# Save plots
		#controller_scenarios[controller_index].save_scenario_plots(500)
		np_experiments_data = np.array(experiments_data)
		print(np_experiments_data)


		# Header for csv
		txt_header = "multiplier_val,kamikaze_success_rate,"
		txt_header += "frechet_sample_size,fretchet_sum,frechet_means,fretchet_var,frechet_grouped_var,frechet_95_ci_low_,frechet_95_ci_high,total_attempts,total_successful"
		txt_header += "fretchet_timings_sum(ms),fretchet_timings_means(ms),fretchet_timings_var,frechet_timings_grouped_var,fretchet_timings_95_ci_low_,fretchet_timings_95_ci_high,"
		txt_header += "scenario_kamikaze_timings_size,scenario_kamikaze_timings_mean(ms),scenario_kamikaze_timings_var,"
		txt_header += "scenario_kamikaze_timings_95_ci_low,scenario_kamikaze_timings_95_ci_high"
		
		np.savetxt(outfilepath, np_experiments_data, delimiter=",", header=txt_header, comments='')





	########################### HELPER FUNCTIONS TO INITIALIZE Experiment organizer ##############################
	# Create directory for each controller case
	def create_controller_directories(self):
		# Create directories in the output_dir path for each controller id
		for controller_id in self.controller_ids:
			for goal_id in self.safe_trajs_map.keys():
				try:
					# Create general mutliplier dir first
					os.makedirs(self.output_dir + "/experiment_m_" + controller_id)
				except OSError as error:
					pass
				try:
					# Create target directory for scenario analysis
					os.makedirs(self.output_dir + "/experiment_m_" + controller_id +
						 "/experiment_g_" + goal_id + "/logfiles")
				except OSError as error:
					print(error)



	""" Organizes the experiment directory by copying the appropiate log files from each directory
		in self.data_dirs into """
	def copy_log_files(self):
		# Go to each of the data dirs to be considered and copy the files 
		for data_directory in self.data_dirs:
			# Search for controller matches within the directory contents and copy log files if found
			for controller_id in self.controller_ids:
				for goal_id in self.safe_trajs_map.keys():
					for safe_key in self.safe_trajs_map[goal_id]:
						controller_goal_logfiles = glob.glob(data_directory 
							+ "/*_data_g_" + goal_id + "*k_" + safe_key + "_m_" + controller_id + "*")
						dst_dir = (self.output_dir + "/experiment_m_" + controller_id +
							 "/experiment_g_" + goal_id + "/logfiles/")

						# Copy all source files found
						for src_file in controller_goal_logfiles:
							dst = dst_dir + os.path.basename(src_file)
							shutil.copyfile(src_file, dst)




	""" Uses a GUI selection of experiment directories to be considered by an instance 
	of this class """
	def retrieve_directories(self):
		root = tkinter.Tk()
		dirselect = filedialog.Directory()
		directories = []
		while True:
		    selected_dir = dirselect.show()
		    if not selected_dir: break
		    directories.append(selected_dir)
		root.destroy()
		return directories



def compute_synthetic_data():
	# Keys to compute the data
	controller_ids = [".50", ".625", ".75", ".875", "1.00", "1.05", "1.10", "1.125", "1.15"]
	#controller_ids = ["1"]
	safe_trajs_map = {"16" : ["0", "1"], 
                        "18" : ["1"], 
                        "20" : ["0", "1"]}

	# Output files
	outdata_filename = "/home/jimy/Desktop/NUMBERS/SyntheticDataStats.csv"
	output_dir = "/home/jimy/Desktop/NUMBERS/SyntheticDataStats"

	experiment_organizer = OPPTExperimentsOrganizer(controller_ids, safe_trajs_map,
	output_dir, OPPTExperimentsOrganizer.SITUATION_SYNTHETIC_DATA)
	#experiment_organizer.process_organizer()
	experiment_organizer.analyze_experiments_scenarios(outdata_filename, gen_plots = False, sample_size = -1)


def compute_synthetic_data_straight():
	# Keys to compute the data
	controller_ids = [".50", ".625", ".75", ".875", "1.00", "1.05", "1.10", "1.125", "1.15"]
	safe_trajs_map = {"0" : ["0"]}

	# Output files
	outdata_filename = "/home/jimy/Desktop/KamikazeResults/StraightLineSyntheticKamikazeResuts.csv"
	output_dir = "/home/jimy/Desktop/KamikazeResults/StraightLineSyntheticKamikazeResuts"

	experiment_organizer = OPPTExperimentsOrganizer(controller_ids, safe_trajs_map,
	output_dir, OPPTExperimentsOrganizer.SITUATION_SYNTHETIC_DATA)
	#experiment_organizer.process_organizer()
	experiment_organizer.analyze_experiments_scenarios(outdata_filename, gen_plots = True, sample_size = 500)



def compute_real_data():
	# Keys to compute the data
	controller_ids = [".50", ".625", ".75", ".875", "1.00", "1.05", "1.10", "1.125", "1.15"]
	safe_trajs_map = {"2" : ["6"], 
						"3" : ["3" , "7"], 
						"4" : ["3", "6"]}
	# Output files
	outdata_filename = "/home/jimy/Desktop/NUMBERS/RealDataKamikazeResuts.csv"
	output_dir = "/home/jimy/Desktop/NUMBERS/RealDataKamikazeResuts"

	experiment_organizer = OPPTExperimentsOrganizer(controller_ids, safe_trajs_map,
	output_dir, OPPTExperimentsOrganizer.SITUATION_REAL_DATA)
	#experiment_organizer.process_organizer()
	experiment_organizer.analyze_experiments_scenarios(outdata_filename, gen_plots = False, sample_size = 500)

	

	



if __name__ == '__main__':
	compute_real_data()




