# Import yaml
import argparse
import yaml
import json
import os
import subprocess
import shutil
import fileinput
from datetime import datetime
import sys

# Utils
import OPPTLogAnalyser




class KamikazeTrajGenerator:
	"""
	Class use to generate safe trajectories of the pedestrian
	"""
	def __init__(self, safe_traj_filepath, goal_area, config_file_path, module_output_dir, timestamp):
		""" 
		Constructor for the Safe Traj Generator Module.
		Each Trajectory generator is identified by a timestamp that is used as a
		prefix in the output files of this module.
		"""
		self.timestamp = timestamp
		self.config_path = config_file_path
		self.safe_traj_filepath = safe_traj_filepath
		self.module_output_dir = module_output_dir
		self.goal_area = goal_area

		# General configurations
		gen_configs = self.get_skd_configurations()
		# Assert configurations exists
		assert (gen_configs["kamikaze_traj_gen_configs"] != None), "No Safe Traj Gen Configs"
		configs = gen_configs["kamikaze_traj_gen_configs"]
		
		# Assign extra experiment configurations
		try:
			self.num_attempts = configs["attempts_per_goal"]
			self.controller_multipliers = configs["controller_multipliers"]
			self.num_safe_trajs = configs["max_safe_trajs"]
		except Exception as e:
			print("Error in configs")


		# Load configurations
		self.oppt_logs_dir = self.module_output_dir + "/oppt_logs" + "/%s/goal_%d_%d" % (self.timestamp, self.goal_area[0], self.goal_area[1])
		self.oppt_experiment_cfgs = self.module_output_dir + "/oppt_experiment_cfgs" + "/%s/goal_%d_%d" % (self.timestamp, self.goal_area[0], self.goal_area[1])
		self.log_post_fix = self.timestamp + "_kamikaze_traj_gen_g_%d_%d" % (self.goal_area[0], self.goal_area[1])


		# Create module output_dirs
		assert (self.create_module_output_dirs()), "Error creating directores"



	def create_module_output_dirs(self):
		""" 
		Creates the corresponding output directories for the KamikazeTrajGenerator Module
		"""
		# Create ouput dir if not existent
		try:
			# Create Safe Traj Experiment Attempt Logfiles
			os.makedirs(self.oppt_logs_dir)
			# Create DB dir for successful safe crossings in the Safe Traj Experiment attempts
			os.makedirs(self.oppt_experiment_cfgs)
		# Throw exception
		except OSError as error:
			return False

		return True


	def get_skd_configurations(self):
	    """
	    Loads the yaml configuration file for the assessment of a vehicle. The function returns a
	    dictionary with the corresponding configuration fields
	    and values for the assessment.
	    """
	    with open(self.config_path) as config_file:
	        configurations = yaml.full_load(config_file)
	        print(configurations)
	        return configurations

	def run_kamikaze_traj_generator(self, index_val, planner_executable_path):
		"""
		Executes the first part of the SKD process by loading
		specific assessment configurations, and executing the experiments.
		"""
		# Create a custom cfg for the safe trajectory scenario to be attempted
		skd_python_dir = os.getcwd()
		planner_config = self.gen_kamikaze_traj_oppt_cfg(index_val)
		print(planner_config)
		# Need to change the std out and sterr of this 
		result = subprocess.run([planner_executable_path, "--cfg", planner_config], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		print(result)
		print("=============================== KAMIKAZE TRAJS GENERATED ============================================")


	def gen_kamikaze_traj_from_safe_file(self, num_safe_trajs = self.num_safe_trajs):
		""" Generates kamikaze trajectories according to the configurations specified,
		for all of the trajectories or until self.num_safe_trajs are examined """
		for safe_traj_index in range(min(self.num_safe_trajs, num_safe_trajs))
			self.run_kamikaze_traj_generator(safe_traj_index)



	def get_oppt_log_filename(self):
		return "log_ABT_Pedestrian_" + self.log_post_fix + ".log"

	def get_oppt_log_filepath(self):
		return self.oppt_logs_dir + "/" + self.get_oppt_log_filename()


	def gen_kamikaze_traj_oppt_cfg(self, safe_traj_index):
		""" 
		Generates a new ".cfg" with assessment options set to
		file_options according to the desired goal area, goal margins, and initial state parameters"""

		# Copy from a template cfg file and change the appropiate lines with 'sed'
		skd_python_dir = os.getcwd()
		planner_config_path = skd_python_dir + "/../skd_oppt/cfg/SKDBasicController/KamikazeTrajGen.cfg"

		# Destination of the new custom safeTrajGen
		assessment_configs_path = self.oppt_experiment_cfgs + "/KamikazeTrajGen_%d_%d.cfg" % (self.goal_area[0], self.goal_area[1]) 
		cfg_dst = shutil.copyfile(planner_config_path, assessment_configs_path)

		# Replace contents of file
		self.sed_file(cfg_dst, "logPath =.*", "logPath = %s" % (self.oppt_logs_dir))
		# Replace post fix to file name
		self.sed_file(cfg_dst, "logFilePostfix =.*", "logFilePostfix = %s" % (self.log_post_fix))
		# Replace number of samples 
		self.sed_file(cfg_dst, "nRuns =.*", "nRuns = %d" % (self.num_attempts))

		# Set the path for the safe file trajectory to be loaded into oppt
		self.sed_file(cfg_dst, "safeTrajFilePath =.*", "safeTrajFilePath = %s" % (self.safe_traj_filepath))
		
		# Replace goal margins
		self.sed_file(cfg_dst, "safeTrajIndex =.*", "safeTrajIndex = %s" % (safe_traj_index))

		return cfg_dst

	def sed_file(self, filepath, old_txt, new_txt):
		subprocess.call(["sed -i 's@%s@%s @' %s" % (old_txt, new_txt, filepath)], shell = True)

	def list_to_str(self, list_var):
		result = "["
		for val in list_var:
			result = result + " %s" % (val)
		result = result + " ]"
		return result



def main():
	""" Entry point for assesment """
	argparser = argparse.ArgumentParser(
	description= "Adversary Safe Trajectory Generator")

	argparser.add_argument(
		'-cfg', '--config',
		metavar='configFile',
		type=str,
		help='path to configuration file for SKD Assessment')

	argparser.add_argument(
		'-o', '--outdir',
		metavar='SafeTrajGenModuleOutpuDir',
		type=str,
		help='Parent to output directory of the module')


	# Parse arguments
	args = argparser.parse_args()
	config_path = args.config
	module_outdir = args.outdir

	print(config_path)
	print(module_outdir)

	# Create timestamp
	timestamp = datetime.now().strftime("%m-%d-%H-%M")
	
	


if __name__ == '__main__':
	main()


