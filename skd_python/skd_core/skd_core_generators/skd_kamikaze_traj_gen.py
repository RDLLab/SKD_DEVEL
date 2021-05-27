import sys, os

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_core_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_core_dir)


# Append top level library path
if(skd_python_dir not in sys.path):
	sys.path.append(skd_python_dir)



import argparse, subprocess, shutil
import yaml, json, fileinput, copy
from datetime import datetime


# Utils
import skd_core.skd_core_utils.skd_core_utils as skd_core_utils
import skd_trajectories.trajectories_filters as trajs_filters
import skd_collision_tests.controllers.car_controllers as car_controllers
import skd_trajectories.trajectories_filters as traj_filters



class KamikazeTrajGenerator:
	"""
	Class use to generate kamikaze trajectories associated with the safe trajectories 
	in safe_traj_filepath. The module outputs 
	"""
	def __init__(self, config_file_path, module_output_dir):
		""" 
		Constructor for the Safe Traj Generator Module.
		Each Trajectory generator is identified by a timestamp that is used as a
		prefix in the output files of this module.
		"""
		self.config_path = config_file_path
		self.module_output_dir = module_output_dir

		# Load kamikaze section of config file
		kamikaze_configs = skd_core_utils.get_skd_configurations(self.config_path)
		
		# Assign extra experiment configurations
		try:
			self.safe_traj_files = copy.deepcopy(kamikaze_configs["safe_traj_files"])
			self.num_attempts = kamikaze_configs["attempts_per_goal"]
			self.controller_multipliers = kamikaze_configs["controller_multipliers"]
			self.max_trajs_per_file = kamikaze_configs["max_trajs_per_file"]
			self.config_template_path = kamikaze_configs["kamikaze_cfg_file"]
		except Exception as e:
			print("Error in configs")


		# Load configurations
		self.oppt_logs_dir = self.module_output_dir + "/kamikaze_traj_gen_experiments_logs" 
		self.oppt_experiment_cfgs = self.module_output_dir + "/kamikaze_traj_gen_experiments_cfgs"
		self.experiments_summary_dir = self.module_output_dir + "/kamikaze_experiments_summary"
		self.loader_summary_path = self.experiments_summary_dir + "/experiments_summary.yaml"
		

		# Create module output_dirs
		self.create_module_output_dirs()



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
			# Create dir for experiments summary dump
			os.makedirs(self.experiments_summary_dir)
		# Throw exception
		except Exception as error:
			print(error)

		

	def get_module_outdir(self):
		return self.module_output_dir
		

	def execute_kamikaze_traj_gen_configs(self, planner_executable_path):
		"""
		Executes the first part of the SKD process by loading
		specific assessment configurations, and executing the experiments.
		"""
		# Summary record of experiments
		experiments_summary = {}

		# Create a custom cfg for the safe trajectory scenario to be attempted
		skd_python_dir = os.path.dirname(skd_core_dir)
		
		# Run a sequence of experiments fo each of the controller multiplier values specified
		for controller_multiplier in self.controller_multipliers:
			# Store collection of experimental summaries for the safe trajectory files considered for the controller
			controller_safe_traj_file_summaries = []

			# Run experiments for each of the safe trajectory files included
			for safe_traj_filename in self.safe_traj_files:

				safe_traj_file_summary = self.get_safe_traj_file_summary(controller_multiplier, 
										safe_traj_filename, planner_executable_path)
				
				# Save summary of safe_traj_fileame
				controller_safe_traj_file_summaries.append(copy.deepcopy(safe_traj_file_summary))

			# Save a summary of experiments per controller multiplier
			controller_multiplier_summary = {"controller_multiplier" : controller_multiplier, 
											"safe_traj_file_summaries" : copy.deepcopy(controller_safe_traj_file_summaries)
											}

			experiments_summary[str(controller_multiplier)] = copy.deepcopy(controller_multiplier_summary)
			

		# Save experiments summary and return summary
		skd_core_utils.save_dict_to_yaml(experiments_summary, self.loader_summary_path)


	def get_safe_traj_file_summary(self, controller_multiplier, safe_traj_filename, planner_executable_path):
		# Store the collection of traj file indices and their output directory
		safe_traj_filename_log_dirs = []

		# Set cap on number of trajectories considered per file
		TRAJS_PER_FILE = self.max_trajs_per_file
		if(self.max_trajs_per_file <= 0):
			TRAJS_PER_FILE = skd_core_utils.get_num_safe_trajs(safe_traj_filename)

		# Run experiments for each of the trajectory index in the safe trajectory file
		for safe_traj_number in range(TRAJS_PER_FILE):
			# Generate an oppt configuration file
			planner_config = self.gen_kamikaze_traj_oppt_cfg(safe_traj_filename, safe_traj_number, 
				controller_multiplier)

			# Need to change the stdoout and sterr of this	
			result = subprocess.run([planner_executable_path, "--cfg", planner_config], 
				stdout=subprocess.PIPE, stderr=subprocess.PIPE)

			# Store the result of each of the dirs
			safe_traj_filekey = self.get_safe_traj_filekey(safe_traj_filename)
			kamikaze_config_suffix = self.get_kamikaze_config_suffix(controller_multiplier, safe_traj_filekey, safe_traj_number)
			safe_traj_filename_log_dirs.append(self.get_oppt_logs_dir(kamikaze_config_suffix))


		# Save a summary of the safe_traj_file
		safe_traj_file_summary = {"safe_traj_filepath" : safe_traj_filename,
								"safe_traj_file_log_dirs" : safe_traj_filename_log_dirs}

		return safe_traj_file_summary



	
	def get_kamikaze_config_suffix(self, controller_multiplier, safe_traj_filekey, safe_traj_index):
		return skd_core_utils.get_kamikaze_config_suffix(controller_multiplier, safe_traj_filekey, safe_traj_index)


	def get_oppt_logs_dir(self, kamikaze_config_suffix):
		return self.oppt_logs_dir + "/%s" % (kamikaze_config_suffix)


	def get_config_db_dir(self, kamikaze_config_suffix):
		return self.oppt_experiment_cfgs + "/%s" % (kamikaze_config_suffix)


	def get_safe_traj_filekey(self, safe_traj_filename):
		return os.path.basename(safe_traj_filename).split(".")[0]


	def get_experiments_summary_dir(self):
		return self.loader_summary_path





	def gen_kamikaze_traj_oppt_cfg(self, safe_traj_filename, safe_traj_index, controller_multiplier):
		""" 
		Generates a new ".cfg" with assessment options set to
		file_options according to the desired goal area, goal margins, and initial state parameters"""

		# Copy from a template cfg file and change the appropiate lines with 'sed'

		# Destination of the new custom safeTrajGen
		safe_traj_filekey = self.get_safe_traj_filekey(safe_traj_filename)
		# Output dir for oppt config files for each experiment
		kamikaze_config_suffix = self.get_kamikaze_config_suffix(controller_multiplier, safe_traj_filekey, safe_traj_index) 
		kamikaze_config_db_dir = self.get_config_db_dir(kamikaze_config_suffix)

		# Output dir for log files outputted by oppt after an experiment
		kamikaze_oppt_log_dir = self.get_oppt_logs_dir(kamikaze_config_suffix)

		# Create output directory for the kamikaze config db
		try:
			os.makedirs(kamikaze_config_db_dir)
			os.makedirs(kamikaze_oppt_log_dir)
		except OSError as error:
			print("Error creating kamikaze outdir")


		# Set the destination of the experiment configuration file
		assessment_configs_path = kamikaze_config_db_dir + "/KamikazeTrajGen.cfg" 

		
		# Assess car_start position according to trajectories module
		# Create a traj filter according to the car
		config_safe_trajectory =  skd_core_utils.get_safe_traj_from_file(safe_traj_filename, safe_traj_index)
		# Create a default controller to get starting pos
		car_controller = car_controllers.BasicCarController(controller_multiplier)

		# Query a startig pos
		car_start_pos = traj_filters.get_car_starting_pos(config_safe_trajectory, car_controller)
       
		# Make a local copy of the configuration file
		cfg_file = shutil.copyfile(self.config_template_path, assessment_configs_path)

		# Destination of the oppt log file 
		skd_core_utils.sed_file(cfg_file, "logPath =.*", "logPath = %s" % (kamikaze_oppt_log_dir))

		# Replace post fix to file name
		skd_core_utils.sed_file(cfg_file, "logFilePostfix =.*", "logFilePostfix = %s" 
			% ("kamikaze_traj_gen"))

		# Replace number of samples 
		skd_core_utils.sed_file(cfg_file, "nRuns =.*", "nRuns = %d" % (self.num_attempts))

		# Set starting car pos, according to experiments
		skd_core_utils.sed_file(cfg_file, "carStartPos =.*", "carStartPos = [%f, %f]" % (car_start_pos[0], car_start_pos[1]))

		# Set the path for the safe file trajectory to be loaded into oppt
		skd_core_utils.sed_file(cfg_file, "safeTrajFilePath =.*", "safeTrajFilePath = %s" % (safe_traj_filename))
		# Replace goal margins
		skd_core_utils.sed_file(cfg_file, "safeTrajIndex =.*", "safeTrajIndex = %s" % (safe_traj_index))
		# Set Controller multiplier value tested against
		skd_core_utils.sed_file(cfg_file, "controllerMultiplier =.*", "controllerMultiplier = %s" % (controller_multiplier))

		return cfg_file




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

	argparser.add_argument(
		'-oppt', '--planner',
		metavar='planner_exec_path',
		type=str,
		help='path to configuration file for SKD Kamikaze Traj Generation')
 

	# Parse arguments
	args = argparser.parse_args()
	config_path = args.config
	module_outdir = args.outdir
	planner_path = args.planner

	# Get config file
	print("Generating kamikaze trajectories from configurations in: %s" % (config_path))
	print("Output directory: %s" % (module_outdir))

	# Create timestamp
	timestamp = datetime.now().strftime("%m-%d-%H-%M")
	module_outdir = module_outdir + "_%s" % (timestamp) # Add timestamp to make output unique

	# Create a generator to serve all the options in the configuration file
	kamikaze_generator = KamikazeTrajGenerator(config_path, module_outdir)
	kamikaze_generator.execute_kamikaze_traj_gen_configs(planner_path)
	


if __name__ == '__main__':
	main()


