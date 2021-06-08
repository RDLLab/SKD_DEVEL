import sys, os

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_core_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_core_dir)

if(skd_python_dir not in sys.path):
	sys.path.append(skd_python_dir)

# Import yaml
import argparse
import json, yaml, copy
import subprocess
import shutil, fileinput
from datetime import datetime

# Utils
import skd_core.skd_core_utils.skd_core_utils as skd_core_utils
import skd_core.skd_core_analysers.oppt_log_analyser as oppt_log_analyser


class SafeTrajGenerator:
	"""
	Class use to generate safe trajectories of the pedestrian
	"""
	def __init__(self, config_file_path, module_output_dir):
		""" 
		Constructor for the Safe Traj Generator Module
		"""
		# Initial settings
		self.config_path = config_file_path
		# Ouput directory (Should be created outside of the constructor)
		self.module_output_dir = module_output_dir
		
		# Local count on the number of safe trajectories generated
		self.safe_trajs_generated = []

		# Load configurations
		safe_gen_configs = skd_core_utils.get_skd_configurations(self.config_path)	
		
		# Assign extra configurations and set output dirs
		try:
			# Set extra configs
			self.goal_bounds = safe_gen_configs["goal_bounds"]
			self.num_samples = safe_gen_configs["safe_trajs_attempts_per_goal"]
			self.initial_state = safe_gen_configs["initial_state"]
			self.config_template_path = safe_gen_configs["safe_gen_cfg_file"]
			self.log_post_fix = "safe_traj_gen"


		except Exception as e:
			print("Error in configs")


		# Set output dirs
		self.oppt_logs_dir = self.module_output_dir + "/oppt_logs" 
		self.oppt_experiment_cfgs = self.module_output_dir + "/oppt_experiment_cfgs" 
		# Outdir for validators
		self.safe_traj_validator_outdir = self.module_output_dir + "/safe_traj_validator_outdir" 
		

		# Create internal outpudir directories within module output directry
		assert (self.create_module_output_dirs()), "Error creating directores"

		
	def create_module_output_dirs(self):
		""" 
		Creates the corresponding output directories for the SafeTrajGenerator Module
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


	def generate_config_safe_trajectories(self, planner_executable_path):
		""" Generate safe trajectories based on the specification """
		
		for goal_bound in self.goal_bounds:

			# Generate safe trajectories for each goal_bound configuration
			# Paths and destinations for oppt experiments
			goal_identifier = self.get_goal_bound_identifier(goal_bound)
			experiment_logpath = self.oppt_logs_dir + "/%s" % (goal_identifier)
			assessment_configs_path = self.oppt_experiment_cfgs + "/%s/SafeTrajGen.cfg" % (goal_identifier) 
			oppt_log_post_fix = self.log_post_fix + "_%s" % (goal_identifier)
			
			# Generate oppt config file
			bounds_cfg_file = self.gen_safe_traj_oppt_cfg(goal_bound, experiment_logpath, assessment_configs_path, oppt_log_post_fix)

			# Need to change the std out and sterr of this 
			result = subprocess.run([planner_executable_path, "--cfg", bounds_cfg_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

			# Validate and ouput sucessful save trajs
			oppt_result_logfile = experiment_logpath + "/%s" % (skd_core_utils.get_oppt_log_filename(oppt_log_post_fix))
			validator_outdir = self.safe_traj_validator_outdir + "/%s" % (goal_identifier)
			safe_traj_validator = SafeTrajValidator(oppt_result_logfile, validator_outdir)

			# Safe successful safe_trajs
			safe_trajs_outpath = validator_outdir + "/safe_trajs.json"
			safe_traj_validator.save_successful_safe_trajs(safe_trajs_outpath)
			# Save name saved trajectories file
			self.safe_trajs_generated.append(safe_trajs_outpath)


			print("=============================== SAFE TRAJS GENERATED ============================================")



	def gen_safe_traj_oppt_cfg(self, goal_bound, experiment_logpath, assessment_configs_path, oppt_log_post_fix):
		""" Generates a new ".cfg" with assessment options set to
		file_options according to the desired goal area, goal margins, and initial state parameters """

		# Copy from a template cfg file and change the appropiate lines with 'sed'
		skd_devel_dir = os.path.dirname(skd_python_dir)
		skd_oppt_dir = skd_devel_dir + "/skd_oppt"

		# Create output directories
		try:
			os.makedirs(os.path.dirname(assessment_configs_path))
		except OSError as error:
			print("Error creating safe traj config outdir")


		# Make a copy of the configuration file
		cfg_dst = shutil.copyfile(self.config_template_path, assessment_configs_path)

		## HARD CODED RELATIVE PATH TO DYNAMICS MODEL FOR NOW
		dynamics_dir = skd_oppt_dir + "/dynamics_files"
		intention_model_file = dynamics_dir + "/discretizeIntentions.csv"
		dynamics_model_file = dynamics_dir + "/dynamicsDB.csv"


		# Replace contents of file
		skd_core_utils.sed_file(cfg_dst, "logPath =.*", "logPath = %s" % (experiment_logpath))
		skd_core_utils.sed_file(cfg_dst, "logFilePostfix =.*", "logFilePostfix = %s" % (oppt_log_post_fix))
		# Replace number of samples
		skd_core_utils.sed_file(cfg_dst, "nRuns =.*", "nRuns = %d" % (self.num_samples))
		# Set experiments to start from a deterministicc point by setting same lower and upper bound of initil belief dist
		skd_core_utils.sed_file(cfg_dst, "lowerBound =.*", "lowerBound = %s" % (skd_core_utils.list_to_str(self.initial_state)))
		skd_core_utils.sed_file(cfg_dst, "upperBound =.*", "upperBound = %s" % (skd_core_utils.list_to_str(self.initial_state)))	
		# Replace location of dynamics and intentions file
		skd_core_utils.sed_file(cfg_dst, "intentionModelFile =.*", "intentionModelFile = %s" % (intention_model_file))
		skd_core_utils.sed_file(cfg_dst, "dynamicsModelFile =.*", "dynamicsModelFile = %s" % (dynamics_model_file))
		# Replace goal area
		skd_core_utils.sed_file(cfg_dst, "goalBounds =.*", "goalBounds = %s" % (skd_core_utils.list_to_str(goal_bound)))

		return cfg_dst


	def get_goal_bound_identifier(self, goal_bound):
		""" Return identifier string based on goal bound """
		return "goal_%d_%d_%d_%d" % (goal_bound[0], goal_bound[1], goal_bound[2], goal_bound[3])


	def get_generated_safe_files(self):
		""" Returns the filepath of the safe traj files generated by this module """
		return self.safe_trajs_generated





class SafeTrajValidator:
	""" 
	Helper Class to examine the log files describing experiments generated by SafeTrajGenerator
	"""
	def __init__(self, logfile, outputdir):
		self.logfile = logfile
		self.outdir = outputdir
		# Analyser used to examine a log from the associated oppt planner
		self.log_analyser = oppt_log_analyser.OPPTLogAnalyser(self.outdir, self.logfile)


	def save_successful_safe_trajs(self, dst_filepath):
		""" 
		Saves all the successful trajectories into a json_file, where
		each trajectory is pair with an index as a key.
		"""
		self.log_analyser.split_runs()
		safe_trajs = self.log_analyser.get_successful_ped_trajectories()

		# Safe trajectories
		safe_trajs_dict = {}
		for safe_traj_index in range(len(safe_trajs)):
			safe_trajs_dict[str(safe_traj_index)] = safe_trajs[safe_traj_index]

		with open(dst_filepath, 'w') as outfile:
			json.dump(safe_trajs_dict, outfile)

		# Returns the number of successful safe trajectories
		return len(safe_trajs)






def main():
	""" Entry point for assesment """
	argparser = argparse.ArgumentParser(
	description= "Adversary Safe Trajectory Generator. Generates Safe Trajectories using the POMDP Model, according to the parameters"
	" specified in the config file (-cfg). The planner executable(-p) will be used to generate the trajectories")

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
		metavar='SafeTrajGenModuleOutpuDir',
		type=str,
		help='Parent to output directory of the module')


	# Parse arguments
	args = argparser.parse_args()
	config_path = args.config
	module_outdir = args.outdir
	planner_exec_path = args.planner

	# Create timestamp
	timestamp = datetime.now().strftime("%m-%d-%H-%M")
	print("Timestamp: %s" % (timestamp))

	# Create generator to run all the experiments
	generator = SafeTrajGenerator(config_path, module_outdir)
	generator.generate_config_safe_trajectories(planner_exec_path)





if __name__ == '__main__':
	main()


