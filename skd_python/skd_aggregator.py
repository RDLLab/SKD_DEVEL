import sys,os
import argparse
from datetime import datetime
import time

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_python_dir = os.path.dirname(source_path)
sys.path.append(skd_python_dir)

# Import local skd_libraries
import skd_collision_tests.collision_environment.collision_env_utils as collision_env_utils
import skd_collision_tests.collision_environment.collision_environment as collision_environment
import skd_collision_tests.collision_environment.collision_experiments_loader as collision_loader
import skd_collision_tests.collision_environment.collision_data_analyser as collision_data_analyser

# Import skd core libraries
import skd_core.skd_core_utils.skd_core_utils as skd_core_utils
import skd_core.skd_core_generators.skd_kamikaze_traj_gen as skd_kamikaze_traj_gen
import skd_core.skd_core_analysers.skd_kamikaze_data_analyser as skd_kamikaze_data_analyser
import skd_core.skd_core_generators.skd_safe_traj_gen as skd_safe_traj_gen


def main():
	argparser = argparse.ArgumentParser(
	description= "Adversary Safe Trajectory Generator")

	argparser.add_argument(
	 	'-cfg', '--config',
		metavar='configFile',
		type=str,
		help='path to configuration file for SKD Assessment')

	argparser.add_argument(
		'-o', '--outdir',
		metavar='AggregatorModuleOutpuDir',
		type=str,
		help='Parent to output directory of the module')

	argparser.add_argument(
		'-oppt', '--planner',
		metavar='planner_exec_path',
		type=str,
		help='path to configuration file for SKD Assessment')


	# Parse arguments
	args = argparser.parse_args()
	skd_config_file = args.config
	module_outdir = args.outdir
	planner_exec = args.planner


	# Create ouput dirs
	safe_traj_gen_dir = module_outdir + "/SafeTrajGenDir"
	kamikaze_traj_gen_dir = module_outdir + "/KamikazeTrajDir"
	kamikaze_analyser_dir = module_outdir + "/KamikazeAnalyserDir"

	# Create dirs
	try:
		os.makedirs(safe_traj_gen_dir)
		os.makedirs(kamikaze_traj_gen_dir)
		os.makedirs(kamikaze_analyser_dir)
	except OSError as error:
		print("ERROR creating dirs in SKD Aggregator")

	# Grab configs and split them locally
	skd_options = skd_core_utils.get_skd_configurations(skd_config_file)
	safe_options = skd_options["safe_traj_gen_configs"]
	kamikaze_options = skd_options["kamikaze_traj_gen_configs"]

	# Save to local file for now
	safe_configs_path = module_outdir + "/safe_configs.yaml"
	skd_core_utils.save_dict_to_yaml(safe_options, safe_configs_path)

	# Start safe traj generation based on given config file
	print("============================ GENERATING SAFE TRAJECTORIES FROM OPPT ==========================================")
	safe_traj_generator = skd_safe_traj_gen.SafeTrajGenerator(safe_configs_path, safe_traj_gen_dir)
	safe_traj_generator.generate_config_safe_trajectories(planner_exec)

	# Append generated files to kamikaze config
	safe_traj_files = safe_traj_generator.get_generated_safe_files()
	kamikaze_configs_path = module_outdir + "/kamikaze_configs.yaml"
	kamikaze_options["safe_traj_files"] = safe_traj_files
	# Save local modified of configurations
	skd_core_utils.save_dict_to_yaml(kamikaze_options, kamikaze_configs_path)

	print("============================ GENERATING ASSOCIATED KAMIKAZE TRAJECTORIES ===============================")
	# Run kamikae trajectory experiment
	kamikaze_generator = skd_kamikaze_traj_gen.KamikazeTrajGenerator(kamikaze_configs_path, kamikaze_traj_gen_dir)
	kamikaze_generator.execute_kamikaze_traj_gen_configs(planner_exec)
	summary_file = kamikaze_generator.get_experiments_summary_dir()

	print("SUMMARY FILE IS")
	print(summary_file)
	# Print (Analysing experiments)
	print("Analysing experiments output data")
	analyser = skd_kamikaze_data_analyser.SKDKamikazeDataAnalyser(summary_file, kamikaze_analyser_dir)
	analyser.parse_summary_data()
	




	

if __name__ == '__main__':
	main()
