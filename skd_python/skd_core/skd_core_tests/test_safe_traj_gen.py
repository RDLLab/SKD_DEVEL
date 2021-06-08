import sys, os
import argparse
from datetime import datetime
# Setup
source_path = os.path.abspath(__file__)
skd_core_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_core_dir)
if(skd_python_dir not in sys.path):
    sys.path.append(skd_python_dir)

# Import local skd_libraries
import skd_collision_tests.collision_environment.collision_env_utils as collision_env_utils
import skd_collision_tests.collision_environment.collision_environment as collision_environment
import skd_collision_tests.collision_environment.collision_experiments_loader as collision_loader
import skd_collision_tests.collision_environment.collision_data_analyser as collision_data_analyser

# Import skd core libraries
import skd_core.skd_core_utils.skd_core_utils as skd_core_utils
import skd_core.skd_core_generators.skd_safe_traj_gen as skd_safe_traj_gen


# Example function to generate safe traj generation experiments
def generate_safe_experiments_config(outputdir):
    # Set outpath for config file (Send it to ../config for now)
	# Set options for config file
    config_outpath = outputdir + "/safe_traj_gen.yaml"
    # Set goal bounds in [longit_min, longit_max, hoz_min, hoz_max]
    goal_bound1 = [119, 120, -4.75, -3.75]
    goal_bound2 = [119.5, 121, -4, -3.5]
    goal_bounds = [goal_bound1, goal_bound2]
    
    # Template path for config file
    cfg_template = skd_python_dir + "/config/SafeTrajGen.cfg"

    # Number of attempts per goal bound
    attempts = 3
    # Initial state for safe traj file [ped_longit, ped_hoz, car_longit, car_hoz, car_speed, car_intention]
    initial_state = [120, 3, 100, -2, 0, 3]
    configs = skd_core_utils.get_safe_trajs_config(goal_bounds, initial_state, cfg_template, attempts=attempts)
    # Save configs to outpath
    skd_core_utils.save_dict_to_yaml(configs, config_outpath)

    print("SAVING FILE TO PATH %s") , config_outpath

    # Return config outpath
    return config_outpath


# Execute the kamikaze trajecoty experiments based on configuration file
def execute_pomdp_safe_traj_experiments(outputdir, safe_config_filepath, planner_exec_path):
	# Create a generator to serve all the options in the configuration file
	safe_traj_generator = skd_safe_traj_gen.SafeTrajGenerator(safe_config_filepath, outputdir)
	safe_traj_generator.generate_config_safe_trajectories(planner_exec_path)




def main():
	""" Entry point for assesment """
	argparser = argparse.ArgumentParser(
	description= "Adversary Safe Trajectory Generator. Generates Safe Trajectories using the POMDP Model, according to the parameters"
	" specified in the config file (-cfg). The planner executable(-p) will be used to generate the trajectories")


	argparser.add_argument(
		'-o', '--outdir',
		metavar='SafeTrajGenModuleOutpuDir',
		type=str,
		help='Parent to output directory of the module')


	argparser.add_argument(
		'-p', '--planner',
		metavar='SafeTrajGenModuleOutpuDir',
		type=str,
		help='Parent to output directory of the module')


	# Parse arguments
	args = argparser.parse_args()
	module_outdir = args.outdir
	planner_path = args.planner


	# Generate safe trajectory gen options
	print("GENERATING CONFIG PATH")
	config_filepath = generate_safe_experiments_config(module_outdir)
	print("DONE")

	#### Generate example kamikaze traj gen from configuration file ######
	summary_file = execute_pomdp_safe_traj_experiments(module_outdir, config_filepath, planner_path)








	
	


if __name__ == '__main__':
	main()


