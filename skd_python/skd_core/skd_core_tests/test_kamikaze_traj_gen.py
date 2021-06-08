import sys, os, glob
import argparse, subprocess, shutil
import yaml, json, fileinput, copy
from datetime import datetime

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_core_dir = os.path.dirname(os.path.dirname(source_path))
skd_python_dir = os.path.dirname(skd_core_dir)
if(skd_core_dir not in sys.path):
	sys.path.append(skd_core_dir)


# Import skd core modules
import skd_core_utils.skd_core_utils as skd_core_utils
import skd_core_generators.skd_kamikaze_traj_gen as skd_kamikaze_traj_gen
import skd_core_analysers.skd_kamikaze_data_analyser as skd_kamikaze_data_analyser



# Example function to generate kamikaze traj generation experiments
def generate_kamikaze_experiments_config(planner_exec_path):
    # Set outpath for config file (Send it to ../config for now)
	# Set options for config file
    config_outpath = os.getenv("HOME") + "/Desktop/tmp/kamikaze_traj_gen.yaml"
    generic_cfg_path = skd_python_dir + "/config/KamikazeTrajGen.cfg"
    controller_ids = [0.5, 1.0]
    # Safe trajectory filepaths to consider. If empty (GUI will ask user for files)
    safe_traj_files = []
    attemps_per_file = 2
    max_traj_per_file = 2
    skd_core_utils.generate_kamikaze_configs(config_outpath, controller_ids, generic_cfg_path,
    			 attempts = attemps_per_file, trajs_per_file = max_traj_per_file)
    
    # Return config outpath
    return config_outpath



# Execute the kamikaze trajecoty experiments based on configuration file
def execute_kamikaze_experiments(outputdir, kamikaze_config_filepath, planner_exec_path):
	# Create a generator to serve all the options in the configuration file
	kamikaze_generator = skd_kamikaze_traj_gen.KamikazeTrajGenerator(kamikaze_config_filepath, outputdir)
	kamikaze_generator.execute_kamikaze_traj_gen_configs(planner_exec_path)

	return kamikaze_generator.get_experiments_summary_dir()






def analyse_experimental_output(summary_file_path, outputdir):
	analyser = skd_kamikaze_data_analyser.SKDKamikazeDataAnalyser(summary_file_path, outputdir)
	analyser.parse_summary_data()




def main():
	planner_path = os.getenv("HOME") + "/jimy_ws/oppt/bin/abt"
	print("GENERATING CONFIG PATH")
	config_filepath = generate_kamikaze_experiments_config(planner_path)
	print("EXECUTING EXPERIMENTS ON CONFIG FILE")
	experiments_outdir = os.getenv("HOME") + "/Desktop/tmp/testingKamikaze"
	summary_file = execute_kamikaze_experiments(experiments_outdir, config_filepath, planner_path)
	print("SUMMARY FILE IS")
	print(summary_file)
	# Print (Analysing experiments)
	print("Analysing experiments output data")
	experiments_data_dir = os.getenv("HOME") + "/Desktop/tmp/testingKamikazeData"
	analyse_experimental_output(summary_file, experiments_data_dir)







	
	


if __name__ == '__main__':
	main()