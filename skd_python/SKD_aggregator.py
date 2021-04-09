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
import SKDSafeTrajGenerator
import SKDKamikazeTrajGenerator
import OPPTLogAnalyser




def get_skd_configurations(config_path):
    """
    Loads the yaml configuration file for the assessment of a vehicle. The function returns a
    dictionary with the corresponding configuration fields
    and values for the assessment.
    """
    with open(config_path) as config_file:
        configurations = yaml.full_load(config_file)
        print(configurations)
        return configurations

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
	safe_traj_gen_module_outdir = module_outdir + "/SafeTrajGen"
	kamikaze_traj_gen_module_outdir = module_outdir + "/KamikazeTrajGen"
	
	# Get general configurations
	general_configs = get_skd_configurations(config_path)
	
	# Start here
	assert (general_configs["safe_traj_gen_configs"] != None), "No Safe traj gen configs"
	assert (general_configs["kamikaze_traj_gen_configs"] != None), "No Kamikaze traj gen configs"
	
	# Start assesment
	# Safe a list of generators
	safe_traj_generators = []
	# Kamikaze 
	kamikaze_traj_generators = []
	goal_areas = general_configs["safe_traj_gen_configs"]["goal_areas"]


	# Generate safe trajectories
	for goal_area_index in range(len(goal_areas)):
		print("ASSESSING GOAL AREA")
		# Create a SafeTrajGenerator for the goal area
		safe_traj_gen = SKDSafeTrajGenerator.SafeTrajGenerator(config_path, goal_area_index, safe_traj_gen_module_outdir, timestamp)
		safe_traj_gen.run_safe_traj_generator()
		safe_traj_generators.append(safe_traj_generators)

	# Generate kamikaze trajectories for each safe_traj_file 
	for safe_generator in safe_traj_generators:
		print("Generating Kamikaze Trajectories")
		print("Safe Traj file: %s, Safe_traj index: %s")
		# Get file path
		safe_traj_file_path =safe_generator.get_safe_traj_filepath()
		# Create Kamikaze Gen
		kamikaze_traj_gen = SKDKamikazeTrajGenerator.KamikazeTrajGenerator(safe_traj_file_path, config_path, kamikaze_traj_gen_module_outdir, timestamp)



if __name__ == '__main__':
	main()


