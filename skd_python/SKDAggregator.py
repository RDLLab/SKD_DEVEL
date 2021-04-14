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
import copy

# Utils
import SKDUtils
import SKDSafeTrajGenerator
import SKDKamikazeTrajGenerator
import OPPTLogAnalyser

from SKDKamikazeExperimentsAnalyser import KamikazeExperimentsOrganizer as KamikazeExperimentsOrganizer


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
	config_path = args.config
	module_outdir = args.outdir
	planner_exec = args.planner

	print(config_path)
	print(module_outdir)

	# Create timestamp
	timestamp = datetime.now().strftime("%m-%d-%H-%M")
	safe_traj_gen_module_outdir = module_outdir + "/SafeTrajGen"
	kamikaze_traj_gen_module_outdir = module_outdir + "/KamikazeTrajGen"
	
	# Get general configurations
	planner_exec_path = planner_exec
	general_configs = SKDUtils.get_skd_configurations(config_path)
	
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
		safe_traj_gen.run_safe_traj_generator(planner_exec_path)
		safe_traj_generators.append(safe_traj_gen)

	# Generate kamikaze trajectories for each safe_traj_file 
	for safe_generator in safe_traj_generators:
		print("Generating Kamikaze Trajectories from %s" % (safe_generator.get_safe_traj_filepath()))
		# Get file path
		safe_traj_file_path = safe_generator.get_safe_traj_filepath()
		kamikaze_goal_area = safe_generator.get_goal_area()
		num_safe_trajs = safe_generator.get_num_trajs_generated()
		# Create Kamikaze Gen
		kamikaze_traj_gen = SKDKamikazeTrajGenerator.KamikazeTrajGenerator(safe_traj_file_path, kamikaze_goal_area, config_path, kamikaze_traj_gen_module_outdir, timestamp)
		kamikaze_traj_gen.gen_kamikaze_traj_from_safe_file(planner_exec_path, num_safe_trajs)
		kamikaze_traj_generators.append(kamikaze_traj_gen)
		#print(kamikaze_traj_gen.get_process_scenario_records())

	print("======================= TRAJECTORY GENERATION STAGES COMPLETE ================================")


	# Generate SKD Computation
	general_controller_scenarios = {}
	for kamikaze_traj_gen in kamikaze_traj_generators:
		# Dictionary with controller multiplier as key and list of scenarios 
		kamikaze_gen_controller_scenarios = kamikaze_traj_gen.get_process_scenario_records()

		for controller_multiplier in kamikaze_gen_controller_scenarios:
			if (controller_multiplier in general_controller_scenarios):
				# Extend the scenarios
				general_controller_scenarios[controller_multiplier].extend(
					kamikaze_gen_controller_scenarios[controller_multiplier])
			else:
				# Seeing first time. copy the scenario list
				general_controller_scenarios[controller_multiplier] = copy.deepcopy(
					kamikaze_gen_controller_scenarios[controller_multiplier])

	# # # Generate SKD Computation
	print(general_controller_scenarios)

	# # Iterate over the kamikaze gen traj to generate input for data processing module
	# Set the outdir of organizer to be same as record_root for scenarios
	organizer_outdir = kamikaze_traj_generators[0].get_kamikaze_scenario_record_dir()
	skd_computator = KamikazeExperimentsOrganizer(general_controller_scenarios, organizer_outdir)
	skd_computator.analyze_experiments_scenarios(module_outdir + "/skd_testing.csv")


if __name__ == '__main__':
	main()


