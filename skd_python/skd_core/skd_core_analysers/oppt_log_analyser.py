import matplotlib.pyplot as plt
import argparse
import os, sys
import copy
import csv
import statistics
import numpy as np
from pprint import pprint

# Add parent dir to package
source_path = os.path.abspath(__file__)
skd_core_dir = os.path.dirname(os.path.dirname(source_path))
if(skd_core_dir not in sys.path):
	sys.path.append(skd_core_dir)


# Metric computation
import skd_core_metrics.Fretchet as Fretchet


""" Class to represent a particle in a belief from oppt """
class OPPTParticle:
	# Constructor
	def __init__(self, state, weight):
		# State associated with particle
		self.state = state
		# Weight associated with particl
		self.weight = weight


	# Print format of the particle
	def print_particle(self):
		print("Particle state")
		print(self.state)
		print("Particle weight")
		print(self.weight)




""" Parses a generic log file outputted by oppt """
class OPPTLogAnalyser:
	# Class constants
	# STATE SPACE IS [PED_LONGIT, PED_HOZ, CAR_LONGIT, CAR_HOZ, CAR_SPEED, CAR_INTENTION]
	PED_LONGIT_INDEX = 0
	PED_HOZ_INDEX = 1
	CAR_LONGIT_INDEX = 2
	CAR_HOZ_INDEX = 3
	CAR_LONGIT_SPEED = 4
	CAR_INTENTION = 5

	# ACTION SPACE IS [PED_ACT_LONGIT, PED_ACT_HOZ]
	PED_ACT_LONGIT = 0
	PED_ACT_HOZ = 1

	# OBSERVATION SPACE IS [OBS_REL_HOZ, OBS_REL_LONGIT]
	## NEED TO VERIFY THIS
	OBSERVATION_REL_HOZ = 0
	OBSERVATION_REL_LONGIT = 1

	def __init__(self, output_dir, filepath):
		# Path to output of this program
		self.output_dir = output_dir
		# Path to the logfile
		self.filepath = filepath
		# Number of runs processed
		self.batch_size = 0
		# Flag to indicate if the log file associated with the analyser has been processed
		self.processed_file = False
		# Statistics of the associated log file after parsing
		self.discounted_rewards = []
		self.run_statuses = []
		self.num_successful = 0
		self.run_timings = []
		


	""" Get number of runs processed by parser """
	def get_num_runs(self):
		return self.batch_size

	def get_num_successful(self):
		return self.num_successful

	""" Get the processed status of this analyser """
	def is_processed(self):
		return self.processed_file

	""" Get the success statistics of the runs in the associated log file """
	def get_success_statistics(self):
		return self.discounted_rewards, self.run_statuses, self.num_successful


	""" Retrieve the run timing data for the runs in this log file """
	def get_run_timings(self):
		return self.run_timings

	""" Queries for the oppt log file assocaited with this analyser """
	def get_analyser_filepath(self):
		return self.filepath

	""" Returns the list of the successful timings as recorded by the indices of the  self.run_statuses """
	def get_successful_run_timings(self):
		successful_timings = []

		assert (len(self.run_timings) == len(self.run_statuses)), "NUMBER OF TIMINGS AND STATUSES DON'T MATCH"

		# Append successfull timings to the result
		for run_index in self.run_statuses:
			if(self.run_statuses[run_index] == True):
				# Successful runs
				successful_timings.append(self.run_timings[run_index])


		return successful_timings



	""" Reads from an OPPT logfile and extracts each experiment run into an independent log file """
	def split_runs(self):
		#FILE IO
		readFile = open(self.filepath)

		# Create local dirs for particular log analysis
		try:
			# Create target directory for individual run logs
			run_path = self.output_dir + "/runFiles"
			plot_path = self.output_dir +"/plots"
			os.makedirs(run_path)
			os.makedirs(plot_path)
		except: 
			pass

		# Iterate over file
		run_num = 0 

		# Read first line
		currentLine = readFile.readline()

		# Iterate until end of file
		while(currentLine != ""):
			# Copy run info into its own file
			if("Run #" in currentLine):
				# Creat new run file with appropiate run index
				newFileName = "%s/runFiles/run%d.txt" % (self.output_dir, run_num)
				outFile = open(newFileName, "w+")
				# Print headers for individual run file
				outFile.write("------------------------- LOG OF RUN NUMBER %d-------------------------\n" % run_num)
				outFile.write(currentLine)
				# Consume lines 
				currentLine = readFile.readline()
				# Copy all lines into the new file until the end of the run information
				while(not("RUN_FINISHED_USER_DATA_END" in currentLine)):	
					# Check if run was successful based on positive or negative rewards
					if("Total discounted reward:" in currentLine):
						# Parse the current line
						line_content  = (currentLine.split())
						# Last splitted value is reward. Cast it to float
						total_discounted_reward = float(line_content[-1])
						self.discounted_rewards.append(total_discounted_reward)
						if(total_discounted_reward > 0):
							self.run_statuses.append(True)
							self.num_successful += 1
						else:
							self.run_statuses.append(False)

					# Extract timing for current run as well
					if("Total time taken" in currentLine):
						# Parse the current line
						time_line = (currentLine.split())
						# time info is the last item
						time_data_ms = time_line[-1]
						# extract the float out of the unit
						run_time_ms_split = time_data_ms.split("ms")
						run_time_ms = float(run_time_ms_split[0])
						self.run_timings.append(run_time_ms)

					# Write line and move to next one
					outFile.write(currentLine)
					currentLine = readFile.readline()

				#close file
				outFile.close()
				# Increment file number
				run_num +=1
			# Get to next line
			currentLine = readFile.readline()
		#Save size of runs
		self.batch_size = run_num
		# Mark processed file
		self.processed_file = True

		#Close file
		readFile.close()




################################### EXTRACT INFORMATION FROM RUNS IN LOG FILE ######################################
	""" Extracts the pedestrian trajectory for an specified run number in the associated experiment
		log file """
	def get_ped_trajectory(self, run_num):
		# STATE INDICES
		PED_LONGIT_INDEX = 0
		PED_HORIZONAL_INDEX = 1
		# Return fraction of state data
		return self.get_state_data(run_num, PED_LONGIT_INDEX, PED_HORIZONAL_INDEX)


	""" Extracts the pedestrian trajectory for an specified run number in the associated experiment
		log file """
	def get_veh_trajectory(self, run_num):
		# Return fraction of state data
		return self.get_state_data(run_num, self.CAR_LONGIT_INDEX, self.CAR_HOZ_INDEX)

	""" Extracts all the pedestrian trajectories within the associated log file """
	def get_all_ped_trajectories(self):
		trajectories = []

		# Combine all trajectories from the different runs in the log file
		for run_number in range(self.batch_size):
			trajectories.append(self.get_ped_trajectory(run_number))

		# Return
		return trajectories


	""" Extracts the pedestrian trajectories of the experimental runs, in which the goal 
		was reached """
	def get_successful_ped_trajectories(self):
		trajectories = []

		# Compute the run numbers of the successful trajectories
		assert (self.processed_file), "Log file has not been processed"
		assert (self.batch_size == len(self.run_statuses)), "Number of processed runs does not match success stats"

		# Get the successsful indices
		for run_number in range(self.batch_size):
			if(self.run_statuses[run_number] == True):
				# Append the successful trajectory to the list
				trajectories.append(self.get_ped_trajectory(run_number))

		return copy.deepcopy(trajectories)



	""" Extracts the pedestrian trajectories of the experimental runs, in which the goal 
		was reached """
	def get_successful_ped_veh_trajectories(self):
		ped_trajectories = []
		veh_trajectories = []

		# Compute the run numbers of the successful trajectories
		assert (self.processed_file), "Log file has not been processed"
		assert (self.batch_size == len(self.run_statuses)), "Number of processed runs does not match success stats"

		# Get the successsful indices
		for run_number in range(self.batch_size):
			if(self.run_statuses[run_number] == True):
				# Append the successful trajectory to the list
				ped_trajectories.append(copy.deepcopy(self.get_ped_trajectory(run_number)))
				veh_trajectories.append(copy.deepcopy(self.get_veh_trajectory(run_number)))

		return ped_trajectories, veh_trajectories






	""" Extracts the information for a speficied run number in the associatd experiment log file.
		The information extract is parametrized by the given start_index and up to and includeing
		the end_index of the variables in the state space """
	def get_state_data(self, run_num, state_start_index, state_end_index):
		STATE_SPACE_SIZE = 6
		STATE_DESCRIPTION_OFFSET = 1
		# Check that data has been processed
		assert (self.processed_file), "Associated log file has not been processed"
		# Check that run_number exists
		assert (run_num < self.batch_size), "Run number exceeds processed data"
		# Check that data indices are valid
		assert (state_start_index >= 0 and state_start_index < STATE_SPACE_SIZE), "Invalid state_start_index"
		assert (state_end_index > 0 and state_end_index < STATE_SPACE_SIZE), "Invalid state_end_index"

		# File to read from 
		readFile = open("%s/runFiles/run%d.txt" % (self.output_dir, run_num), "r")
		# List to hold data
		state_data = []

		# Read first line
		currentLine = readFile.readline()
		# Iterate until end of file
		while(currentLine != ""):
			# Check for lines describing a state and save its info
			if("END USER_DATA_BEGIN  USER_DATA_END" in currentLine):
				# save state variables of current state
				state_line_items = (currentLine.split())
				state_list = [float(item) for item in state_line_items[state_start_index + STATE_DESCRIPTION_OFFSET : 
								state_end_index + STATE_DESCRIPTION_OFFSET + 1]]
				state_data.append(state_list)

			# Get to next line
			currentLine = readFile.readline()

		readFile.close()

		# Returns information extracted for run
		return copy.deepcopy(state_data)



	

##################################### PLOTTING FUNCITONALITIES FOR RUNS IN LOG FILE ################################
	""" Loads a csv table represetnation of the run tracker data and plots the trajectory of 
	    both the pedestrian and the car involved in the data """
	def save_plot_number(self, run_num, distance):
		# Pack information from pedestrian and car's location
		ped_loc_longit_points = []
		ped_loc_horizontal_points = []
		car_loc_longit_points = []
		car_loc_horizontal_points = []

		# Get data from run file
		states = self.get_state_data(run_num, self.PED_LONGIT_INDEX, self.CAR_HOZ_INDEX)
		NUM_POINTS = len(states)

		# Extract data
		for state in states:
			ped_loc_longit_points.append(state[0])
			ped_loc_horizontal_points.append(-state[1])
			car_loc_longit_points.append(state[2])
			car_loc_horizontal_points.append(-state[3])

		# Plot data here
		fig = plt.figure()
		axes = plt.axes()

		# Color settings
		alphas = np.linspace(0.1, 1, NUM_POINTS)
		rgba_colors_ped = np.zeros((NUM_POINTS, 4))
		rgba_colors_car = np.zeros((NUM_POINTS, 4))
		# for red the first column needs to be one
		rgba_colors_ped[:, 0] = 1.0
		# the fourth column needs to be your alphas
		rgba_colors_ped[:, 3] = alphas
		# Plot the trajectory of the pedestrian
		axes.scatter(ped_loc_horizontal_points, ped_loc_longit_points, color=rgba_colors_ped)
		# Plot the trajectory of the pedestrian
		rgba_colors_car[:, 2] = 1.0
		rgba_colors_car[:, 3] = alphas

		axes.scatter(car_loc_horizontal_points, car_loc_longit_points, color=rgba_colors_car)

		# Annotate plot start and end
		axes.annotate("START", (ped_loc_horizontal_points[0], ped_loc_longit_points[0]))
		axes.annotate("END", (ped_loc_horizontal_points[-1], ped_loc_longit_points[-1]))
		axes.annotate("START", (car_loc_horizontal_points[0], car_loc_longit_points[0]))
		axes.annotate("END", (car_loc_horizontal_points[-1], car_loc_longit_points[-1]))

		# Annotate time index
		for time in range(NUM_POINTS):
			axes.annotate(time, (ped_loc_horizontal_points[time], ped_loc_longit_points[time]))
			axes.annotate(time, (car_loc_horizontal_points[time], car_loc_longit_points[time]))

		# Set options for plot
		axes.grid(axis="both")
		axes.set_title("Pedestrian vs Car Trajectory Run#%d and Fretchet distance = %f" % (run_num, distance))
		axes.set_xlabel("Horizontal section of road")
		axes.set_ylabel("Longitudinal section of road")
		axes.set_autoscalex_on(True)
		axes.set_autoscaley_on(True)

		# Save plot
		#plt.show()
		logfile_basename = os.path.basename(self.filepath)
		plt.savefig(self.output_dir +"/plots/%s_traj%d.png" % (logfile_basename, run_num))
		plt.close()






def main():

	argparser = argparse.ArgumentParser(
		description= "Parser for OPPT log files for the NCAP Experiment")

	argparser.add_argument(
		'-f', '--file',
		metavar='logFile',
		default="/home/jimy/Desktop/tmp_experiments/log_ABT_Pedestrian_basic_testing_particles.log",
		type=str,
		help='path to the file to be parsed')

	argparser.add_argument(
		'-d', '--dir',
		metavar='binDir',
		default="/home/jimy/Desktop/tmp_experiments",
		type=str,
		help='path to the parent directory where results are saved')


	args = argparser.parse_args()
	filepath = args.file
	output_dir = args.dir




if __name__ == '__main__':
	main()