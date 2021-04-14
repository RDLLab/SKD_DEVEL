import os
import shutil 
import numpy as np
import glob
import scipy.stats as st
import json
import copy
# Local class to parse oppt log files
import SKDUtils
from SKDUtils import ScenarioSituation as ScenarioSituation
import SKDGeneralScenarioAnalyser

class KamikazeExperimentsOrganizer:
	SITUATION_SYNTHETIC_DATA = 0
	SITUATION_REAL_DATA = 1

	# Constructor of the class
	def __init__(self, controller_scenarios, scenario_output_dir):
		# Copy controller scenarios
		self.controller_scenarios = copy.deepcopy(controller_scenarios)
		# Fill analyzers for each controller scenario
		self.controller_scenario_analysers = {}

		for controller_multiplier in self.controller_scenarios.keys():
			scenario_situations = self.controller_scenarios[controller_multiplier]
			controller_scenario_analyser = SKDGeneralScenarioAnalyser.OPPTGeneralGroupedScenarioAnalyser(
				scenario_situations, scenario_output_dir, controller_multiplier)
			# Save analyser to perform parsing
			self.controller_scenario_analysers[controller_multiplier] = controller_scenario_analyser


	""" Creates a scenario analyzer for each controller id examined """
	def analyze_experiments_scenarios(self,  outfilepath):
		experiments_data = []

		print(self.controller_scenarios)

		for controller_multiplier in self.controller_scenarios:
			scenario_data = [float(controller_multiplier)]
			controller_scenario_analyser = self.controller_scenario_analysers[controller_multiplier]
			scenario_data.extend(
				controller_scenario_analyser.get_scenario_frechet_stats(augmented = True, 
				sample_limit = -1, save_plot = False))
			experiments_data.append(scenario_data)
			
		# Save plots
		#controller_scenarios[controller_index].save_scenario_plots(500)
		np_experiments_data = np.array(experiments_data)
		print(np_experiments_data)


		# Header for csv
		txt_header = "multiplier_val,kamikaze_success_rate,"
		txt_header += "frechet_sample_size,fretchet_sum,frechet_means,fretchet_var,frechet_grouped_var,frechet_95_ci_low_,frechet_95_ci_high,total_attempts,total_successful"
		
		
		np.savetxt(outfilepath, np_experiments_data, delimiter=",", header=txt_header, comments='')





if __name__ == '__main__':
	compute_synthetic_data()




