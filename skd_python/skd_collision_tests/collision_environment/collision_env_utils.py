import yaml
import subprocess
import os
import copy
import json
import tkinter as tk
import tkinter.filedialog as fd
import matplotlib.pyplot as plt
import numpy as np


""" Loads a yaml file specified by the yaml_file_path and returns a dictionary with the parsed information"""
def load_yaml_file(yaml_file_path):
    with open(yaml_file_path) as config_file:
        configurations = yaml.full_load(config_file)
        print(configurations)
        return configurations




""" Returns the safe ped trajectory associated with this object as a list of lists(2D points) """
def get_all_safe_trajs_json(safe_traj_file_path):
    data_file = open(safe_traj_file_path)
    ped_traj_data_json = json.load(data_file)
    data_file.close()
    # Save safe trajectories here
    file_safe_trajs = []

    for traj_key in ped_traj_data_json:
        file_safe_trajs.append(ped_traj_data_json[traj_key])

    return file_safe_trajs



####################################### Collision experiment configuration file generation methods ################################
def gen_collision_experiments_config(output_path, safe_traj_files = [], num_runs=25, max_num_steps=25, max_trajs_per_file=-1,
	car_controller_type="basic", multiplier_ids=[0.5, 0.625, 0.75, 0.875, 1, 1.05, 1.10, 1.125, 1.15]):
    
    # Ask for files 
    if(len(safe_traj_files) < 1):
        safe_traj_files = ask_for_files("No safe traj files given. Select the safe trajectories to be examined")

    # Pack into dictionary
    experiments_config = {"multiplier_ids" : multiplier_ids, 
                            "car_controller_type" : car_controller_type,
   							"num_runs" : num_runs, 
                            "max_num_steps" : max_num_steps,
                            "safe_trajectory_files" : safe_traj_files,
                            "max_trajs_per_file" : max_trajs_per_file}

    # Ensure output dir exist
    outputdir = os.path.dirname(output_path)

    # Create dir 
    try:
        os.makedirs(outputdir)
    except OSError as error:
        print(error)
          

    with open(output_path, 'w') as experiments_config_file:
        documents = yaml.dump(experiments_config, experiments_config_file)
        print(experiments_config)




####################################### HELPER METHODS FOR UTILITIES ##############################################
def ask_for_files(message):
    """ Uses a GUI selection of experiment directories to be considered by an instance 
    of this class """
    root = tk.Tk()
    files = []
    
    while True:
        new_files = fd.askopenfilenames(parent=root, title=message) 
        if(len(new_files) <= 0):
            break
        else:
            files.extend(new_files)
    root.destroy()
    return files


