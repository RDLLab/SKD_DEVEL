import yaml
import subprocess
import os
import copy
import json
import tkinter as tk
import tkinter.filedialog as fd
import numpy as np
import scipy.stats as st
import matplotlib.pyplot as plt

# Utiliy structures to convenienty pass data
class TrajectoryDataRecord:

    """ Constructor of the class. A Tracjectory Data Pair contains a kamikaze trajectory, and its equivalent 
    safe trajectory ended as a safeTrajectory object """
    def __init__(self, kamikaze_ped_traj, kamikaze_veh_traj, safe_traj):
        self.kamikaze_ped_traj = copy.deepcopy(kamikaze_ped_traj)
        self.kamikaze_veh_traj = copy.deepcopy(kamikaze_veh_traj)
        self.safe_traj = copy.deepcopy(safe_traj)

    """ Returns the kamikaze traj associated with this object """
    def get_kamikaze_ped_traj(self):
        return self.kamikaze_ped_traj

    """ Returns the safe traj as a list associated with this object """
    def get_safe_traj(self):
        return self.safe_traj

    """ Returns the kamikaze_veh_traj associated with the object """
    def get_kamikaze_veh_traj(self):
        return self.kamikaze_veh_traj


        
######################################## SKD Config Files Utilities ###########################################
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




def generate_kamikaze_configs(outpath, controller_multipliers, cfg_template_path,
                     data_files = [], attempts=2, trajs_per_file=-1):
    # Verify that files is not empty
    if(len(data_files) <= 0):
        # Ask for files 
        safe_traj_files = retrieve_safe_files("Select Safe Trajectory Files For Kamikaze Traj Gen")
    else: 
        # Copy files
        safe_traj_files = copy.deepcopy(data_files)

    kamikaze_configs = get_kamikaze_configs(controller_multipliers, safe_traj_files, 
        cfg_template_path, attempts, trajs_per_file)

    # Save file
    with open(outpath, 'w+') as kamikaze_config_file:
        documents = yaml.dump(kamikaze_configs, kamikaze_config_file)
        print("KAMIKAZE CONFIGS GENERATED TO %s" % (outpath))
        print(kamikaze_configs)



def get_kamikaze_configs(controller_multipliers, files, cfg_template_path, attempts = 2, trajs_per_file=-1):
    """ Generates a local configuration file for input to the Kamikaze Traj Generator Module """
  
    # Local config for kamikaze trajectory generation
    kamikaze_traj_configs = {
            "safe_traj_files" : files,
            "attempts_per_goal" : attempts,
            "controller_multipliers" : controller_multipliers,
            "max_trajs_per_file" : trajs_per_file,
            "kamikaze_cfg_file" : cfg_template_path
    }
    return kamikaze_traj_configs


def get_safe_trajs_config(goal_bounds, initial_state, cfg_template_path, attempts=2):
    """ Generates a local configuration file for input to the Kamikaze Traj Generator Module """
    safe_traj_configs = {
            "goal_bounds" : goal_bounds,
            "safe_trajs_attempts_per_goal" : attempts,
            "initial_state" : initial_state, 
            "safe_gen_cfg_file" : cfg_template_path
    }

    return safe_traj_configs




def get_kamikaze_config_suffix(controller_multiplier, safe_traj_filekey, safe_traj_index):
        kamikaze_config_suffix = "controller_m_%s/%s/ST_%d" % (controller_multiplier, safe_traj_filekey, safe_traj_index) 
        return kamikaze_config_suffix



####################################### Utilities for safe trajectories management ##############################################

def retrieve_safe_files(message):
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


def get_num_safe_trajs(filename):
    """ Function to read the number of safe trajectories in a given safe trajectory file"""
    with open(filename) as safe_traj_file:
        safe_traj_data = json.load(safe_traj_file)
        return len(safe_traj_data.keys())


def get_safe_traj_from_file(safe_traj_file_path, key_index):
    """ Returns the safe ped trajectory associated with this object as a list of lists(2D points) """    
    ped_traj_data_json = load_dict_from_json(safe_traj_file_path)

    
    return ped_traj_data_json[str(key_index)]



################################# Utilities to process the general statistics of an array of numbers ##############

def process_general_stats(data_array):
    # Use numpy to compute the statistics
    np_data_array = np.array(data_array)
    np_data_sum = sum(data_array)
    np_data_size = len(data_array)
    np_data_mean = np.mean(data_array)
    np_data_var = np.var(data_array, ddof = 1)
    np_ci =  st.norm.interval(alpha=0.95, loc=np_data_mean, scale=st.sem(np_data_array))

    data_summary = {"DATA_ARRAY" : copy.deepcopy(data_array),
            "DATA_SUM" : np_data_sum,
            "DATA_SIZE" : np_data_size,
            "DATA_MEAN" : np_data_mean,
            "DATA_VAR" : np_data_var,
            "DATA_CI_LOW" : np_ci[0],
            "DATA_CI_HIGH" : np_ci[1]}

    return data_summary


def process_general_stats_array(data_array):   
    # Use numpy to compute the statistics
    np_data_array = np.array(data_array)
    np_data_sum = sum(data_array)
    np_data_size = len(data_array)
    np_data_mean = np.mean(data_array)
    np_data_var = np.var(data_array, ddof = 1)
    np_ci =  st.norm.interval(alpha=0.95, loc=np_data_mean, scale=st.sem(np_data_array))

    return [np_data_size, np_data_sum, np_data_mean, np_data_var, np_ci[0], np_ci[1]]



#################################### GENERAL UTILITIES ########################################
def list_to_str(list_var):
        result = "["
        for val in list_var:
            result = result + " %s" % (val)
        result = result + " ]"
        return result


def save_dict_to_yaml(dict_value, outpath):
    with open(outpath, "w+") as yaml_outfile:
        yaml.dump(dict_value, yaml_outfile)


def load_dict_from_yaml(load_yaml_path):
    with open(load_yaml_path) as load_yaml_file:
        dict_value = yaml.full_load(load_yaml_file)
        
        return dict_value



def save_dict_to_json(dict_value, outpath):
    with open(outpath, "w+") as json_outfile:
        json.dump(dict_value, json_outfile, indent = 4)



def load_dict_from_json(load_json_file):
    with open(load_json_file) as json_file:
        dict_value = json.load(json_file)
        return dict_value

def sed_file(filepath, old_txt, new_txt):
    subprocess.call(["sed -i 's@%s@%s @' %s" % (old_txt, new_txt, filepath)], shell = True)




""" Loads a csv table represetnation of the run tracker data and plots the trajectory of 
    both the pedestrian and the car involved in the data """
def save_plot_number(run_num, exp_states, output_path, status):
    # Pack information from pedestrian and car's location
    ped_loc_longit_points = []
    ped_loc_horizontal_points = []
    car_loc_longit_points = []
    car_loc_horizontal_points = []

    # Get data from run file
    states = copy.deepcopy(exp_states)
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


    # Annotate time index
    for time in range(NUM_POINTS):
        axes.annotate(time, (ped_loc_horizontal_points[time], ped_loc_longit_points[time]))
        axes.annotate(time, (car_loc_horizontal_points[time], car_loc_longit_points[time]))

    # Set options for plot
    axes.grid(axis="both")
    axes.set_title("Pedestrian vs Car Trajectory Run#%d, Collided %r" % (run_num, status))
    axes.set_xlabel("Horizontal section of road")
    axes.set_ylabel("Longitudinal section of road")
    axes.set_autoscalex_on(True)
    axes.set_autoscaley_on(True)

    # Save plot
    plt.savefig(output_path +"/plot_%d.png" % (run_num), transparent=False)
    plt.close()




""" Loads a csv table represetnation of the run tracker data and plots the trajectory of 
    both the pedestrian and the car involved in the data """
def save_trajectories_plot(run_num, ped_safe_traj, ped_kamikaze_traj, veh_traj,  title, output_dir):
    # Pack information from pedestrian and car's location
    ped_safe_loc_longit_points = []
    ped_safe_loc_hoz_points = []
    ped_kamikaze_loc_longit_points = []
    ped_kamikaze_loc_hoz_points = []
    car_loc_longit_points = []
    car_loc_horizontal_points = []

    # Get data from run file
    NUM_POINTS = min(len(ped_safe_traj), len(veh_traj), len(ped_kamikaze_traj))

    # Extract data
    for index_val in range(NUM_POINTS):
        ped_safe_point = ped_safe_traj[index_val]
        ped_kamikaze_point = ped_kamikaze_traj[index_val]
        veh_point = veh_traj[index_val]

        # Append in reverse order to look like scene
        ped_safe_loc_longit_points.append(ped_safe_point[0])
        ped_safe_loc_hoz_points.append(-ped_safe_point[1])
        ped_kamikaze_loc_longit_points.append(ped_kamikaze_point[0])
        ped_kamikaze_loc_hoz_points.append(-ped_kamikaze_point[1])
        car_loc_longit_points.append(veh_point[0])
        car_loc_horizontal_points.append(-veh_point[1])

    # Plot data here
    fig = plt.figure()
    axes = plt.axes()

    # Color settings
    alphas = np.linspace(0.1, 1, NUM_POINTS)
    rgba_colors_ped_safe = np.zeros((NUM_POINTS, 4))
    rgba_colors_ped_kamikaze = np.zeros((NUM_POINTS, 4))
    rgba_colors_car = np.zeros((NUM_POINTS, 4))
    # plot blue for safe traj
    rgba_colors_ped_safe[:, 2] = 1.0
    # the fourth column needs to be your alphas
    rgba_colors_ped_safe[:, 3] = alphas
    axes.scatter(ped_safe_loc_hoz_points, ped_safe_loc_longit_points, color=rgba_colors_ped_safe)
    # Plot the safe traj with red
    rgba_colors_ped_kamikaze[:, 0] = 1.0
    rgba_colors_ped_kamikaze[:, 3] = alphas
    # Plot the trajectory of the pedestrian
    axes.scatter(ped_kamikaze_loc_hoz_points, ped_kamikaze_loc_longit_points, color=rgba_colors_ped_kamikaze)
    # Plot the veh traj with green 
    rgba_colors_car[:, 1] = 1.0
    rgba_colors_car[:, 3] = alphas

    axes.scatter(car_loc_horizontal_points, car_loc_longit_points, color=rgba_colors_car)


    # Annotate time index
    for time in range(NUM_POINTS):
        axes.annotate(time, (ped_safe_loc_hoz_points[time], ped_safe_loc_longit_points[time]))
        axes.annotate(time, (ped_kamikaze_loc_hoz_points[time], ped_kamikaze_loc_longit_points[time]))
        axes.annotate(time, (car_loc_horizontal_points[time], car_loc_longit_points[time]))

    # Set options for plot
    axes.grid(axis="both")
    axes.set_title(title)
    axes.set_xlabel("Horizontal section of road")
    axes.set_ylabel("Longitudinal section of road")
    axes.set_autoscalex_on(True)
    axes.set_autoscaley_on(True)

    # Save plot
    plt.savefig(output_dir +"/plot_%d.png" % (run_num), transparent=False)
    plt.close()







#################### HELPER FUNCTIONS SPECIFIC TO OPPT ###############################################
def get_oppt_log_filename(log_post_fix):
    """ Return the string of oppt log file outputted by the generator """
    return "log_ABT_Pedestrian_" + log_post_fix + ".log"