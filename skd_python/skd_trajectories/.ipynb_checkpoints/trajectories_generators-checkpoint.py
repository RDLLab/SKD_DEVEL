import os, sys

# Import parent dir
cwd = os.getcwd()
skd_python_dir = os.path.dirname(cwd)
skd_collision_experiments_dir = skd_python_dir + "/skd_collision_experiments"
# Append to python path
if(skd_collision_experiments_dir not in sys.path):
    sys.path.append(skd_collision_experiments_dir)

# # Import local libraries
# import controllers.pedestrian_controllers as pedestrian_controllers

# # Import third party libs
import numpy as np
import json
import copy
import matplotlib.pyplot as plt

################################# GLOBAL FUNCTIONS ##################################
""" Clamps a value """
def clamp(value, min_val, max_val):
    # Assert parameters are right
    func_min = min_val
    func_max = max_val

    # Swap in case order was reversed
    if(func_min > func_max):
        func_max = min_val
        func_min = max_val

    # Check which to return 
    if (value < func_min):
        return func_min
    elif (value > func_max):
        return func_max

    # Value is not capped. Return as it is
    return value
######################################################################################

def sample_safe_trajectory_set(
    start_point_longit_bounds, contact_point_longit_bounds, end_point_longit_bounds, 
    num_trajs = 10, steps_half1=10, steps_half2=4):

    """ Returns a set of sampled trajectories as defined by the given options"""

    # Ensure input is of correct format
    assert(len(start_point_longit_bounds) == 2 ), "Starting Point Bounds Incorrect Format"
    assert(len(contact_point_longit_bounds) == 2), "Contact Point Bounds Incorrect Format"
    assert(len(end_point_longit_bounds) == 2), "End Point Bounds Incorrect Format"  

    trajectories_db = []

    start_points = np.random.uniform(start_point_longit_bounds[0], start_point_longit_bounds[1], num_trajs)
    contact_points = np.random.uniform(contact_point_longit_bounds[0], contact_point_longit_bounds[1], num_trajs)
    end_points = np.random.uniform(end_point_longit_bounds[0], end_point_longit_bounds[1], num_trajs)

    # Sample num_trajs samples
    for sample_num in range(num_trajs):
        sampled_traj = sample_safe_trajectory(start_points[sample_num], contact_points[sample_num], end_points[sample_num], steps_half1, steps_half2)
        trajectories_db.append(sampled_traj)
        print("Plotting sampled trajectory number %d" % (sample_num))
       # plot_2D_traj(sampled_traj, "Sampled safe traj num %d" % (sample_num))

    # Return the whole database
    return trajectories_db



""" Loads a csv table represetnation of the run tracker data and plots the trajectory of 
    both the pedestrian and the car involved in the data """
def plot_2D_traj(trajectory, plot_title):
    TRAJ_LONGIT = 0
    TRAJ_HOZ = 1

    traj_len = len(trajectory)
    traj_hoz = []
    traj_longit = []

    # Separate axis values to plot
    for traj_point_index in range(traj_len):
        traj_point = trajectory[traj_point_index]
        # Append the x-axis values for the plot
        traj_hoz.append(traj_point[TRAJ_HOZ])
        traj_longit.append(traj_point[TRAJ_LONGIT])


    # Plot data here
    fig = plt.figure()
    axes = plt.axes()

    # Color settings
    alphas = np.linspace(0.1, 1, traj_len)
    rgba_colors_traj = np.zeros((traj_len, 4))
    # for red the first column needs to be one
    rgba_colors_traj[:, 2] = 1.0
    # the fourth column needs to be your alphas
    rgba_colors_traj[:, 3] = alphas
    # Plot the trajectory of the pedestrian
    axes.scatter(traj_hoz, traj_longit, color=rgba_colors_traj)

    # Annotate time index
    for time in range(traj_len):
        axes.annotate(time, (traj_hoz[time], traj_longit[time]))

    # Set options for plot
    axes.grid(axis="both")
    axes.set_title(plot_title)
    axes.set_xlabel("Horizontal section of road")
    axes.set_ylabel("Longitudinal section of road")
    plt.xlim(-5, 5)
    plt.ylim(110,130)

    # Save plot
    plt.show()
    plt.close()




def plot_trajectory_2D_set(trajectory_set, title=""):
    for trajectory_num in range(len(trajectory_set)):
        title = title + ", Traj Num %d " % (trajectory_num)
        plot_2D_traj(trajectory_set)




""" Models a basic car controller """
def sample_safe_trajectory(start_point, contact_point, end_point, num_steps_half1, num_steps_half2):
    # Paritition space in half to sample
    first_half_path = np.linspace([start_point, 4], [contact_point, -2], num_steps_half1).tolist()
    second_half_path = np.linspace([contact_point, -2], [end_point, -4], num_steps_half2).tolist()
   
    # Sampled path 
    sample_path = first_half_path
    sample_path.extend(second_half_path)
    
    # Return the sampled path
    return sample_path


def save_trajs_to_json(trajectories_db, save_path):
    """ Save the list of trajectories, where a trajectory is a list of 2D lists,
    into the specfied save_path """
    json_trajs = {}

    for traj_num in range(len(trajectories_db)):
        json_trajs[str(traj_num)] = trajectories_db[traj_num]
        

    # Save trajectories into a json file
    with open(save_path, "w+") as trajs_save_file:
        json.dump(json_trajs, trajs_save_file, indent = 4)






def main():
    start_bounds = [-1, 1]
    mid_bounds = [-1, 1]
    end_bounds = [-1, 1]

    # Print sample safe trajectory sets
    print(sample_safe_trajectory_set(start_bounds, mid_bounds, end_bounds, num_trajs=10))






if __name__ == '__main__':
    main()
