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


#################################################### Pipeline for collision experiment generation ####################################
# def gen_sampled_trajectories(sampled_outpath):
#     # Define bounds for start, mid, end points of the safe trajectory
#     start_bounds = [122, 118]
#     mid_bounds = [122, 118]
#     end_bounds = [122, 118]

#     # Define number of safe trajs to generate
#     NUM_SAMPLES = 5

#     # Print sample safe trajectory sets
#     sampled_trajs = skd_traj_gens.sample_safe_trajectory_set(start_bounds, mid_bounds, end_bounds, num_trajs = NUM_SAMPLES)

#     return sampled_trajs


# Example function to generate collision_exp config
def gen_collision_exp_config(output_dir):
    output_path = output_dir + "/collision_config.yaml"
    
    # Set optional configurations
    # Safe trajectory files to consider, if empty, then GUI will allow user to select
    safe_files = []
    # Controller ids in experiment
    multiplier_ids=[0.5, 0.625, 0.75, 0.875, 1, 1.05, 1.1, 1.125, 1.15]
    # Number of collision experiments per safe trajectory
    num_runs=50
    # Maximum number of steps simulated per experiment
    max_num_steps=25
    # Maximum number of trajectories considered per safe trajectory file
    max_trajs_per_file=-1
    # Default controller behaviour type ("basic" only for now)
    car_controller_type="basic"
    
    collision_env_utils.gen_collision_experiments_config(output_path, safe_files, num_runs, max_num_steps, max_trajs_per_file,
                                car_controller_type, multiplier_ids
                                                        )
    return output_path  

def run_collision_env(output_dir, config_file):
    # Create a loader for the config file
    experiments_loader = collision_loader.CollisionExperimentLoader(output_dir, config_file)

    # Run experiments from the config file
    experiments_loader.run_collision_experiments()

    # Get collision summary stats
    summary_filepath = experiments_loader.get_summary_file_path()

    return summary_filepath


def get_collision_summary_stats(output_dir, summary_stats_file):

    """ Create a collision experiment analyser """
    collision_analyser = collision_data_analyser.CollisionExperimentDataAnalyser(summary_stats_file, output_dir)

    """ Parse the data and review it on output_dir """
    collision_analyser.get_analyzer_summary_statistics()



def run_test_collision_experiments(collisions_outputdir):
    """ Excutes all three steps of the collision experiments pipeline.
    1. Gen collision experiments config
    2. Execute collision experiments
    3. Parse collision experiments results 
    """
    print("Generating collision config file")
    config_filepath = gen_collision_exp_config(collisions_outputdir)
    print("Executing collision experiments from config file")
    summary_outpath = run_collision_env(collisions_outputdir, config_filepath)
    print("Analysing collision data")
    get_collision_summary_stats(collisions_outputdir, summary_outpath)


#################################################### Pipeline for kamikaze experiments generation ####################################
# Example function to generate kamikaze traj generation experiments
def generate_kamikaze_experiments_config(kamikaze_traj_gen_outdir):
    # Set outpath for config file (Send it to ../config for now)
    # Set options for config file
    config_outpath = kamikaze_traj_gen_outdir + "/kamikaze_traj_gen.yaml"
    controller_ids = [0.5, 0.625, 0.75, 0.875, 1, 1.05, 1.1, 1.125, 1.15]
    # Safe trajectory filepaths to consider. If empty (GUI will ask user for files)
    safe_traj_files = []
    attempts_per_file = 10
    max_traj_per_file = -1
    skd_core_utils.generate_kamikaze_configs(config_outpath, controller_ids, attempts = attempts_per_file, trajs_per_file = max_traj_per_file)
    
    # Return config outpath
    return config_outpath



# Execute the kamikaze trajecoty experiments based on configuration file
def execute_kamikaze_experiments(outputdir, kamikaze_config_filepath, planner_exec_path):
    # Create a generator to serve all the options in the configuration file
    kamikaze_generator = skd_kamikaze_traj_gen.KamikazeTrajGenerator(kamikaze_config_filepath, outputdir)
    kamikaze_generator.execute_kamikaze_traj_gen_configs(planner_exec_path)

    return kamikaze_generator.get_experiments_summary_dir()



## Example function to analyse output from kamikaze traj experiments
def analyse_experimental_output(summary_file_path, outputdir):
    analyser = skd_kamikaze_data_analyser.SKDKamikazeDataAnalyser(summary_file_path, outputdir)
    analyser.parse_summary_data()




def run_kamikaze_traj_gen_stats(kamikaze_traj_gen_outdir, planner_path):
    # Generate configuration file for experiments
    print("Generating kamikaze trajectories experiments config file")
    config_filepath = generate_kamikaze_experiments_config(kamikaze_traj_gen_outdir)

    # Execute experiments
    print("Executing kamikaze trajectories experiments")
    summary_file = execute_kamikaze_experiments(kamikaze_traj_gen_outdir, config_filepath, planner_path)

    # Analyse data
    print("Processing kamikaze trajectory generation results")
    analyse_experimental_output(summary_file, kamikaze_traj_gen_outdir)




    

if __name__ == '__main__':
    """ Entry point for assesment """
    argparser = argparse.ArgumentParser(
    description= "Main Collision Experiments")

    argparser.add_argument(
        '-o', '--outdir',
        metavar='AggregatorModuleOutpuDir',
        type=str,
        help='Parent to output directory of the module')


    # Parse arguments
    args = argparser.parse_args()
    module_outdir = args.outdir

    # Get timestampt
    timestamp = datetime.now().strftime("%m-%d-%H-%M-%S")
    module_outdir += "_%s" % (timestamp)

    try: 
        os.makedirs(module_outdir)
    except OSError as error:
        print(error)


    
    # Run collision pipeline
    t_start = time.clock()
    run_test_collision_experiments(module_outdir)
    t_end = time.clock()

    print("TOOK %f seconds for collision experiments" % ((t_end - t_start) * 1000))
    # Run kamikaze trajetory generation experiments
    planner_path = os.getenv("HOME") + "/jimy_ws/oppt/bin/abt"
    # Run kamikaze trajectory experiments
    run_kamikaze_traj_gen_stats(module_outdir, planner_path)

