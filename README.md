SKD DEVELOPMENT REPOSITORY
==========================================================================
Repository used for assessment of autonomous vehicles usig SKD as a safety metric


## Repository breakdown:

### Planner Component (OPPT v5)
The opptv5 module contains the local version of the toolkit [OPPT](https://github.com/RDLLab/oppt) used for the planning of the Pedestrian's interaction with the assessed vehicle.

### SKD Assessment Module
- skd_oppt : This module contains the necessary plugins and environment models that are necessary for the
planner used in the assessment mechanism.
- skd_python: This module contains the scripts to execute the SKD Asssement Mechanism

### Installation:
The whole system installation can be run in one script by running the install script with the following options:
```
# Inside the repository root (e.g SKD_DEVEL)
skd@skd:$ chmod +x skd_full_install.sh && ./skd_full_install.sh -i true -d ${PWD}/opptv5/src/build -c true

# The script has the following options
[-i | --install_oppt] set to [true|false]: This argument defines if the "OPPT" toolkit should be installed as part of the installtion process.
[-d | --oppt_dir] set to the [oppt_install_path]: If oppt installation is required, this argument specifies the installation location of oppt. 
Note, the automated script assumes installation of oppt is outside a system directory. For system installation, make sure to use sudo.
[-c | --install_conda] set to [true|false]: This argument defines if "Anaconda" is to be installed as part of the installation process.
If set to false, it is assumed that anaconda is installed. The installation process will attempt to install an anaconda environment.
```

### Individual component installation
#### Install the planner component by running 
```
skd@skd:$ cd <SKD_DEVEL_ROOT>
skd@skd:$ chmod +x oppt_install.sh && ./oppt_install.sh --install_dir <install_dir_path>
```	

#### Install the skd_oppt modules by running

To compile the skd_oppt modules, the shell environment must know about the oppt install location to find the oppt libraries.
To allow this, we set the env variable "oppt_DIR=<oppt_install_location>/lib/cmake/oppt" in a new terminal. 
This can also be added to the local ${HOME}/.bashrc file.	
```
Set env vars to link with oppt share libs
skd@skd:$ export oppt_DIR=<oppt_install_location>/lib/cmake/oppt
skd@skd:$ cd <SKD_DEVEL_ROOT>
skd@skd:$ chmod +x skd_oppt_install.sh && ./skd_oppt_install.sh
```

#### Install the conda environment to include all other python dependencies. 
An example installation script is provided to install anaconda and create the appropiate conda environment. 
If conda is already installed in the machine, proceed to the step to create the conda environment.
```
skd@skd:$ cd <SKD_DEVEL_ROOT>
skd@skd:$ chmod +x install_conda_env.sh && ./install_conda_env.sh -c [true|false]

If anaconda is installed, create the environmen directly by running the following 
inside the "skd_python" directory:
skd@skd:$ cd <SKD_DEVEL_ROOT>/skd_oppt/conda_envs
skd@skd:$ conda env create -f skd_python_env.yml
```

#### Running the SKD Assessment Mechanism
Once the installation is complete, the individual stages of the SKD Assessment can be broken into:
	- Safety Trajectory Generation
	- Kamikaze Trajectory Generation
	- SKD Computation

Before, running any component:
- Activate the anaconda environment to account for all the needed Python Dependencies
- The shell program must know the path for the OPPT and GAZEBO resources required by OPPT.
These can be set by running the following in a new shell program:
```
""" Activate conda environment """
skd@skd:$ conda init bash
skd@skd:$ conda activate skd_conda
```

```
""" Go to skd_oppt_module """
skd@skd:$ cd <SKD_ROOT_DIR>/skd_oppt/scripts
# Source the env var file. Please note, this must be fone inside <SKD_ROOT_DIR>/skd_oppt/scripts
skd@skd:$ source skd_env_vars.sh
```

The environment variables in this script can also be added to the local ${HOME}/.bashrc, as
```
# Source the oppt environment resources
skd@skd:$ source OPPT_INSTALL_DIR=<oppt_install_dir>/share/oppt/setup.bash

# Source the gazebo environment resources
skd@skd:$ source /opt/ros/melodic/setup.sh  #Melodic correspond to the ROS version for Ubuntu 18.04 


# Required environmen resources
# Models and plugins are on "skd_oppt/" with the default installation. Modify according to installation paths
skd@skd:$ export SKD_MODELS=<SKD_DEVEL_ROOT>/skd_oppt/models
skd@skd:$ export SKD_PLUGINS_PATH=<SKD_DEVEL_ROOT>/skd_oppt/plugins/build/oppt/plugins
skd@skd:$ export OPPT_RESOURCE_PATH=${OPPT_RESOURCE_PATH}:${SKD_MODELS}:${SKD_PLUGINS_PATH}
skd@skd:$ export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SKD_MODELS}

""" No uri lookup for online models """
skd@skd:$ export GAZEBO_MODEL_DATABASE_URI=
```


To run the whole system in a single program, 
```
# Inside skd_oppt run 
skd@skd:$ chmod +x run_skd_assessment.sh && ./run_skd_assessment.sh
```

Inside the "run_skd_assessment.sh" script, the environment resources are loaded, and a new directory is created for the output of 
the experiments. An example usage for running the whole assessment can be seen inside the "run_skd_assessment.sh"
```
# Run the assessment system and output into an experiments dir
skd@skd:$ cd ${SKD_ROOT_DIR}/skd_python
skd@skd:$ mkdir experiments

# Run the SKD Assessment 
skd@skd:$ python3 SKDAggregator.py -o ${PWD}/experiments -cfg config/config.yaml -oppt <abt_path>
```



This script will set the corresponding environment and model paths for the planner, and launch the assessment program:
```
# Contents of run_skd_assessment.sh

# Get the name of the directory where the repository is located
skd@skd:$ SKD_ROOT_DIR=$(cd `dirname $0` && pwd)

################### LAUNCH ASSESSMENT MECHANISM ####################
# Run the assessment system and output into an experiments dir
# Get the name of the directory where the repository is located
skd@skd:$ cd ${SKD_ROOT_DIR}/skd_python
skd@skd:$ mkdir experiments

# Run the SKD Assessment 
skd@skd:$ python3 SKDAggregator.py -o ${PWD}/experiments -cfg config/config.yaml -oppt ~/SKD_DEVEL/opptv5/bin/abt 

```

### Running Trajectory Generation Mechanism
The trajectory generation components can also be run in separate separately. 
To do so, make sure that the appropiate enviroment variables are set as explained 
in "Running the SKD Assessment Mechanism". Then, to run each module:
```
# For Safe Traj Generation
skd@skd:$ mkdir SafeTrajOutDir
skd@skd:$ python3 SKDSafeTrajGenerator.py -o ${PWD}/SafeTrajOutDir -cfg config/config.yaml

# For Kamikaze Traj Generation
skd@skd:$ mkdir KamikazeTrajOutDir
skd@skd:$ python3 SKDKamikazeTrajGenerator.py -o ${PWD}/KamikazeTrajOutDir -cfg config/config.yaml

```



