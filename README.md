SKD DEVELOPMENT REPOSITORY
==========================================================================
Repository used for assessment of autonomous vehicles using SKD as a safety metric


### Repository dependencies:
- [Oppt toolkit](https://github.com/RDLLab/oppt): used for the planning of the Pedestrian's interaction with the assessed vehicle.

- [Anaconda](https://docs.anaconda.com/anaconda/install/): used for dependencies management in skd_python


### SKD Assessment Module
- skd_oppt : This module contains the necessary plugins and environment models that are necessary for the
planner used in the assessment mechanism.

- skd_python: This module contains the scripts to execute the SKD Asssement Mechanism

# Installation:

## Full installation:
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

## Individual component installation:

**Install the planner component by running**
```
skd@skd:$ cd <SKD_DEVEL_ROOT>
skd@skd:$ chmod +x oppt_install.sh && ./oppt_install.sh --install_dir <install_dir_path>
```	

**Install the skd_oppt modules**

To compile the skd_oppt modules, the shell environment must know about the oppt install location to find the oppt libraries.
To allow this, we set the env variable "oppt_DIR=<oppt_install_location>/lib/cmake/oppt" in a new terminal. 
This can also be added to the local ${HOME}/.bashrc file.	
```
# Set env vars to link with oppt share libs
skd@skd:$ export oppt_DIR=<oppt_install_location>/lib/cmake/oppt
skd@skd:$ cd <SKD_DEVEL_ROOT>
skd@skd:$ chmod +x skd_oppt_install.sh && ./skd_oppt_install.sh
```

**Install the conda environment to include all other python dependencies**
An example installation script is provided to install anaconda and create the appropiate conda environment. 
If conda is already installed in the machine, proceed to the step to create the conda environment.
```
skd@skd:$ cd <SKD_DEVEL_ROOT>
skd@skd:$ chmod +x install_conda_env.sh && ./install_conda_env.sh -c [true|false]

If anaconda is installed, create the environment directly by running the following 
inside the "skd_python" directory:
skd@skd:$ cd <SKD_DEVEL_ROOT>/skd_oppt/conda_envs
skd@skd:$ conda env create -f skd_python_env.yml
```

# Using SKD Assessment Mechanism:

## Environment resources:
Once the installation is complete, the individual stages of the SKD Assessment can be broken into:
	- Safe Trajectory Generation
	- Safe Trajectory Collision Experiments
	- Kamikaze Trajectory Generation
	- SKD Computation

Before, running any component:
- Activate the anaconda environment to account for all the needed Python Dependencies

```
# Activate conda environment """
skd@skd:$ conda init bash
skd@skd:$ conda activate skd_conda
```

- The shell program must know the path for the OPPT and GAZEBO resources required by OPPT.
These can be set by running the following in a new shell program:

```
# Source the env var file. Please note, this must be fone inside <SKD_ROOT_DIR>/skd_oppt/scripts
skd@skd:$ cd <SKD_ROOT_DIR>/skd_oppt/scripts
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


## Example usage of SKD modules:
Examples provided in notebooks under "skd_python/skd_notebooks"

