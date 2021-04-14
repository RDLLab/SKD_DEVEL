
SKD DEVELOPMENT REPOSITORY
==========================================================================
Repository used for assessm
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
Install the planner component by running 
```
skd@skd:$ chmod +x oppt_install.sh && ./oppt_install.sh
```

Install the skd_oppt modules by running
```
skd@skd:$ chmod +x skd_oppt_install.sh && ./skd_oppt_install.sh
```

Install the conda environment to include all other python dependencies. An example installation script is provided
to install anaconda and create the appropiate conda environment. If conda is already installed in the machine, 
proceed to the step to create the conda environment.
```
skd@skd:$ chmod +x install_conda_env.sh && ./install_conda_env.sh

# If anaconda is installed, create the environmen directly by running the following inside the "skd_python" directory 
skd@skd:$ conda env create -f skd_python_env.yml
```

### Running the SKD Assessment Mechanism
To SKD Assessment is broken into three main stages. 
- Safety Trajectory Generation
- Kamikaze Trajectory Generation
- SKD Computation

To run the whole system in a single program, 
```
# Inside skd_oppt run 
skd@skd:$ chmod +x run_skd_assessment.sh && ./run_skd_assessment.sh
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
To do so, make sure that the appropiate enviroment variables are set. This can be done by running the following steps from "run_skd_assessment.sh":
```
################### SET ENVIRONMENT RESOURCES ####################
# # Set environment and agensts models for use in oppt
skd@skd:$ cd ${SKD_ROOT_DIR}/skd_oppt/scripts
# Source from inside dir 
skd@skd:$ source skd_env_vars.sh
```

Then, to run each module:
```
# For Safe Traj Generation
mkdir SafeTrajOutDir
python3 SKDSafeTrajGenerator.py -o ${PWD}/SafeTrajOutDir -cfg config/config.yaml

# For Kamikaze Traj Generation
mkdir KamikazeTrajOutDir
python3 SKDKamikazeTrajGenerator.py -o ${PWD}/KamikazeTrajOutDir -cfg config/config.yaml

```


