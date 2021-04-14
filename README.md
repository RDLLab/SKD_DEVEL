
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
This installation has been tested on a clean Ubuntu 18.04.5 LTS machine.

- Install the planner component by the following from the repository root dir
```
chmod +x oppt_install.sh && ./oppt_install.sh
```

- Install the skd_oppt modules by running from the repository root dir
```
chmod +x skd_oppt_install.sh && ./skd_oppt_install.sh
```

- This project requires anaconda to install the conda environment with all the python dependencies required.
This repository provides an example install script to install anaconda and create the associated anaconda environment.


### Running the SKD Assessment Mechanism
To SKD Assessment is broken into three main stages. 
	- Safety Trajectory Generation
	- Kamikaze Trajectory Generation
	- SKD Computation

To run the whole system in a single program, 
```
# Inside skd_oppt run 
chmod +x run_skd_assessment.sh && ./run_skd_assessment.sh
```

### Running Trajectory Generation Mechanism
The trajectory generation components can also be run in separate such as:
```
# For Safe Traj Generation
mkdir SafeTrajOutDir
python3 SKDSafeTrajGenerator.py -o ${PWD}/SafeTrajOutDir -cfg config/config.yaml

# For Kamikaze Traj Generation
mkdir KamikazeTrajOutDir
python3 SKDKamikazeTrajGenerator.py -o ${PWD}/KamikazeTrajOutDir -cfg config/config.yaml

```

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
chmod +x oppt_install.sh && ./oppt_install.sh
```

Install the skd_oppt modules by running
```
chmod +x skd_oppt_install.sh && ./skd_oppt_install.sh
```

Install the conda environment to include all other python dependencies. An example installation script is provided
to install anaconda and create the appropiate conda environment. If conda is already installed in the machine, 
proceed to the step to create the conda environment.
```
chmod +x install_conda_env.sh && ./install_conda_env.sh

# If anaconda is installed, create the environmen directly by running the following inside the "skd_python" directory 
conda env create -f skd_python_env.yml
```

### Running the SKD Assessment Mechanism
To SKD Assessment is broken into three main stages. 
	- Safety Trajectory Generation
	- Kamikaze Trajectory Generation
	- SKD Computation

To run the whole system in a single program, 
```
# Inside skd_oppt run 
chmod +x run_skd_assessment.sh && ./run_skd_assessment.sh
```

### Running Trajectory Generation Mechanism
The trajectory generation components can also be run in separate such as:
```
# For Safe Traj Generation
mkdir SafeTrajOutDir
python3 SKDSafeTrajGenerator.py -o ${PWD}/SafeTrajOutDir -cfg config/config.yaml

# For Kamikaze Traj Generation
mkdir KamikazeTrajOutDir
python3 SKDKamikazeTrajGenerator.py -o ${PWD}/KamikazeTrajOutDir -cfg config/config.yaml

```


