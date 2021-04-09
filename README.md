
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
chmod +x oppt_install.sh && ./oppt_install.sh
```

Install the skd_oppt modules by running
```
chmod +x skd_oppt_install.sh && ./skd_oppt_install.sh
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


