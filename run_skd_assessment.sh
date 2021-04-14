#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)


# # Set environment and agensts models for use in oppt
# cd ${SKD_ROOT_DIR}/skd_oppt/scripts
# # Source from inside dir 
# source skd_env_vars.sh

# Run the assessment system and output into an experiments dir
# Get the name of the directory where the repository is located
cd ${SKD_ROOT_DIR}/skd_python
mkdir experiments

# Run the SKD Assessment 
python3 SKDAggregator.py -o ${PWD}/experiments -cfg config/config.yaml -oppt ~/SKD_DEVEL/opptv5/bin/abt 

