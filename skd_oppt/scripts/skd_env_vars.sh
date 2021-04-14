#!/bin/bash

# Load appropiate environment variables
# Get the name of the directory where the repository is located
ENVSDIR=`pwd`

# Source GAZEBO ENV VARS
source /opt/ros/melodic/setup.sh

# Source oppt ENV VARS
source /usr/local/share/oppt/setup.sh


# Reference dir to locate all models and plugins	
SKD_OPPT_DIR=${ENVSDIR}/..


# Export environment variables so planner can load models
export SKD_MODELS=${SKD_OPPT_DIR}/models
export SKD_PLUGINS_PATH=${SKD_OPPT_DIR}/plugins/build/oppt/plugins
export OPPT_RESOURCE_PATH=${OPPT_RESOURCE_PATH}:${SKD_MODELS}:${SKD_PLUGINS_PATH}
export GAZEBO_MODEL_PATH=${SKD_MODELS}:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_DATABASE_URI=



