#!/bin/bash

# Get the name of the dir where the script is located
SCRIPTDIR=$(cd `dirname $0` && pwd)
SKD_OPPT_DIR=${SCRIPTDIR}/..

export SKD_MODELS=${SKD_OPPT_DIR}/models
export SKD_PLUGINS_PATH=${SKD_OPPT_DIR}/plugins/build/oppt/plugins
export OPPT_RESOURCE_PATH=${OPPT_RESOURCE_PATH}:${SKD_MODELS}:${SKD_PLUGINS_PATH}
export GAZEBO_MODEL_PATH=${SKD_MODELS}:${GAZEBO_MODEL_PATH}



