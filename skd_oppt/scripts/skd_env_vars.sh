#!/bin/bash
#!/bin/bash



# Load appropiate environemnt variables

# Source GAZEBO ENV VARS
source /opt/ros/melodic/setup.sh

# Source oppt ENV VARS
source /usr/local/share/oppt/setup.sh


SCRIPTDIR=`pwd`
echo $SCRIPTDIR
	
SKD_OPPT_DIR=${SCRIPTDIR}/..

export SKD_MODELS=${SKD_OPPT_DIR}/models
export SKD_PLUGINS_PATH=${SKD_OPPT_DIR}/plugins/build/oppt/plugins
export OPPT_RESOURCE_PATH=${OPPT_RESOURCE_PATH}:${SKD_MODELS}:${SKD_PLUGINS_PATH}
export GAZEBO_MODEL_PATH=${SKD_MODELS}:${GAZEBO_MODEL_PATH}



