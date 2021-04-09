#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)
OPPT_DIR=${SKD_ROOT_DIR}/opptv5

## Go to oppt directory
cd ${OPPT_DIR}

## Runt the install dependencies script
chmod +x install_dependencies.sh && ./install_dependencies.sh --use-ros

# Source ros-kinetic before installing oppt
source /opt/ros/kinetic/setup.sh

## Build and install OPPT
cd ${OPPT_DIR}/src/

# Check if the build folder doesn't exist yet
if [ ! -d "build" ]
then
   mkdir build
fi
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc) && make install

