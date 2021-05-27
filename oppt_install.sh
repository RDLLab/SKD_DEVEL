#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)

if [ $# -lt 2 ]
then
    echo "Usage : $0 --install_dir install_dir_path"
    exit
fi

INSTALL_DIR_OPTION=$1
case "${INSTALL_DIR_OPTION}" in
-c|--install_dir)
  INSTALL_DIR=$2
  echo "Installing OPPT Into" ${INSTALL_DIR}
  ;;
esac

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
# Install inside build directory
cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ..
make -j$(nproc - 1) && make install

