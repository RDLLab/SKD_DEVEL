#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)


# Install mlpack dependencies
cd ${SKD_ROOT_DIR}
tar -xvzpf mlpack-3.4.2.tar.gz
mkdir mlpack-3.4.2/build && cd mlpack-3.4.2/build
cmake ../
make -j$(nproc) && sudo make install


# Install SKD PLUGINS FOR OPPT
SKD_OPPT_DIR=${SKD_ROOT_DIR}/skd_oppt
## Install the plugins	
cd ${SKD_OPPT_DIR}/plugins

## Create install directory
mkdir build
cd build

## Build and install OPPT
# Set flag for OPPT INSTALL MACRO
cmake -DCMAKE_INSTALL_DATA_DIR=${PWD} ..
make -j$(nproc) && make install


# INSTALL ANACONDA FOR PYTHON DEPENDENCIES IN ENV
cd /tmp
#curl -O https://repo.anaconda.com/archive/Anaconda3-2019.03-Linux-x86_64.sh
bash ./Anaconda3-2019.03-Linux-x86_64.sh -b -p $HOME/anaconda
source ${HOME}/.bashrc
cd ${HOME}/anaconda/bin && conda init
source ${HOME}/.bashrc


# Ready to launch

