#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)


# Install mlpack dependencies
cd ${SKD_ROOT_DIR}
tar -xvzpf mlpack-3.4.2.tar.gz
mkdir mlpack-3.4.2/build && cd mlpack-3.4.2/build
cmake -D USE_OPENMP=OFF ../
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
cmake -DCMAKE_INSTALL_DATADIR=${PWD} ..
make -j$(nproc) && make install


