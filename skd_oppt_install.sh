#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)
SKD_OPPT_DIR=${SKD_ROOT_DIR}/skd_oppt

# Install mlpack dependency

# Go to SKD_ROOT_DIR TO INSALL MLPACK
cd ${SKD_ROOT_DIR}
wget https://www.mlpack.org/files/mlpack-3.1.1.tar.gz
tar -xvzpf mlpack-3.1.1.tar.gz
mkdir mlpack-3.1.1/build && cd mlpack-3.1.1/build
cmake ../
make -j$(nproc) && sudo make install

## Install the plugins
cd ${SKD_OPPT_DIR}/plugins

## Create install directory
mkdir build && cd build

## Build and install OPPT
# Set flag for OPPT INSTALL MACRO
cmake -DCMAKE_INSTALL_DATA_DIR=${PWD} ..
make -j$(nproc) && make install

