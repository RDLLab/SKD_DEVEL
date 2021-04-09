#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)
SKD_OPPT_DIR=${SKD_ROOT_DIR}/skd_oppt

## Install the plugins
cd ${SKD_OPPT_DIR}/plugins

## Create install directory
mkdir build && cd build

## Build and install OPPT
# Set flag for OPPT INSTALL MACRO
cmake -DCMAKE_INSTALL_DATA_DIR=${PWD} ..
make -j$(nproc) && make install

