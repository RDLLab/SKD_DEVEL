#!/bin/bash


# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)


# # Install anaconda for conda environment used for the project
cd /tmp
curl -O https://repo.anaconda.com/archive/Anaconda3-2019.03-Linux-x86_64.sh
bash ./Anaconda3-2019.03-Linux-x86_64.sh -b -p $HOME/anaconda

# Start anaconda
cd ${HOME}/anaconda/bin && conda init
source ${HOME}/.bashrc


# Create conda environment
cd ${SKD_ROOT_DIR}/skd_python/conda_envs	
conda env create -f skd_conda.yml
conda activate skd_conda

