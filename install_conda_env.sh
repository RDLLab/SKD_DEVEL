#!/bin/bash


# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)

# Check for correct number of command line arguments
if [ $# -lt 2 ]
then
    echo "Usage : $0 --install_conda [true|false]"
    exit
fi

CONDA_OPTION=$1
case "${CONDA_OPTION}" in
-c|--install_conda)
  INSTALL_CONDA_FLAG=$2
  echo "CONDA INSTALL OPTION WAS SET TO ${INSTALL_CONDA_FLAG}"
  ;;
esac

# Check if oppt is to be installed
if (${INSTALL_CONDA_FLAG});
then
	echo "FLAG SET TO ${INSTALL_FLAG}: Installing CONDA"
	# # Install anaconda for conda environment used for the project
	cd /tmp
	curl -O https://repo.anaconda.com/archive/Anaconda3-2020.11-Linux-x86_64.sh
	bash ./Anaconda3-2020.11-Linux-x86_64.sh -b -p $HOME/anaconda
fi

# # # Create conda environment
# Start anaconda
cd ${SKD_ROOT_DIR}/skd_python/conda_envs	
conda env create -f skd_conda.yml
