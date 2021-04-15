#!/bin/bash

# Get the name of the directory where the repository is located
SKD_ROOT_DIR=$(cd `dirname $0` && pwd)


# Ensure that the commandline arguments are used
if [ $# -lt 6 ]
then
    echo "Usage : $0 --install_oppt install_dir_path [true|false] --oppt_install_dir [oppt_install_path] --install_conda [true|false]"
    exit
fi


# Assign the options
OPPT_OPTION=$1
OPPT_INSTALL_OPTION=$3
CONDA_OPTION=$5

# Get the whole install procedure
case "${OPPT_OPTION}" in
-i|--install_oppt)
  INSTALL_OPPT=$2
  echo "OPPT INSTALL OPTION WAS SET TO ${INSTALL_OPPT}"
  ;;
esac

case "${OPPT_INSTALL_OPTION}" in
-d|--oppt_install_dir)
  OPPT_INSTALL_DIR=$4
  echo "INSTALL_DIR is $OPPT_INSTALL_DIR"
  echo "OPPT INSTALL LOCATION WAS SET TO ${OPPT_INSTALL_DIR}"
  ;;
esac

case "${CONDA_OPTION}" in
-c|--install_conda)
  INSTALL_CONDA_FLAG=$6
  echo "CONDA INSTALL OPTION WAS SET TO ${INSTALL_CONDA_FLAG}"
  ;;
esac



# Check if oppt is to be installed
echo ${INSTALL_OPPT}
if (${INSTALL_OPPT});
then
	echo "FLAG SET TO ${INSTALL_OPPT_FLAG}: Installing OPPT into ${OPPT_INSTALL_DIR}"
	cd ${SKD_ROOT_DIR}
	chmod +x oppt_install.sh && ./oppt_install.sh --install_dir ${OPPT_INSTALL_DIR}
fi


# # Export oppt libraries for installed oppt dir
# # Proceed with skd_oppt install
# set oppt_DIR for CMake to find oppt
export oppt_DIR=${OPPT_INSTALL_DIR}/lib/cmake/oppt
echo "INSTALLING SKD_OPPT PLUGINS AND ENVIRONMENT MODELS"
cd ${SKD_ROOT_DIR}
chmod +x skd_oppt_install.sh && ./skd_oppt_install.sh


# # # Install conda dependencies
echo "SETTING UP ANACONDA DEPENDENCIES"
cd ${SKD_ROOT_DIR}
chmod +x install_conda_env.sh && ./install_conda_env.sh -c ${INSTALL_CONDA_FLAG}

echo "DONE INSTALLING ALL MODULES. READY TO RUN ASSESSMENT FOR TESTING"


