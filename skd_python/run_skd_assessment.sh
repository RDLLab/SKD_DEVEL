#!/bin/bash

# Get the name of the directory where the repository is located
mkdir experiments

# Run the SKD Assessment 
python3 SKDAggregator.py -o ${PWD}/experiments -cfg config/config.yaml -oppt ~/SKD_DEVEL/opptv5/bin/abt 

