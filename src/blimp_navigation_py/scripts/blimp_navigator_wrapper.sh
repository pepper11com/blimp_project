#!/bin/bash
# Wrapper script to run blimp_navigator with yamspy environment

# Activate the yamspy virtual environment
source /home/blimp2/yamspy_env/bin/activate

# Run the actual Python script with all arguments passed through
exec python3 /home/blimp2/blimp_ws/install/blimp_navigation_py/lib/python3.12/site-packages/blimp_navigation_py/blimp_navigator.py "$@"