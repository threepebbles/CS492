# Setting up this repository 
This repository includes a mid-level controller, complex_action_client, for (mobile) manipulators such as UR5. 

## Install
Install dependencies:
~~~~bash
sudo apt-get install ros-melodic-trac-ik ros-melodic-trac-ik-python -y 
~~~~

This repo. also requires KDL libraries and its wrapper:
~~~~bash
git clone https://github.com/rirolab/riro-kdl.git    
~~~~


## RestrictionsConditions
* This arm client assumes spherical wrist joints.
* This arm client has been tested with 6 DoF arms.
