# Research track 2: First assignment (about the RT1 Final Assignment)
This software architecture in ROS implements a node to control a robot via a jupyter notebook.  
The software needs the gmapping package (that implements the omonimus algorithm) and the move_base package for localizing the robot and plan the motion.  
The user interface (as jupyter notebook) will allow the user to choose either to let the robot drive itself to a specific given point or to directly drive it freely. Even more, the user has the possibility to be assisted by the software in the obstacle avoidance.  
The interface has multiple plots to better understand the environment and the robot status.

The development phase, the architecture of the system and the algorithms are fully explained in the ReadMe of the main branch. 
In order to run the script, please refer to the instructions inside the notebook `jupyter_notebook_user_interface.ipynb`.
