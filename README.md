# Final Assignment
This software architecture in ROS implements two nodes to control a robot in the given environment.  
The software needs the gmapping package (that implements the omonimus algorithm (?)) and the move_base package for localizing the robot and plan the motion.  
The user interface node will allow the user to chose either to let the robot drive itself to a specific given point or to directly drive it freely. Even more, the user has the possibility to be assisted by the software in the obstacle avoidance.  
 
## Pre-development phase
To fully exploit the move_base package it's useful to read its [documentation](https://wiki.ros.org/move_base). At the linked page, the Action Subscribed Topics are listed. Those  are almost essential during the development, even though the goals will be sent using a *SimpleActionClient*.  
 
The quickest and easiest way to manually control the robot is by using the teleop_twist_keyboard. This will publish on the `/cmd_vel` topic a speed chosen according to the command given by the user. In order to help the user drive the robot without crushing on a wall, it's necessary to check, every time a new velocity is published, if the given setting will endanger the robot. If so, the speed must be modified.  
To do so, in the launch file called Simulation.launch, the `/cmd_vel` is remapped just before the teleop_twist_keyboard is ran. Doing so, only the *middleman* node and the *move_base* server will publish on the `/cmd_vel` topic. teleop_twist_keyboard will publish on a topic which has as its only subscriber the *middleman* node itself. This will check if the given speed is enough safe to be published on the `/cmd_vel` topic.

## Running
The repository has a launch file that will, in order:  
- call the *simulation_gmapping* launch file;  
- call the *move_base* launch file;    
- run the *user_interface* node;  
- run the controller node (called *middleman*);  
- remap `/cmd_vel` to `/middleman/cmd_vel`;  
- run the *teleop_twist_keyboard node*.  

The ROS Master node will be automatically called when launching the file.  
To launch use `roslaunch final_assignment Simulation.launch`  

## Robot behaviour 
It has three different behaviors, depending on the user input:
- It can autonomously reach a point given by the user. The move_base action server will elaborate the best plan to reach it, based on the knowledge acquired thanks to the gmapping algorithm. Thanks to the action paradigm, the user has the possibility to cancel the action before it's complete.  
- It can be fully controlled by the user;  
- It can be partially controlled by the user. If the robot is dangerously close to a wall, its speed will be modified in order to avoid a collision.  

The user can change the robot behavior anytime, by giving another command or just by pressing *0*.






