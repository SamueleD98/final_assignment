# Final Assignment
This software architecture in ROS implements two nodes to control a robot in the given environment.  
The software needs the gmapping package (that implements the omonimus algorithm (?)) and the move_base package for localizing the robot and plan the motion.  
The user interface node will allow the user to chose either to let the robot drive itself to a specific given point or to directly drive it freely. Even more, the user has the possibility to be assisted by the software in the obstacle avoidance.  
 
## Pre-development phase
To fully exploit the move_base package it's useful to read its [documentation](https://wiki.ros.org/move_base). At the linked page, the Action Subscribed Topics are listed. Those  are almost essential during the development, even though the goals will be sent using a *SimpleActionClient*.  
 
The quickest and easiest way to manually control the robot is by using the teleop_twist_keyboard. This will publish on the `/cmd_vel` topic a speed chosen according to the command given by the user. In order to help the user drive the robot without crushing on a wall, it's necessary to check, every time a new velocity is published, if the given setting will endanger the robot. If so, the speed must be modified.  
To do so, in the launch file called Simulation.launch, the `/cmd_vel` is remapped just before the teleop_twist_keyboard is executed. Doing so, only the *middleman* node and the *move_base* server will publish on the `/cmd_vel` topic. teleop_twist_keyboard will publish on a topic which has as its only subscriber the *middleman* node itself. This will check if the given speed is safe to be published on the `/cmd_vel` topic.

## Running
The repository has a launch file that will, in order:  
- call the *simulation_gmapping* launch file;  
- call the *move_base* launch file;    
- run the *user_interface* node;  
- run the *middleman* node;  
- remap `/cmd_vel` to `/middleman/cmd_vel`;  
- run the *teleop_twist_keyboard node*.  

The ROS Master node will be automatically called when launching the file.  
To launch use `roslaunch final_assignment Simulation.launch`  

## Robot behaviour 
It has three different behaviors, depending on the user input:
- It can autonomously reach a point given by the user. The move_base action server will elaborate the best plan to reach it, based on the knowledge acquired thanks to the gmapping algorithm. Once the goal is sent to the *move_base* server, the node will wait one among the following possible events:
  - The User chose to cancel the action;
  - The Action Server realizes the target is unreachable;
  - The given Execution Time expires;
  - The robot manage to reach the target.
- It can be fully controlled by the user;  
- It can be *partially* controlled by the user: if the robot is dangerously close to a wall, its speed will be modified from the original desired one, in order to avoid a collision.  

The user can change the robot behavior anytime by giving another command. Everytime a new input is inserted, the program automatically terminate any previous operation and elaborate the new command.  
If the user wants so, the robot could simply stop by just pressing *0*. As code it really doesn't do much because, as mentioned, canceling past operations is a default operation. In this case, though, no other commands are given to the robot that then won't move.

## User Interface node
The UI node takes care of the user inputs for the control of the robot. It offers 4 options, as shown in the image !!!!!!!!!!!!!!!!!!!!!!!!!!:  

![user_interface](/images/roslaunch.png)

The code implements the following algorithm:  
<pre>
<b>while</b> the program is running
 input user choice
 send commands to cancel previous commands !!!!!!!!!!!
 <b>switch</b> user choice
  <b>case</b> '1'
   input coordinates the robot should autonomously reach
   send the command to the Move_Base server
   wait the action to be completed or to be canceled
   <b>break</b>
  <b>case</b> '2'
   send to middlman a command to enable the user control
   <b>break</b>
  <b>case</b> '3'
   send to middlman a command to partially enable the user control
   <b>break</b>
  <b>case</b> '4'
   exit
   <b>break</b>
  <b>case</b> '0'
   <b>break</b>
  <b>default</b>
   print "Wrong character, please type again."
</pre>  

The *wait()* function contains the code that allow the user to cancel a goal while receiving feedbacks from the Move_Base server. Its algortihm is the following:
<pre>
<b>while</b> the action is not accepted yet
 sleep for a second
<b>while</b> the action is active
 <b>if</b> the user press enter during a one second interval
  cancel the goal
 <b>else</b>
  decrease the countdown variable by one second
  <b>if</b> countdown is equal to zero
   cancel the goal 
</pre>  

The *countdown* variable correponds to the number of seconds the program will wait the robot to reach the target before it will cancel the goal. The variable is given as parameter in the previously mentioned launch file. Also, it has a default value of 150 seconds if the program can't manage to find the parameter.  

In order to interrupt an input operation after a given time, the [selectors module](https://docs.python.org/3/library/selectors.html) is used. This module provides access to the select() system call, which allow a program to monitor multiple file descriptors, waiting until one or more of the file descriptors become "ready" for some class of I/O operation. A file descriptor is considered ready if it is possible to perform the corresponding I/O operation without blocking. In this particular case the desired operation is a *READ* operation on the *stdin*. The select call will try to read for one second before returning.  

Since the user's possible inputs are integers as commands and floats as coordinates, the program needs to elaborate them as so. To prevent errors due to inputs with different types, some try/except statements are implemented.

## Middleman node
The system gives the user the chance to directly control the robot thanks to the *teleop_twist_keyboard*. Since the keyboard should be enabled only when the user explicity ask to control the robot, an intermediary must exist betwen the *teleop_twist_keyboard* and the robot itself (that's why the node is called *midlleman*). This node will write the velocity commands given by the *teleop_twist_keyboard* on the `/cmd_vel` topic only when the user asks to (command number 2).  

Also, the user can take advantage of an assistant that, when driving with the keyboard, prevents the robot to collide with a wall (command number 3). In this case the node will modify the velocity commands received by the keyboard before publishing them on the `/cmd_vel` topic.  

The node mainly works with callbacks to the topics it is subscribed to:
- `/scan` topic: if the assistant mentioned before is enabled (boolean helper_status), the node will:
  1. update the distance of the closest obstacle in the left lateral sector [-90°,-18°], central sector [-18°,18°] and right lateral sector [18°,90°], everytime a new laser scan is published;
  2. update the velocity commands to avoid collision following this simple algorithm:
   <pre>
   <b>if</b> very close obstacle in front
    set the linear velocity to zero
   <b>else</b>
    set the linear velocity to the desired one *
   <b>if</b> very close obstacle in lateral region AND the angular velocity is dangerous **
    set the angular velocity to zero
   <b>else</b>
    set the angular velocity to the desired one
   </pre>  
  3. publish the new velocity   
- `/middleman/control` topic: custom topic where the User Interface node publish the commands that reflects the robot desired behavior. The callback function will set the *keyboard_status* and the *helper_status*  global boolean variables to the desired value after having canceled any previous command.
- `/middleman/cmd_vel` topic: custom topic where the *teleop_twist_keyboard* publish the velocity commands. The callback function behaves accordingly to the following algorithm:
  <b>if</b> keyboard_status
   <b>if</b> helper_status
    set the desired velocity equal to the one published by the *teleop_twist_keyboard*
   <b>else</b>
    publish the velocity published by the *teleop_twist_keyboard*
  </pre>  
  So, the function won't have any effect if the keyboard is not enabled. If it is, and so is the assistant, the velocity will be saved on the global variable desired_speed and eventually published by the `/scan` callback (if possible). If the assistant is disabled, then the velocity commands will be directly published on the `\cmd_vel` topic.

\* the desired velocity is a global variable set by the user when using the keyboard to control the robot  

\** the angular velocity is considered dangerous if the robot will point to a close obstacle by keeping that velocity.

