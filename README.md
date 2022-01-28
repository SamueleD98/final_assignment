# Final Assignment
This software architecture in ROS implements two nodes to control a robot in the given environment.  
The software needs the gmapping package (that implements the omonimus algorithm) and the move_base package for localizing the robot and plan the motion.  
The user interface node will allow the user to choose either to let the robot drive itself to a specific given point or to directly drive it freely. Even more, the user has the possibility to be assisted by the software in the obstacle avoidance.  
 
## Pre-development phase
To fully exploit the move_base package it's useful to read its [documentation](https://wiki.ros.org/move_base). At the linked page, the Action Subscribed Topics are listed. Those  are almost essential during the development, even though the goals will be sent using a *SimpleActionClient*.  
 
The quickest and easiest way to manually control the robot is by using the teleop_twist_keyboard. This will publish on the `/cmd_vel` topic a speed chosen according to the command given by the user. In order to help the user drive the robot without crushing on a wall, it's necessary to check, every time a new velocity is published, if the given setting will endanger the robot. If so, the speed must be modified. To do so, in the launch file called Simulation.launch, the `/cmd_vel` is remapped just before the teleop_twist_keyboard is executed. Doing so, only the *middleman* node and the *move_base* server will publish on the `/cmd_vel` topic. teleop_twist_keyboard will publish on a topic which has as its only subscriber the *middleman* node itself. This will check if the given speed is safe to be published on the `/cmd_vel` topic.

## Architecture of the system

![graph](/images/rosgraph.png)

The enlightened nodes are the results of this work:
- the middleman node which receives velocity commands by the *teleop_twist_keyboard* node and publish the corrected ones on the `/cmd_vel` topic
- the user interface node that sends goals to the move_base node and commands to the middleman node.

## Running
The repository has a launch file that will, in order:  
- call the *simulation_gmapping* launch file;  
- call the *move_base* launch file;   
- define the *execution_time* argument and its default value 
- set the parameter called *countdown* as equal to *execution_time*  
- run the *user_interface* node;  
- run the *middleman* node;  
- remap `/cmd_vel` to `/middleman/cmd_vel`;  
- run the *teleop_twist_keyboard node*.  
 
To launch use `roslaunch final_assignment Simulation.launch`.  
It is possible to set the amount of time the program will wait the robot to reach a point as a parameter by setting the *execution_time* argument. The default value is 150 seconds.

## Robot behaviour 
It has three different behaviors, depending on the user input:
- It can autonomously reach a point given by the user. The move_base action server will elaborate the best plan to reach it, based on the knowledge acquired thanks to the gmapping algorithm. Once the goal is sent to the *move_base* server, the node will wait one among the following possible events:
  - The User choose to cancel the action;
  - The Action Server realizes the target is unreachable;
  - The given Execution Time expires;
  - The robot manage to reach the target.
- It can be fully controlled by the user;  
- It can be *partially* controlled by the user: if the robot is dangerously close to a wall, its speed will be modified from the original desired one, in order to avoid a collision.  

The user can change the robot behavior anytime by giving another command. Every time a new input is inserted, the program automatically terminate any previous operation and elaborate the new command.  
If the user wants so, the robot could simply stop by just pressing *0*. As code, it really doesn't do much because, as mentioned, canceling past operations is a default operation. In this case, though, no other commands are given to the robot that then won't move.

## User Interface node
The UI node takes care of the user inputs for the control of the robot. It offers 4 options, as shown in the image:  

![user_interface](/images/interface.png)

The code implements the following algorithm:  
<pre>
<b>while</b> the program is running
 send commands to cancel previous commands
 <b>switch</b> user choice
  <b>case</b> '1'
   input coordinates the robot should autonomously reach
   send the command to the Move_Base server
   wait the action to be completed or to be canceled
   <b>break</b>
  <b>case</b> '2'
   send to middleman a command to enable the user control
   <b>break</b>
  <b>case</b> '3'
   send to middleman a command to enable the user partial control
   <b>break</b>
  <b>case</b> '4'
   exit
   <b>break</b>
  <b>case</b> '0'
   <b>break</b>
  <b>default</b>
   print "Not a command, please type again."
</pre>  

The *wait()* function contains the code that allows the user to cancel a goal while receiving feedbacks from the Move_Base server. Its algorithm is the following:
<pre>
<b>while</b> the action is not accepted yet
 sleep for a second
<b>while</b> the action is active
 <b>if</b> the user press enter during a one second interval
  cancel the goal
 <b>else</b>
  decrease the countdown variable by one  
  <b>if</b> countdown is equal to zero
   cancel the goal 
</pre>  

The *countdown* variable corresponds to the number of seconds the program will wait the robot to reach the target before it will cancel the goal. The variable is given as a parameter in the previously mentioned launch file. Also, it has a default value of 150 seconds if the program can't manage to find the parameter.  

In order to interrupt an input operation after a given time, the [selectors module](https://docs.python.org/3/library/selectors.html) is used.  
This module provides access to the *select()* system call, which allows a program to monitor multiple file descriptors, waiting until one or more of the file descriptors become "ready" for some class of I/O operation. A file descriptor is considered ready if it is possible to perform the corresponding I/O operation without blocking.  
In this particular case, the desired operation is a *Read* operation on the *stdin*. The select call will try to read for one second before returning.  

Since the user's possible inputs are integers as commands and floats as coordinates, the program needs to elaborate them as so. To prevent errors due to inputs of different types, several try/except statements are implemented.

## Middleman node
The system gives the user the chance to directly control the robot thanks to the *teleop_twist_keyboard*. Since the keyboard should be enabled only when the user explicitly ask to control the robot, an intermediary must exist between the *teleop_twist_keyboard* and the robot itself (that's why the node is called *middleman*). This node will write the velocity commands given by the *teleop_twist_keyboard* on the `/cmd_vel` topic only when the user asks to (command number 2).  

Also, the user can take advantage of an assistant that, when driving with the keyboard, prevents the robot to collide with a wall (command number 3). In this case, the node will modify the velocity commands received by the keyboard before publishing them on the `/cmd_vel` topic.  

The node mainly works with callbacks to the topics it is subscribed to:
### `/scan`
If the assistant mentioned before is enabled (boolean helper_status), the node will:
1. update the distance of the closest obstacle in the left lateral sector [-90°,-18°], central sector [-18°,18°] and right lateral sector [18°,90°] every time a new laser scan is published;
2. update the velocity commands to avoid collision following this simple algorithm:  <pre>
    <b>if</b> very close obstacle in front
     set the linear velocity to zero
    <b>else</b>
     set the linear velocity to the desired one
    <b>if</b> very close obstacle in lateral region AND the angular velocity is dangerous
     set the angular velocity to zero
    <b>else</b>
     set the angular velocity to the desired one
   </pre>Please note, the desired velocity is a global variable set by the user when using the keyboard to control the robot. Also, the angular velocity is considered  dangerous if the robot will point to a close obstacle by keeping that velocity.
3. publish the new velocity   

### `/middleman/control`
Topic (with custom message called CommandMessage) where the User Interface node publish the commands that reflects the robot desired behavior. The callback function will set the *keyboard_status* and the *helper_status*  global boolean variables to the desired value after having canceled any previous command.  

The custom message CommandMessage is made up of two boolean: enable_userCtrl and enable_helper. Those set the the keyboard_status and the helper_status in middleman node.

### `/middleman/cmd_vel`
Topic where the *teleop_twist_keyboard* publish the velocity commands. The callback function behaves accordingly to the following algorithm: 
<pre>
<b>if</b> keyboard_status
 <b>if</b> helper_status
  set the desired velocity equal to the one published by the *teleop_twist_keyboard*
 <b>else</b>
  publish the velocity published by the *teleop_twist_keyboard*
</pre>
So, the function won't have any effect if the keyboard is not enabled. If it is, and so is the assistant, the velocity will be saved on the global variable desired_speed and eventually published by the `/scan` callback (if possible). If the assistant is disabled, then the velocity commands will be directly published on the `\cmd_vel` topic.

## Further improvements
To make the code more modular, the client of the Move_Base server should be implemented in the middleman node, leaving to the user interface node just the task of taking user's commands and sending them to the middleman node. Actually, the development of this change already began in the move_base-interface-on-middleman branch of this project.  

The Move_Base server takes a non-negligible amount of time to notify the successful reaching of a point. Here, for example, the robot was asked to reach the very point it was located.
![countdown](/images/countdown.png)
It took slightly more than 3 minutes to the Move_Base server to notify the reached point.
During this time the countdown could reach zero and set the automatic canceling of the goal, resulting in a non completed operation (that actually was completed). To avoid this situation, it is, of course, possible to increase the time given to the robot to reach the target. This will, though, make the countdown useless since the program will either abort the operation for an unreachable target or actually reach it before it could go off.  
What if the user wants to know if a point is actually reachable in a specific amount of time? The program should return the accomplished operation before the countdown expires and before the move_base server notify the "reached point" event. It is possible to do so by comparing the actual position of the robot to the desired one and if they are the same (maybe with a given tolerance) the goal is accomplished.










