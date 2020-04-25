# Sistem-Robosot-Race
Simple ball detecting program to be used on ROS Turtlebot3 for Robosot Race FIRA

IMPORTANT!!! If you are a begineer, please follow the following steps to save your debugging time.
Preparation:

1. Make sure that you have installed ROS Kinetic Kame before running this program on your Ubuntu 16.04 OS.

2. Kindly do your own mapping. In my case I am using gmapping.
http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-nodes
Save your generated map in somewhere. 'Home' path is recomended.
**get the coordinate of goal:
In a new terminal, run:
$ rostopic echo /clicked_point
In RViz, click on 'Publish Point'(toolbar on top of the map), then click at a point approx. 20cm in front of the goal opening.


3. Create a playing field in gazebo simulation/ in reality. You may use this:
https://github.com/arixrobotics/robosot_race_gazebo
- if you use this, noticed that the package name is: robosot_race_gazebo

4. Clone this repository to your catkin workspace (catkin_ws). Unzip and rename it as 'mmdrsot'. You may also create your onw ROS package and move the content into it. Just dont forget to change the package name in commands late. 
**Remember to 'catkin_make'.

To use:
1. Bring up
(a) Launch the Gazebo test field: $ roslaunch robosot_race_gazebo robosot_gazebo.launch 
(b) If you are using on robot, follow the http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/ to bring up your robot

2. Activate TT3 navigation node:
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
* I have set my robot model to burger. You may refer http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes for your own model.
* map_file <PATH of your map>
  
3. Launch ball finder: $ roslaunch mmdrsot finder.launch
This is a launch file which to run multiple 'detetcor.py' node at one time. Depends on how many types of color you have. You may adjust your own color ball parameter and save it as a .yaml file in folder 'parameter'. Remember to include your parameter in the **finder.launch** too.

4. Launch the commander: $ rosrun mmdrsot main.py
This a executable which control the flow of robot. Kindly refer the documentation in this script. 

**'mmdrsot' is my ROS package name. You may change to yours!!
** If you have problem in dynamic configuration (e.g no module named xxx.cfg), you may comment out the code. Core functions still work fine.
** just contact me if you still struggled with any problem. It can be my bugs. You can reach out to be from the package.xml
