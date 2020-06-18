# Sistem-Robosot-Race
Simple ball detecting program to be used on ROS Turtlebot3 in Robosot Race FIRA. The robot can detect 3 types of coloured ball: red, blue, yellow. Robot will approach the balls and take them to corresponding goal. Turtlebot3 will do path planning itself so no manually control is needed.  

If you are a begineer, please follow the following steps to save your time.<br />
Preparation:<br />
1. Make sure that you have installed ROS Kinetic Kame before running this program on your Ubuntu 16.04 OS. Please at least go through some basic linux operation tutorial. It may be very helpful.
2. Choose your own mapping algorithm, in my case I am using gmapping.<br /> Tutorial as follow:
http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-nodes <br />
Save your generated map. '$HOME' path is most recomended.<br />
- To the coordinate of goal:<br />
  - In a new terminal, run:<br />
$ rostopic echo /clicked_point <br />
  - In RViz, click on 'Publish Point'(toolbar on top of the map), then click at a point approx. 20cm in front of the goal opening. You may see the x,y,z of the point you clicked. Record the x,y coordinates only.
  - Insert the recorded coordinates into main.py
3. Create a playing field in gazebo simulation/ in reality. You may use this:<br />
https://github.com/
4. Clone this repository to your catkin workspace (catkin_ws). Unzip and rename it as 'robosot_race'. You may also create your onw ROS package and move the content into it. Just dont forget to change the package name in commands late.
**Remember to 'catkin_make'.** <br />

To launch the Robosot Race Robotics System:<br />
1. Bring up <br />
(a) Launch the Gazebo test field: $ roslaunch robosot_race_gazebo robosot_gazebo.launch <br />
(b) If you are using on a real machine, follow the http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/ to bring up your robot. <br />

2. Activate TT3 navigation node: <br /> 
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml <br />
 - I have export my robot model (Burger) in the bashrc file.  https://emanual.robotis.com/docs/en/platform/turtlebot3/export_turtlebot3_model/
 - map_file:= < PATH to your map > <br />  

3. Launch ball finder: $ roslaunch robosot_race finder.launch <br />
This is a launch file which to run multiple 'detetcor.py' node at one time. Depends on how many types of color you have. You may adjust your own color ball parameter and save it as a .yaml file in folder 'parameter'. Remember to include your parameter in the **finder.launch** too. <br />

4. Launch the commander: $ rosrun robosot_race main.py <br />
This a executable which control the flow of robot. Kindly refer the documentation in this script. <br />

- 'robosot_race' is my ROS package name. You may change to yours.<br />
- If you have problem in dynamic configuration (e.g no module named xxx.cfg), you may comment out the code. Core functions still work fine.
- If you find out any script is not working, it can be the executable permission issue. Just go to the location of script and use command: $ chmod a+x <script_name>.
- Just email me if you experience any problem. It can be my bugs. You can reach out to me from the "package.xml" file.
