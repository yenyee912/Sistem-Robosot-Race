# Sistem-Robosot-Race
Simple ball detecting program to be used on ROS Turtlebot3 in Robosot Race FIRA. The robot can detect 3 types of coloured ball: red, blue, yellow. Robot will approach the balls and take them to corresponding goal. Turtlebot3 will do path planning itself so no manually control is needed.  
Example of program demostration: https://youtu.be/q1ffZCIumT0

If you are a begineer, please kindly refer the following steps to save your time.<br />
Preparation:<br />
1. Make sure that you have installed ROS Kinetic Kame before running this program on your Ubuntu 16.04 OS. Please at least go through some basic linux operation tutorial. It may be very helpful.
2. Choose your own mapping algorithm, in my case I am using gmapping. <br /> Tutorial as follow:
http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-nodes <br />
Save your generated map. '$HOME' path is most recomended. I have prepare a ready map (map.pgm/.yaml) that you may give it a try. Just copy the map files and save them to $HOME.<br />
- To the coordinate of goal:<br />
  - In a new terminal, run:<br />
$ rostopic echo /clicked_point <br />
  - In RViz, click on 'Publish Point'(toolbar on top of the map), then click at a point approx. 20cm in front of the goal opening. You may see the x,y,z of the point you clicked. Record the x,y coordinates only.
  - **Insert the recorded coordinates into src/main.py (line 30).**
3. Create a playing field in gazebo simulation/ in reality. You may use this:<br />
https://github.com/yenyee912/Robosot-Race-Gazebo4. 
Clone and unzip this repository to your catkin workspace (catkin_ws). You may choose to create your onw ROS package and copy the contents into it your new made file. Just dont forget to change the package name in commands later.
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
This is a launch file which to run multiple 'src/detetcor.py' node at one time. Depends on how many types of color you have. You may adjust your own color ball parameter and save it as a .yaml file in folder 'parameter'. Remember to include your parameter in the **finder.launch** too. <br />

4. Launch the commander: $ rosrun robosot_race main.py <br />
This a executable which control the flow of robot.

- 'robosot_race' is my ROS package name. You may change to yours.<br />
- If you find out any script is not working, it can be the executable permission issue. Just go to the location of script and use command: $ chmod a+x <script_name>.
- Just email me if you experienced any problem. You can reach out to me from the "package.xml" file.
- May customize your desired color ball, change the parameter file in "parameter" folder. The value of parameters can be found by dynamic reconfigure. http://wiki.ros.org/dynamic_reconfigure <br />
- The color ball parameters found can be save as a .yaml file. Remember to include your parameter in the **finder.launch** too. <br />
