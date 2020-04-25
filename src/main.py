#!/usr/bin/env python
import rospy #import rospy if you are writing a ROS Node.
import actionlib
import tf #coordinate transformation
import time
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
#messages used to communicate with the move_base node
from std_msgs.msg import String #reuse the std_msgs/String msg type (a simple string container) for publishing.
from geometry_msgs.msg import Twist
from math import pi

# ---Global Initialization---

# array to store [x,y,r] information of ball
red_xyr=[-1,-1,-1]
blue_xyr=[-1,-1,-1]
yellow_xyr=[-1,-1,-1]

# list of balls--> to store more than one ball xyr (if detected)
xyr=[red_xyr, blue_xyr, yellow_xyr]

# PID constant(speed)
kp_angular=1.2 #proportional gain
kp_linear=0.6
max_angular=0.5
max_linear=0.15
offset_y_gnd=-0.45 #determine how near robot will stop
slow_approach_count=0
lost_ball_count=0
reverse_count=0

# list of goals--> x,y coordinates generated from topic 'publish_point'
waypoints=[ #red, blue, yellow goals
    [(2.1,-0.6,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],
    [(2.1,0.0,0.0), tf.transformations.quaternion_from_euler(0, 0, 0 * pi / 180)],
    [(2.1,0.3,0.0), tf.transformations.quaternion_from_euler(0, 0, 0 * pi / 180)],
    # tf.transformations.quaternion_from_euler()--> transform x,y,z to roll,pitch,yaw
]

# ---Function---
# callback function to receive data/msg from ball detector
def red_callback(data):
    global red_xyr
    # ball_xyr.publish(str(cx)+","+str(cy)+","+str(cr))
    red_xyr_str= data.data.split(',')
    # print (red_xyr[0], red_xyr[1], red_xyr[2])
    #convert string to float
    red_xyr[0]=float(red_xyr_str[0]) # x-coordinate
    red_xyr[1]=float(red_xyr_str[1]) # y-coordinate
    red_xyr[2]=float(red_xyr_str[2]) # radius
    # print (red_xyr[0], red_xyr[1], red_xyr[2])
    # print (red_xyr_str[0], red_xyr_str[1], red_xyr_str[2])

def blue_callback(data):
    global blue_xyr
    blue_xyr_str= data.data.split(',')
    blue_xyr[0]=float(blue_xyr_str[0])
    blue_xyr[1]=float(blue_xyr_str[1])
    blue_xyr[2]=float(blue_xyr_str[2])

def yellow_callback(data):
    global yellow_xyr
    yellow_xyr_str= data.data.split(',')
    #convert string to float
    yellow_xyr[0]=float(yellow_xyr_str[0])
    yellow_xyr[1]=float(yellow_xyr_str[1])
    yellow_xyr[2]=float(yellow_xyr_str[2])

def goalpose_conversion(pose): #use when robot need to move to goal
    # (pose) is the waypoint[] item input when needed--> [(x,y,z),(roll, yaw, pitch)]
    gp=MoveBaseGoal() #important!!must include this line
    # -->create a goal to send to move_base node
    #MoveBaseGoal() is msg type of move_base_msgs(***IMPORT!!!)

    gp.target_pose.header.frame_id= '/map' #in map coordinate FRAME....can be base_link sommetimes
    #position:x,y,z
    gp.target_pose.pose.position.x=pose[0][0]
    gp.target_pose.pose.position.y=pose[0][1]
    gp.target_pose.pose.position.y=pose[0][1]
    #quaternion/orientation:x,y,z,w
    gp.target_pose.pose.orientation.x=pose[1][0]
    gp.target_pose.pose.orientation.y=pose[1][1]
    gp.target_pose.pose.orientation.z=pose[1][2]
    gp.target_pose.pose.orientation.w=pose[1][3]

    return gp
    #so "goal" object contained coordinate(converted)that can be send to move_base
    #-->state 4

# ---Commander Function---
def commander():
    print("Dear user: WELCOME to Robosot Race Commander Program.")
    print("--Please make sure Gazebo World is loaded OR TT3 is activated.--")
    # ---Declaration---
    global kp_angular, kp_linear, max_angular, max_linear, offset_y_gnd
    global red_xyr, yellow_xyr, blue_xyr
    global slow_approach_count #how long the state berterusan
    global lost_ball_count
    global reverse_count
    #setup ROS node
    rospy.init_node('commander', anonymous=True)
    # anonymous = True-->ensures that your node has a unique name by adding random numbers to the end of NAME.
    # -->so that multiple listeners can run simultaneously.
    # Unique names are more important for nodes like drivers, where it is an error if more than one is running.
    # If two nodes with the same name are detected on a ROS graph, the older node is shutdown.

    #Publisher
    pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # set up a  publish command velocity to move TT3.
    # -->declare this node(commander) publishing to the 'cmd_vel' topic using the msg type Twist.
    # Twist--> class geometry.msg.Twist.
    # The queue_size ARGUMENT limits the amount of queued messages if any subscriber is not receiving them fast enough.

    #Subscriber --> all ball finders
    # use "topic name"!!!
    # This declares that your node subscribes to the "red/ball_xyr" topic which is of type std_msgs.msgs.String.
    # When new messages are received, 'red_callback' callback is invoked with the message as the first argument.
    rospy.Subscriber("red/ball_xyr", String, red_callback) #no need to create sub obj--> not using it
    rospy.Subscriber("blue/ball_xyr", String, blue_callback)
    rospy.Subscriber("yellow/ball_xyr", String, yellow_callback)
    #rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
 
    print("Connecting to move_base server...")
    print("Please make sure the navigation node is active or the map.yaml file is loaded.")
    #create action client--> send action command to action server (using MoveBaseAction)
    client=actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    print("Lets go! Robosot Race Commander Program is READY!")
    print("-------------------------------")
    raw_input("PRESS ENTER TO START THE GAME.")
    #state=0 #turn around-->scan diff. region
    state=1 #choose the nearest ball (within all balls detected)
    ball_color=-1
    #!!!!! 0=red, 1=blue, 2=yellow--> STRICTLY FOLLOW THE ORDER IN xyr[]!!!!!
    # or the waypoints of goal will be wrong!
    rate=rospy.Rate(10) #communicate at 10Hz
    #create geometry_msgs object, 'vel'--> send cmd_vel(Twist msg type) to TT3
    vel=Twist()

    while not rospy.is_shutdown():
        #state transition--> 0,1,2,3,4,5,6,-1
        if state==0: #turn around
            vel.angular.z=0.2
            vel.linear.x=0.15
            #The Twist has units of m/s for the linear terms, as well as radian/s for the angular terms.
            state=1 #check once per iteration--> if ball not found will back to STATE 0 again

        elif state==1: #decided which is the nearest ball
            rospy.sleep(0.5) #wait for a while
            BALL_COLOR=0
            min_y=100 #impossible coordinate
            min_y_i=-1 #NOT ball code

            for i, current_xyr in enumerate(xyr): #loop over the list of detected/not balls
                # radius>0--> detected;
                # y< min_y--> find out smallest y in the xyr[] list
                # 1. detetmine the nearest detected ball(a)--> get color code(b)
                if current_xyr[2]>0 and current_xyr[1]<min_y:
                    min_y=current_xyr[1]
                    min_y_i=i #(b)
                    #rospy.loginfo(min_y_i," min: ",min_y)

            #--this condition is for state transition--
            if min_y_i==0 or min_y_i==1 or min_y_i==2:
                ball_color=min_y_i
                state=2
            else:
                state=0

        elif state==2: #move to the ball
            kp_angular = 1.2
            kp_linear = 0.6
            max_angular = 0.5
            max_linear = 0.15
            offset_y_gnd = -0.45

            #proportional gain for rotation speed
            vel.angular.z= kp_angular * xyr[ball_color][0]

            #move forward speed
            offset=xyr[ball_color][1]-offset_y_gnd #minimize the offset
            vel.linear.x= kp_linear * offset

            #not to exceed the max speed
            if vel.linear.x> max_linear:
                vel.linear.x= max_linear #publish the maximum linear speed
            if vel.angular.z> max_angular:
                vel.angular.z= max_angular
            if vel.angular.z < -max_angular:
                vel.angular.z= -max_angular

            #check if the "y" is near enough for state 3
            if xyr[ball_color][1]<offset_y_gnd and -0.05<xyr[ball_color][0]<0.05:
                state=3

            if xyr[ball_color][2]==0: #no radius
                lost_ball_count+=1
            if lost_ball_count>10:
                state=1 #assume the lost ball is around-->find the nearset ball

        elif state==3: #approach ball with low speed
            vel.angular.z=0 #no rotation
            vel.linear.x=0.08
            slow_approach_count+=1
            if slow_approach_count>30:
                slow_approach_count=0 #reset the count--> for next ball
                state=4

        elif state==4: #decided which goal
            goal= goalpose_conversion(waypoints[ball_color])

            # Sends the goal to the action server
            client.send_goal(goal)
            print("Moving towards goal:"+str(ball_color))

            # Waits for the server to finish performing the action.
            wait=rospy.Duration.from_sec(60)
            client.wait_for_result(wait)#-->new goal command/msg will send to robot after 60s
            state=5

        elif state==5:
            # this state is to determine (a) speed and (b) time of approaching
            vel.angular.z=0 #(a)same as state 3
            vel.linear.x=0.08

            slow_approach_count+=1
            if slow_approach_count>45: #(b)
                #slow_approach_count=0
                state=6 #stop when count>45

        elif state==6: #reverse from the goal
            reverse_count+=1
            if reverse_count<45: #reverse
                vel.linear.x=-0.08
                vel.angular.z=0

            elif reverse_count<95: #turn away
                vel.linear.x=0
                vel.angular.z=0.4 #turn

            else:
                reverse_count=0

        print("--------------------")
        color=""
        if ball_color==0:
            color="RED(0)"
        elif ball_color==1:
            color="BLUE(1)"
        if ball_color==2:
            color="YELLOW(2)"

        print("STATE: " + str(state))
        print("Current Target: "+color+" Ball")
        print("COMMAND: x:" + str(vel.linear.x) + "   z: " + str(vel.angular.z))

        #publish cmd_vel--> after go through condition
        pub.publish(vel)
        rate.sleep()

# ---ROS Main Method---
if __name__== '__main__':
    try:
        commander()
    except rospy.ROSInterruptException: #ctrl+z
        #The reason this exception is raised is so that you don't
        # accidentally continue executing code after the sleep().
        pass
