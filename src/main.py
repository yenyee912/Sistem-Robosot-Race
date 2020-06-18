#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import actionlib
import tf
from math import pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

red_xyr = [-1,-1,-1]
blue_xyr = [-1,-1,-1]
yellow_xyr = [-1,-1,-1]
xyr = [red_xyr, blue_xyr, yellow_xyr]

# Max speed and PID constants
# angular rotation:
max_a = 0.5 # rad/s
kp_a = 1.2
max_l = 0.15 # m/s
kp_l = 0.6
lost_ball_cnt = 0
slow_approach_cnt = 0
offset_y_gnd =  -0.45 #0.12 # offset of y-axis 

# This is our list goal coordinates
waypoints = [  
    [(2.1,-0.6,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],     # red
    [(2.1, 0.0,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],     # blue
    [(2.1, 0.6,0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],     # yellow 
]

def goal_pose(pose):  # Convert from waypoints into goal positions
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

def red_xyr_callback(data):
    global red_xyr
    red_xyr_str = data.data.split(',')
    red_xyr[0] = float(red_xyr_str[0])
    red_xyr[1] = float(red_xyr_str[1])
    red_xyr[2] = float(red_xyr_str[2])
    
def blue_xyr_callback(data):
    global blue_xyr
    blue_xyr_str = data.data.split(',')
    blue_xyr[0] = float(blue_xyr_str[0])
    blue_xyr[1] = float(blue_xyr_str[1])
    blue_xyr[2] = float(blue_xyr_str[2])

def yellow_xyr_callback(data):
    global yellow_xyr
    yellow_xyr_str = data.data.split(',')
    yellow_xyr[0] = float(yellow_xyr_str[0])
    yellow_xyr[1] = float(yellow_xyr_str[1])
    yellow_xyr[2] = float(yellow_xyr_str[2])

def robot_commander():
    print("Starting Robot Commander")
    global kp_a, kp_l, max_a, max_l
    global red_xyr, blue_xyr, yellow_xyr
    global lost_ball_cnt, slow_approach_cnt
    # Setup our node
    rospy.init_node('commander', anonymous=True)
    # Publish to cmd_vel to move tb3
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    # Subscribe to all ball finders
    rospy.Subscriber("red/ball_xyr", String, red_xyr_callback)
    rospy.Subscriber("blue/ball_xyr", String, blue_xyr_callback)
    rospy.Subscriber("yellow/ball_xyr", String, yellow_xyr_callback)

    rospy.Subscriber ('/odom', Odometry, get_rotation)
    # Move base client for waypoint navigation
    print("Connecting to move_base server...Please ensure the map data have been input.")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    print("Program Activation DONE!")
    raw_input("Press ENTER to start the game...")
    rate = rospy.Rate(10) # 10hz
    command = Twist()
    # state 0 - turn around
    # state 1 - find nearest ball
    # state 2 - move to ball
    # state 3 - secure the ball
    # state 4 - travel to goal
    # state 5 - go inside goal 
    # state 6 - reverse from goal and turn
    STATE = 1
    # BALL_COLOUR
    # 0 - red 
    # 1 - black 
    # 2 - yellow 
    
    BALL_COLOUR = 0
    while not rospy.is_shutdown():
        if STATE == 0: # turn around
            command.angular.z = 0.2
            command.linear.x = 0.05
            STATE = 1

        elif STATE == 1: # get nearest ball
            BALL_COLOUR = 0
            rospy.sleep(0.5) # wait a bit

            min_y = 500
            min_y_i = -1
            for i, c_xyr in enumerate(xyr):
                if c_xyr[2] > 0 and c_xyr[1] < min_y: #if radius>0
                    min_y = c_xyr[1]
                    min_y_i = i 
                    
            if min_y_i > -1: 
                BALL_COLOUR = min_y_i 
                lost_ball_cnt = 0
                STATE = 2
            else:
                STATE = 0

        elif STATE == 2: # move towards ball
            max_a = 0.5  # max turning speed, rad/s
            kp_a = 1.2   # proportional gain for turning speed
            max_l = 0.15 # max forward speed, m/s
            kp_l = 0.6   # proportional gain for forward speed 
            offset_y_gnd =  -0.45 # determines how near to the ball will the robot stop

            command.angular.z = kp_a * xyr[BALL_COLOUR][0]
            command.linear.x = kp_l * (xyr[BALL_COLOUR][1] - offset_y_gnd)
            # limit the speeds
            if command.linear.x > max_l:
                command.linear.x = max_l
            if command.angular.z > max_a:
                command.angular.z = max_a
            if command.angular.z < -max_a:
                command.angular.z = -max_a
            
            # check if ball is near enough
            if xyr[BALL_COLOUR][1] < offset_y_gnd and -0.05<xyr[BALL_COLOUR][0]<0.05:
                STATE = 3

            # check if ball is lost
            # if yes, start looking again
            if xyr[BALL_COLOUR][2] == 0:
                lost_ball_cnt+=1
            if lost_ball_cnt > 20:
                STATE = 1

        elif STATE == 3: # Slow approach ball
            command.angular.z = 0.0
            command.linear.x = 0.08 # approach speed
            print("Moving to ball..." +str(BALL_COLOUR))
            slow_approach_cnt+=1
            STATE = 4
            
        elif STATE == 4: # go to goal
            goal = goal_pose(waypoints[BALL_COLOUR])                         
            client.send_goal(goal)
            print("Moving towards goal " + str(BALL_COLOUR))
            # wait for robot to move, but if more than 60 sec, send next goal/waypoint
            client.wait_for_result(rospy.Duration.from_sec(60.0))
            # Once reached goal, approach goal
            STATE = 5

        elif STATE == 5: # approach goal
            command.angular.z = 0.0
            command.linear.x = 0.08 # approach speed
            slow_approach_cnt+=1
            if slow_approach_cnt > 50: # how long to move like this
                slow_approach_cnt = 0
                STATE = 6

        elif STATE == 6: # reverse from goal and turn
            slow_approach_cnt += 1
            if slow_approach_cnt < 50:  # how long to reverse
                # reverse
                command.angular.z = 0.0
                command.linear.x = -0.08  # reverse speed
                print("Reversing...")

            elif slow_approach_cnt < 100:  # how long to turn away
                # turn
                command.angular.z = 0.4  # turning speed
                command.linear.x = 0.0
                print("Turning Back...")
            else:
                slow_approach_cnt = 0
                STATE = 1


        print("--------------------------")
        print("STATE: " + str(STATE))
        print("BALL_COLOUR: " + str(BALL_COLOUR))
        print("COMMAND: x:" + str(command.linear.x) + "   z: " + str(command.angular.z))
        pub.publish(command) 
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_commander()
    except rospy.ROSInterruptException:
        pass
