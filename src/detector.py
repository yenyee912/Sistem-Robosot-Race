#!/usr/bin/env python
import rospy
import cv2
import math
import numpy

#import the config file and dynamic_reconfigure
from dynamic_reconfigure.server import Server
from mmdrsot.cfg import Color2Config
#from robosot_race.cfg import ColourRangeConfig
#package_name.folder_name

#important dependencies for using OPen CV with ROS
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #link ros and cv image
from std_msgs.msg import String

#---Initialization----
H_min, S_min, V_min=0,0,0
H_max, S_max, V_max=180, 255, 255 #Hue: 0-180
hough_res=1.2 #hough accumulation resolution
min_distance=5 #minimum distance between every circle
#too small-->fake circle; too large-->missed target
radius_min=0
radius_max=100
canny_th=100
hough_th=28

#this callback is used for dynamic reconfiguration
#enable user to tune parameter via rqt dashboard
def color_callback(cfg, level): #-->level=bitmask
    #--Declaration--
    global H_min, S_min, V_min #=0 -->reset for every color ball
    global H_max, S_max, V_max

    # hough transform parameters
    global hough_res, min_distance, radius_min, radius_max
    global canny_th, hough_th  # threshold

    #refer to ColourRangeConfig.cfg
    H_min= cfg.H_min
    H_max= cfg.H_max
    S_min= cfg.S_min
    S_max= cfg.S_max
    V_min= cfg.V_min
    V_max= cfg.V_max
    hough_res= cfg.hough_accum_resolution
    min_distance= cfg.min_circle_dist
    radius_max= cfg.max_radius
    radius_min= cfg.min_radius
    canny_th= cfg.canny_edge_th
    hough_th= cfg.hough_accum_th

    return cfg #used for dynamic reconfigure

#this callback is to (a) manage the incoming ros image stream(video)
# -->do Open Cv Image Processing
#--> generate output to be publish in listener()--> main function
def callback(ros_image):
    global ball_xyr, output_circle, output_color #main output
    global H_min, S_min, V_min
    global H_max, S_max, V_max

    #hough transform parameters
    global hough_res, min_distance, radius_min, radius_max
    global canny_th, hough_th #threshold

    #create CvBridge() object--> communication
    bridge=CvBridge()

    #capture one frame--> convert ros image to cv image
    #img=cv2.imread(ros_image) //not using this since the dt type is ros image
    img= bridge.imgmsg_to_cv2(ros_image,"bgr8") #encoding
    hsv= cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #bgr-->hsv

    # for color filtering(manipulate hue only)
    #----Color identification----filter HSV image to get only wanted colors
    #define the hsv range for every color-->data is from yaml file (params)
    lower_hsv= numpy.array([H_min, S_min, V_min])
    upper_hsv= numpy.array([H_max, S_max, V_max])
    # each image  is represented as a NumPy array with shape (h, w, 3),
    # where the 3 is the number of channels in bgr image

    # create mask for the EVERY color
    mask=cv2.inRange(hsv,lower_hsv,upper_hsv)
    #cv2.imshow("MASK",mask)

    #merge mask with original cv2 image
    detected_color=cv2.bitwise_and(img,img,mask=mask) #-->bitwise is a operation to merge 2 img
    # parameters: src,output,mask=
    detected_circle=detected_color.copy() #clone the color display

    #---Cricle deection---hough transfomr
    gray=  cv2.cvtColor(detected_color,cv2.COLOR_BGR2GRAY) #grayscale image
    edges= cv2.Canny(gray, 50,240) #double threshold--> will help you find edges
    #edges is a variable which store EDGES detected

    #dilations-> increase detected region
    kernel= numpy.ones((3,3),numpy.uint8) #3x3 kernel with value unsingned integer:0-255(grayscale)
    dilation= cv2.dilate(edges,kernel=kernel,iterations=1)
    #erosion=cv2.erode(dilation,kernel=kernel,iterations=1)

    #https://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    circles=cv2.HoughCircles(dilation,cv2.HOUGH_GRADIENT, hough_res,min_distance,param1=canny_th,
                             param2=hough_th,minRadius=radius_min,maxRadius=radius_max)

    #parameters: src,method,dp,min distance between circle, param1=Gradient value used to handle edge detection,
    #           Param2=Accumulator threshold value for the cv2.HOUGH_GRADIENT method.
    #           smaller threshold-->including false circles).
    #           larger threshold--> missed circle
    #           min_r, max_r
    #type(circles) #-->check dt type:

    #check whether is any circle detected:
    if circles is not None: #at least one ball found
        rospy.loginfo(rospy.get_caller_id() + " Ball(s) found.")
        #convert the (x, y) coordinates and radius of the circles to integers
        circles= numpy.round(circles[0, :]).astype("int")
        circles_sorted= sorted(circles, key=lambda x: x[2])
        #search for the largest radius in circles[]

        #---for display used only---
        for (x,y,r) in circles:  #loop over the (x, y) coordinates and radius of the circles
            cv2.circle(detected_circle,(x,y),r,(0, 255, 0),4) #green color with 4px(thickness)
            cv2.rectangle(detected_circle,(x-2,y-2),(x+2,y+2),(0, 128, 255),-1)
            # draw the actual detected circle on Line 118 using the cv2.circle function,
            # followed by drawing a rectangle at the center of the circle on Line 29.

    else:
         rospy.loginfo(rospy.get_caller_id()+ "Sorry!No ball found.")
         circles_sorted=[[0,0,0]]

    #convert processed cv image--> ros image    
    color_conversion=bridge.cv2_to_imgmsg(detected_color,"bgr8")
    shape_conversion=bridge.cv2_to_imgmsg(detected_circle,"bgr8")

    output_circle.publish(shape_conversion)
    output_color.publish(color_conversion)

    circle_x=1.0*circles_sorted[-1][0]/img.shape[1] #x=row
    circle_y=1.0*circles_sorted[-1][1]/img.shape[0] #y=column
    # each image is represented as a NumPy array with shape (h, w, 3),
    # where h is col, w is row, "3" is the number of channels

    CX= 0.5-circle_x
    CY= 0.5-circle_y
    CR= 1.0*circles_sorted[-1][2]/ img.shape[1]
    ball_xyr.publish(str(CX)+","+str(CY)+","+str(CR))
    #remember to insert comma---> used for data separation in callback function

    cv2.waitKey(1)
    #INCLUDE THIS OR WILL NOT GETTING ANY OUTPUT!!!!!!!


#--Listener--
def listener():

    #--Declaration--
    global ball_xyr, output_circle, output_color

    rospy.init_node('ball_detector', anonymous=True)
    rospy.Subscriber("/camera/image", Image, callback)
    ball_xyr=rospy.Publisher('ball_xyr', String, queue_size=1) #xyr
    output_color= rospy.Publisher('output_color',Image, queue_size=1)
    output_circle= rospy.Publisher('output_circle', Image, queue_size=1)

    # Dynamic reconfigure server
    #srv = Server(ColourRangeConfig,color_callback)
    srv = Server(Color2Config,color_callback)
    # Just spin and wait
    rospy.spin()

#--ROS main method---
if __name__ == '__main__': #main(
#check if the python script is to executed directly or to be import
    # try:
    #     listener()
    # except rospy.ROSInterruptException:
    #     pass

    listener() #---> needed to be execute forever until end program!!!!
