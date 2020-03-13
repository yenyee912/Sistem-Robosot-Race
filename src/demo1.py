import numpy as np
import cv2

hough_res = 1.2  # hough accumulation resolution
min_distance = 5  # minimum distance between every circle
# too small-->fake circle; too large-->missed target
radius_min = 0
radius_max = 100
canny_th = 100
hough_th = 28

img= cv2.imread("ball.png")
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
canny= cv2.Canny(gray,50,240)

kernel=np.ones([3,3], np.unit8) #3x3 kernel with u
cv2.imshow("",canny)
cv2.waitKey(0)





