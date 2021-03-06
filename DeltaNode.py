#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import cv2
import numpy as np

#topic for publishing
output_topic = 'Motor_Control'
# publishes centroid and verifies when to take a new picture
def DeltaRobot():
    rospy.init_node( 'DeltaRobot', anonymous = True )
    sub = rospy.Subscriber( output_topic, String, queue_size=10)
   

# get xy from centroids
def getControl():
    cen = centroid_from_Picture()
    return xy_from_centroid( cen )

# captures picture and processes centroids
def centroid_from_Picture():
    cap=cv2.VideoCapture(0)
    ret, frame = cap.read()
    img = cv2.resize(frame,(1000,1000))
    
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cap.release()
    shapes = getShapes( img,hsv )
    centroids = getCentroids( shapes, gray )
    return centroids

# should be tuples of (
# color (String),
# lower bound (array), upper bound (array) )
colors = []
# creates color segmentation of the workspace.
def getShapes( image, h ):
    masks = []
    for (c, l ,u) in colors:
        mask = cv2.inRange( h, np.array(l), np.array(u) )
        masks.append(mask)
        
    # gets first shape from image    
    shapes = cv2.bitwise_and( image, image, masks[0] )
    # gets resulting shapes and or with current shape.
    for i in range(1 , len(masks) ):
        sh = cv2.bitwise_and( image, image, masks[i] )
        shapes = cv2.bitwise_or( shapes, shapes, sh )
    return shapes


lower_thresh = 100
# gets the centroid from segmentation
def getCentroids( shapes, g ):
    ret, thresh = cv2.threshold( g, lower_thresh, 220, 0)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)

    M = [cv2.moments(contours[i]) for i in range(0, len(contours))]
    cx = [ ( int( m['m10']/m['m00'] ) ) for m in M]
    cy = [ ( int( m['m01']/m['m00'] ) ) for m in M]
    cen = list( zip(cx,cy) )
    return cen

# processes all centroids for publishing
def xy_from_centroid( centroid_points ):
    result = list( map( camera_transfer, centroid_points ) )
    return result

# converts a centroid point to a camera function
def camera_transfer( centroid_point ):
    pass

if __name__ == '__main__':
    try:
        Centroid()
    except rospy.ROSInterruptException:
        pass