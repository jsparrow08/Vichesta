#!/usr/bin/env python

import rospy
import cv2
import sys
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import tf
import numpy as np

class door_detection:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub1 = message_filters.Subscriber('/camera/depth/image_rect_raw',Image)
        self.sub2 = message_filters.Subscriber('/odom',Odometry)
        self.sub3 = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.ts = message_filters.TimeSynchronizer([self.sub1,self.sub2,self.sub3],10)
        self.ts.registerCallback(self.callback)
        self.listener = tf.TransformListener()
    def callback(self,depth_data,odom_data,img_data):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_data,"passthrough")
            color_img = self.bridge.imgmsg_to_cv2(img_data,"bgr8")
            #color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print(e)
        point = odom_data.pose.pose.position
        x = point.x
        y = point.y
        orient = odom_data.pose.pose.orientation
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        ###if (y > -1.3) and (y < -0.6) and (x > 0.5) and (x < 2.0) :#and (yaw > 0.10) and (yaw < 0.30):
        if (y > -1) and (y < -0.1) and (x > 1.0) and (x < 2.0) and (yaw > 0.10) and (yaw < 0.30):
            print('detecting')
            for i in range(5):
                colors = rospy.get_param('aruco_id_' + str(i))
                red, green, blue = colors['r'], colors['g'], colors['b']
                if self.segment(color_img,red,green,blue) == False:
                    break
                ((cx1,cy1),(cx2,cy2)) = self.segment(color_img,red,green,blue)
                odom_x1,odom_y1,odom_z1,a = self.cam_to_odom(depth_img,cx1,cy1)
                odom_x2,odom_y2,odom_z2,b = self.cam_to_odom(depth_img,cx2,cy2)
                odom_x = float((odom_x1 + odom_x2)/2)
                odom_y = float((odom_y1 + odom_y2)/2)
                odom_z = float((odom_z1 + odom_z2)/2)  
                rospy.set_param('door_id_'+str(i), {'x': odom_x, 'y': odom_y, 'z': odom_z}) 
            if (self.segment(color_img,red,green,blue) != False) :
                if not np.isnan(odom_y) :    
                #if a and b:
                    print(a,b,np.isnan(odom_y))
                    rospy.set_param('doors',1)
        #cv2.imshow('win',color_img)
        #cv2.waitKey(3)

    def segment(self,color_img,red,green,blue):
        low = np.array([blue-20,green-20,red-20])
        high = np.array([blue+20,green+20,red+20])
        mask = cv2.inRange(color_img,low,high)

        contours, heirarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        centre = list()
        if len(contours)>=2:
            for i in range(2):
                x_sum , y_sum = 0,0
                #cv2.imwrite('mask.jpg',mask)
                #cv2.imwrite('img.jpg',color_img)
                for cord in contours[i]:
                    x_sum += cord[0][0]
                    y_sum += cord[0][1]
                x_sum /= contours[i].shape[0]
                y_sum /= contours[i].shape[0]
                #centre[0] += int(x_sum/2)
                #centre[1] += int(y_sum/2)
                centre.append(tuple([int(x_sum), int(y_sum)]))
            center = tuple(centre)
            
        else :
            center = False
        # cv2.imshow('segment',mask)
        # cv2.waitKey(3)
        return center

    def cam_to_odom(self,depth_image,cx1,cy1,param=True):
        Z = depth_image[cy1,cx1]
        X = Z*(cx1 - 320.5)/320.255
        Y = Z*(cy1 - 240.5)/320.255
        now = rospy.Time.now()
        self.listener.waitForTransform('camera_depth_frame','map',now,rospy.Duration(4.0))
        msg = PointStamped()
        msg.header.frame_id = "camera_depth_frame"
        msg.point.x = X
        msg.point.y = Y
        msg.point.z = Z
        ret_msg = self.listener.transformPoint('map', msg)
        #(trans, rot) = self.listener.lookupTransform('camera_depth_frame','odom',now)
        #print(X,Y,Z)
        #print(ret_msg)
        #print('\n\n')
        odom_x = ret_msg.point.x
        odom_y = ret_msg.point.y
        odom_z = ret_msg.point.z
        xx = str(odom_x)
        xx.lower()
        yy = str(odom_y)
        yy.lower()

        #print(Z)

        if yy.find('nan')!=-1:
            print("methord 3")
            param = False
        

        return (odom_x,odom_y,odom_z,param)

def main(args):
    rospy.init_node("door_detector", anonymous=True)
    dd = door_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt or rospy.get_param('doors')==1:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
