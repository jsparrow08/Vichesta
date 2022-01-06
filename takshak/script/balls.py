#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import sys

import os

def main(args):
    path= os.path.abspath("map.pgm")


    rospy.init_node("ball", anonymous=True)
    img = cv2.imread(path,-1)
    #img=img[1700:2500,1400:2500]
    img[1800:2100,1890:2000]=255
    img=img[1800:2300,1890:2150]
    image=cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    img = cv2.GaussianBlur(img, (3,3), 0)
    (thresh, img) = cv2.threshold(img, 195, 255, cv2.THRESH_BINARY)
    #edges = cv2.Canny(image=img, threshold1=100, threshold2=200)

    countours,_=cv2.findContours(img,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)

    count_balls=0
    list=[]
    for i in range(int(len(countours))):
        max_x=0
        min_x=10000
        max_y=0
        min_y=10000
        x=countours[i][0][0][0]
        y=countours[i][0][0][1]
        a_x=0
        a_y=0
        for j in range(len(countours[i])):
            if(max_x<countours[i][j][0][0]):
                max_x=countours[i][j][0][0]
            if(max_y<countours[i][j][0][1]):
                max_y=countours[i][j][0][1]
            if(min_y>countours[i][j][0][1]):
                min_y=countours[i][j][0][1]
            if(min_x>countours[i][j][0][0]):
                min_x=countours[i][j][0][0]
            if(countours[i][j][0][0]==x or countours[i][j][0][0]==x-1 or countours[i][j][0][0]==x+1):
                a_x+=1
            if(countours[i][j][0][1]==y or countours[i][j][0][1]==y-1 or countours[i][j][0][1]==y+1):
                a_y+=1
        if((max_x-min_x>7) and (max_y-min_y>8)):
            if(cv2.contourArea(countours[i])<700 and cv2.contourArea(countours[i])>50):
                if(a_x<17 and a_y<17):
                    count_balls+=1
                    list.append(i)
    for i in list:
        cv2.drawContours(image,countours,i, (0, 255, 0), 1)
    print(count_balls)
    print(count_balls%5)
    id=count_balls%5
    # x =rospy.get_param('/door_id_'+str(id)+'/x')
    # x = max(7.5 ,min(8.5,x))
    x=7.8
    y=rospy.get_param('/door_id_'+str(id)+'/y')
    rospy.set_param('gate',[[x,y,0],[0,0,0,1]])
    rospy.set_param('gate_open',1)
    #print(list)

    # cv2.imshow('map',image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == '__main__':
    main(sys.argv)
