#!/usr/bin/env python
import rospy
import sys
import roslaunch
from subprocess import Popen  
import subprocess
from nav_msgs.msg import Odometry
import tf
import os


def main(args):
    rospy.init_node("controller")


    #rospy.set_param('goal_point',[[-9.65,-2.7,0],[0,0,0.6051864,0.7960838]])
    rospy.set_param('aruco',0)
    rospy.set_param('gate_open',0)
    
    while rospy.get_param('aruco') == 0:
        rospy.sleep(0.005)
    #aruco.kill()
    ###rospy.set_param('goal_point',[[1.4221,-1,0],[ 0, 0, 0.1494381, 0.9887711 ]])
    rospy.set_param('goal_point',[[1.569,-0.66,0],[ 0, 0, 0.0998334, 0.9950042 ]])
    
    
    rospy.set_param('doors',0)
    door=subprocess.Popen(["rosrun", "takshak", "door_detection.py"])
    while rospy.get_param('doors') == 0:
        rospy.sleep(0.005)
    door.kill()



    #rospy.set_param('goal_point',[[6.9,6.5,0],[ 0, 0, 0.3662725, 0.9305076 ]])#42
    rospy.set_param('goal_point',[[6.9,6.5,0],[ 0, 0, 0.3173047, 0.9483237 ]]) #37
    rospy.set_param('map_down',0)
    while rospy.get_param('map_down')==0 :
        rospy.sleep(0.1)
    # for i in range(50):
    #     print(1)
    # if (x>6.3) and (y>5.0) :
    path= os.path.abspath("map.pgm")
    print(path)
    mapp=subprocess.Popen(["rosrun", "map_server", "map_saver"])
    rospy.sleep(1)
    mapp.kill()
    rospy.sleep(0.1)

    counting=subprocess.Popen(["rosrun", "takshak", "balls.py"])


    while rospy.get_param('gate_open')==0 :
        rospy.sleep(0.1)
    
    goal_1 =rospy.get_param('gate')
    rospy.set_param('goal_point',goal_1)
    


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == '__main__':
    main(sys.argv)
