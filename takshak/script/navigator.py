#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
import sys



def goal_point(pos,ori):
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)

    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = pos[0]
    goal.pose.position.y = pos[1]
    goal.pose.position.z = pos[2]

    goal.pose.orientation.x = ori[0]
    goal.pose.orientation.y = ori[1]
    goal.pose.orientation.z = ori[2]
    goal.pose.orientation.w = ori[3]
    rospy.sleep(1)

    goal_publisher.publish(goal)



def main(args):

    rospy.init_node("navigator")
    rospy.set_param('goal_point',[[-9.85,-3.32,0],[0,0,0.6051864,0.7960838]])
    rospy.set_param('finish',0)
    
    
    #rospy.set_param('goal_point',[[-9.7,-3.7,0],[0,0,0.6051864,0.7960838]])
    point= rospy.get_param('goal_point')
    rospy.sleep(0.5)
    goal_point(point[0],point[1])
    while True:



        #goal_point(point[0],point[1])
        if point != rospy.get_param('goal_point'):
            point=rospy.get_param('goal_point')
            goal_point(point[0],point[1])
        rospy.sleep(0.01)
        if rospy.get_param('finish')==1:
            break

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == '__main__':
    main(sys.argv)
