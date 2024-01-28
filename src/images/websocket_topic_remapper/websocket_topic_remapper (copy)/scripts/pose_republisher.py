#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

def callback(data):
   # print("Callback Called")
   reciveData = PoseStamped()

   reciveData.pose = data.goal.target_pose.pose
   reciveData.header = data.goal.target_pose.header

   pub.publish(reciveData)
   # print(f'Recived Data = {reciveData}')
   
   
rospy.init_node('pose_republisher')
# print("pose_republisher has started")
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
sub = rospy.Subscriber('/move_base_simple/goal/goal', MoveBaseActionGoal, callback)
rospy.spin()


#Suscriber goal.target_pose.pose

#Publisher pose.position
