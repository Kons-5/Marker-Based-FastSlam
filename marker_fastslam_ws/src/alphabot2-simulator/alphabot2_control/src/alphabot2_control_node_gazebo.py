#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import time

class control_node:

  def __init__(self):
    """ Initialize control node """
    rospy.init_node('alphabot2_control_node_gazebo', anonymous= True)

    """ Subscribe to alphabot2/control topic of Twist type"""
    self.sub = rospy.Subscriber('/alphabot2/control', Twist, self.callback)

    """ Initialize publisher to Gazebo"""
    self.gazebo_pub = rospy.Publisher('/alphabot2/control', Twist, queue_size=1)

  def callback(self, message_received):
    """ Sending to Gazebo """
    self.gazebo_pub.publish(message_received)

def main():
  cn = control_node()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    print ("Shutting down alphabot2 control node.")
    pass
