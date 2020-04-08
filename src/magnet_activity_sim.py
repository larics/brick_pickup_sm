#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool
from math import sqrt

class MagnetActivity:
  def __init__(self):
    self.mag_sub = rospy.Subscriber("/magnet_uav/mfs", MagneticField, self.mag_cb, queue_size=1)
    self.activity_pub = rospy.Publisher("brick_attached", Bool, queue_size=1)

  def mag_cb(self, msg):
    mag_field = sqrt(msg.magnetic_field.x ** 2 + msg.magnetic_field.y ** 2 + msg.magnetic_field.z ** 2)
    newMsg = Bool()
    if mag_field > 1e-5:
      newMsg.data = True
    else:
      newMsg.data = False
    self.activity_pub.publish(newMsg)

if __name__ == "__main__":
  rospy.init_node("magnet_activity_node")
  mag = MagnetActivity()
  rospy.spin()