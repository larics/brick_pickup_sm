#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from math import sqrt

class KopterPatch:
  def __init__(self):
    self.status_sub = rospy.Subscriber("red_color_filter/kopter_patch/status", Bool, self.patch_status_cb, queue_size=1)
    self.patch_sub = rospy.Subscriber("visual_servo/centroid/transformed", Vector3, self.patch_cb, queue_size=1)
    self.patch_pub = rospy.Publisher("kopter/patch_position", Vector3, queue_size=1)
    self.current_status = False
    self.current_centroid = Vector3()
    self.current_centroid.x = -1
    self.current_centroid.y = -1
    self.current_centroid.z = -1
    

  def patch_status_cb(self, msg):
    self.current_status = msg.data

  def patch_cb(self, msg):
    if not self.current_status:
      return
    self.patch_pub.publish(msg)

if __name__ == "__main__":
  rospy.init_node("kopter_patch_node")
  mag = KopterPatch()
  rospy.spin()