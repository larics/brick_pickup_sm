#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from math import sqrt

class KopterPatch:
  def __init__(self):
    self.manual_target_location = rospy.get_param("~manual_target_location")
    self.target_input_sub = rospy.Subscriber("kopter/patch_position_input", Vector3, self.patch_input_cb, queue_size=1)

    self.current_status = False
    self.status_sub = rospy.Subscriber("red_color_filter/kopter_patch/status", Bool, self.patch_status_cb, queue_size=1)
    

    self.patch_sub = rospy.Subscriber("visual_servo/centroid/transformed", Vector3, self.patch_cb, queue_size=1)
    self.patch_pub = rospy.Publisher("kopter/patch_position", Vector3, queue_size=1)
    
    
  def patch_input_cb(self, msg):
    if not self.manual_target_location:
      print("kopter_patch_node - manual target location input is disabled")
      return

    # Publish user input as the new patch position
    print("kopter_patch_node - publishing manual input to [{}, {}, {}]".format(msg.x, msg.y, msg.z))
    self.patch_pub.publish(msg)

  def patch_status_cb(self, msg):
    self.current_status = msg.data

  def patch_cb(self, msg):
    if self.manual_target_location:
      return

    if not self.current_status:
      return

    # Publish only if 
    self.patch_pub.publish(msg)

if __name__ == "__main__":
  rospy.init_node("kopter_patch_node")
  mag = KopterPatch()
  rospy.spin()