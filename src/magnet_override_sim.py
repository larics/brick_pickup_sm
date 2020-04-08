#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32

class MagnetOverride:

  def __init__(self):
    self.service_on = rospy.Service('magnet/override_ON', Empty, self.override_on_cb)
    self.service_off = rospy.Service('magnet/override_OFF', Empty, self.override_off_cb)
    self.magnet_pub = rospy.Publisher('/magnet_uav/gain', Float32, queue_size=1)

  def override_on_cb(self, req):
    print("MagnetOverride - turning ON magnet")
    offMsg = Float32()
    offMsg.data = 1.0
    self.magnet_pub.publish(offMsg)
    return EmptyResponse()
  
  def override_off_cb(self, req):
    print("MagnetOverride - turning OFF magnet")
    offMsg = Float32()
    offMsg.data = 0.0
    self.magnet_pub.publish(offMsg)
    return EmptyResponse()

if __name__ == "__main__":
  rospy.init_node("magnet_override_node")
  mag = MagnetOverride()
  rospy.spin()