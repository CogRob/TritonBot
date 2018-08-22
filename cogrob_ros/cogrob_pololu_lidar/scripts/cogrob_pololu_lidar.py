#!/usr/bin/env python
# Copyright (c) 2018, The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the University of California nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import math
import serial
import sys
import threading

import rospy
import std_msgs.msg
import tf

class PololuSerial(object):

  def __init__(self, tty_device='/dev/ttyACM0'):
    self._serial = serial.Serial(tty_device, timeout=0.2)
    self._lock = threading.Lock()


  def _SendCommand(self, command_sequence, reply_length=0):
    # TODO(shengye): Check command_sequence is an iterable of integers [0, 255]
    buf_out = bytearray(command_sequence)
    with self._lock:
      self._serial.write(buf_out)
      if reply_length > 0:
        buf_in = bytearray(self._serial.read(reply_length))
        assert len(buf_in) == reply_length
      else:
        buf_in = None
    return buf_in


  def Close(self):
    with self._lock:
      self._serial.close()
      self._serial = None


  def SetTarget(self, target_val, channel=0):
    command = [0x84, channel, target_val & 0x7F, (target_val >> 7) & 0x7F]
    self._SendCommand(command)


  def SetSpeed(self, speed_val, channel=0):
    command = [0x87, channel, speed_val & 0x7F, (speed_val >> 7) & 0x7F]
    self._SendCommand(command)


  def SetAcceleration(self, acc_val, channel=0):
    command = [0x89, channel, acc_val & 0x7F, (acc_val >> 7) & 0x7F]
    self._SendCommand(command)


  def GetPos(self, channel=0):
    command = [0x90, channel]
    result = self._SendCommand(command, 2)
    return_val = (result[1] << 8) | result[0]
    return return_val


def main(argv):
  rospy.init_node("head_laser_servo_tf")

  # Parameters
  tty_device = rospy.get_param("~tty_device", "/dev/ttyACM0")
  acceleration = rospy.get_param("~acceleration", 20)
  speed = rospy.get_param("~speed", 10)

  min_val = rospy.get_param("~min_val", 885 * 4)
  min_deg = rospy.get_param("~min_deg", -90)
  max_val = rospy.get_param("~max_val", 1900 * 4)
  max_deg = rospy.get_param("~max_deg", 0)

  default_deg = rospy.get_param("~default_deg", -90)

  fixed_frame = rospy.get_param("~fixed_frame", "head_laser_servo_base")
  rotating_frame = rospy.get_param("~rotating_frame",
                                   "head_laser_servo_mount")

  time_adj = rospy.get_param("~time_adj", 0.125)
  tf_pub_rate = rospy.get_param("~tf_pub_rate", 20)

  dev = PololuSerial(tty_device)
  dev.SetAcceleration(acceleration)
  dev.SetSpeed(speed)

  tf_broadcaster = tf.TransformBroadcaster()
  disable_tf_publisher = [False]
  latest_deg = [min_deg]

  def MoveToDeg(target_deg):
    target = int((target_deg - min_deg) / (max_deg - min_deg) *
                 (max_val - min_val) + min_val)
    dev.SetTarget(target)
    pos = float(dev.GetPos())
    disable_tf_publisher[0] = True
    while pos != target:
      deg = ((pos - min_val) / (max_val - min_val) * (max_deg - min_deg)
             + min_deg)
      tf_broadcaster.sendTransform(
          (0, 0, 0),
          tf.transformations.quaternion_from_euler(
              0, -deg / 180.0 * math.pi, 0),
          rospy.Time.now() + rospy.Duration(time_adj),
          rotating_frame,
          fixed_frame)
      latest_deg[0] = deg
      rospy.loginfo("Degree: %f, Value: %f", deg, pos)
      pos = float(dev.GetPos())
    disable_tf_publisher[0] = False

  def HeadLaserAngleCallback(data):
    angle = data.data
    if angle < min_deg or angle > max_deg:
      rospy.logerr("%f is not between [%f, %f]", angle, min_deg, max_deg)
    else:
      MoveToDeg(angle)

  MoveToDeg(min_deg)
  MoveToDeg(max_deg)
  MoveToDeg(default_deg)

  rospy.Subscriber("/head_laser/angle", std_msgs.msg.Float64,
                   HeadLaserAngleCallback)

  rospy.loginfo("Ready to serve.")

  tf_pub_rate_rospy_rate = rospy.Rate(tf_pub_rate)
  while not rospy.is_shutdown():
    if not disable_tf_publisher[0]:
      tf_broadcaster.sendTransform(
          (0, 0, 0),
          tf.transformations.quaternion_from_euler(
              0, -latest_deg[0] / 180.0 * math.pi, 0),
          rospy.Time.now() + rospy.Duration(time_adj),
          rotating_frame,
          fixed_frame)
    tf_pub_rate_rospy_rate.sleep()

  dev.Close()


if __name__ == "__main__":
  main(sys.argv)
