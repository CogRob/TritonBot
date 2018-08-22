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


from __future__ import print_function

from absl import flags
from absl import logging
import grpc
import signal
import time
import threading

from openface_client.proto import openface_service_pb2
from openface_client.proto import openface_service_pb2_grpc
import openface_client.proto.common_pb2 as openface_common_pb2

import cogrob_face_msgs.msg as face_msgs
import sensor_msgs.msg as sensor_msgs
import cv_bridge
import cv2
import sys
import rospy

logging.set_verbosity(logging.INFO)
FLAGS = flags.FLAGS
flags.DEFINE_string("openface_server", "localhost:7008",
                    "OpenFace Server host:port")


class WaitToTerminate:
  def __init__(self):
    self._kill_now = False
    signal.signal(signal.SIGINT, self.ExitGracefully)
    signal.signal(signal.SIGTERM, self.ExitGracefully)
    while not self._kill_now:
      signal.pause()


  def ExitGracefully(self, signum, frame):
    self._kill_now = True


class HeadCamReporter(object):

  def __init__(self):
    channel = grpc.insecure_channel(FLAGS.openface_server)
    self._stub = openface_service_pb2_grpc.OpenfaceServiceStub(channel)

    self._bridge = cv_bridge.CvBridge()

    self._face_pub = rospy.Publisher(
      '/cogrob/detected_openface_embedding',
      face_msgs.DetectedOpenFaceEmbedding, queue_size=10)
    self._face_pub_seq = 0

    self._last_request_time = 0
    self._latest_image = None

    self._image_sub = rospy.Subscriber(
        "/head_camera/rgb/image_raw", sensor_msgs.Image, self._TopicCallback)

    self._openface_client_thread = threading.Thread(
        target=self._OpenfaceClientThread)
    self._openface_client_thread.daemon = True
    self._openface_client_thread.start()


  def _OpenfaceClientThread(self):
    while True:
      if time.time() - self._last_request_time < 0.3:
        time.sleep(0.05)
        continue

      if self._latest_image:
        start_time = time.time()
        self._last_request_time = time.time()

        try:
          latest_image = self._latest_image
          self._latest_image = None
          cv_image = self._bridge.imgmsg_to_cv2(latest_image, "bgr8")
        except cv_bridge.CvBridgeError as e:
          logging.error(e)
          continue

        _, buf = cv2.imencode(".bmp", cv_image)

        request = openface_service_pb2.GetEmbeddingRequest()
        request.image_format = openface_common_pb2.BMP_IMAGE
        request.image_data = buf.tostring()

        try:
          response = self._stub.GetEmbedding(request)
        except Exception as e:
          print("Error when GetEmbedding from OpenFace server: {}".format(
              str(e)))
          continue


        for face in response.faces:
          to_pub = face_msgs.DetectedOpenFaceEmbedding()
          to_pub.src_image_header = latest_image.header

          to_pub.header.stamp = rospy.Time.now()
          self._face_pub_seq += 1
          to_pub.header.seq = self._face_pub_seq

          to_pub.face_size = face.face_size
          to_pub.embedding = list(face.embedding)

          self._face_pub.publish(to_pub)

        elapsed_time = time.time() - start_time
        print("Found %d faces in %f seconds." % (len(response.faces),
                                                 elapsed_time))


  def _TopicCallback(self, data):
    self._latest_image = data


def main(argv):
  FLAGS(argv)
  logging.info("openface_client_node started.")
  rospy.init_node('openface_client_node')
  reporter = HeadCamReporter()

  rospy_spin_thread = threading.Thread(target=rospy.spin)
  rospy_spin_thread.daemon = True
  rospy_spin_thread.start()

  WaitToTerminate()


if __name__ == '__main__':
  main(sys.argv)
