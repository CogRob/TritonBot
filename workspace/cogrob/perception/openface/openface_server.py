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

from absl import flags
from absl import logging
import concurrent.futures
import cv2
import grpc
import itertools
import numpy as np
import openface
import os
import signal
import sys
import time

from cogrob.perception.openface.proto import openface_service_pb2
from cogrob.perception.openface.proto import openface_service_pb2_grpc
import cogrob.perception.openface.proto.common_pb2 as openface_common_pb2
import cogrob.universal_logger.proto.archive_entry_pb2 as archive_entry_pb2
import cogrob.universal_logger.universal_logger as universal_logger

FLAGS = flags.FLAGS

flags.DEFINE_string(
  "dlib_model",
  "/opt/openface/models/dlib/shape_predictor_68_face_landmarks.dat",
  "Path to dlib's face predictor.")
flags.DEFINE_string(
  "openface_net_model",
  "/opt/openface/models/openface/nn4.small2.v1.t7",
  "Path to Openface Torch network model.")
flags.DEFINE_integer("openface_image_dimension", 96,
                      "Default Openface image dimension.")


class WaitToTerminate:
  def __init__(self):
    self._kill_now = False
    signal.signal(signal.SIGINT, self.ExitGracefully)
    signal.signal(signal.SIGTERM, self.ExitGracefully)
    while not self._kill_now:
      signal.pause()


  def ExitGracefully(self, signum, frame):
    self._kill_now = True


class OpenfaceServer(openface_service_pb2_grpc.OpenfaceServiceServicer):

  def __init__(self, aligned_face_logger=None):
    self._align = openface.AlignDlib(FLAGS.dlib_model)
    self._image_dim = FLAGS.openface_image_dimension
    self._net = openface.TorchNeuralNet(FLAGS.openface_net_model,
                                        self._image_dim)
    self._aligned_face_logger = aligned_face_logger


  def GetEmbedding(self, request, context):
    response = openface_service_pb2.GetEmbeddingResponse()

    # Checks if the incoming image format is supported.
    supported_formats = (openface_common_pb2.BMP_IMAGE,
                         openface_common_pb2.JPG_IMAGE)
    if request.image_format not in supported_formats :
      context.set_details("Image type not supported.")
      context.set_code(grpc.StatusCode.UNIMPLEMENTED)
      return response

    # Loads image.
    np_image_data = np.fromstring(request.image_data, np.uint8)
    cv_image = cv2.imdecode(np_image_data, cv2.CV_LOAD_IMAGE_COLOR)
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    image_size = rgb_image.shape[0] * rgb_image.shape[1]

    # Gets and sorts the bounding_boxs, the larger face will have smaller index.
    bounding_boxs = self._align.getAllFaceBoundingBoxes(rgb_image)
    bounding_boxs = sorted(bounding_boxs, key=lambda x:x.area(), reverse=True)

    for bounding_box in bounding_boxs:
      aligned_face = self._align.align(
          self._image_dim, rgb_image, bounding_box,
          landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
      if aligned_face is None:
        logging.error("Cannot align face.")
        continue
      size_ratio = float(bounding_box.area()) / image_size
      rep = self._net.forward(aligned_face)

      result_face = response.faces.add()
      result_face.embedding.extend(rep)
      result_face.face_size = size_ratio
      result_face.process_methods.append(
          openface_common_pb2.DLIB_ALIGN_SHAPE_PREDICTOR_68_LANDMARKS_OUTER_EYES_NOSE)
      result_face.process_methods.append(
          openface_common_pb2.OPENFACE_NET_FORWARD_NN4_SMALL2_V1_T7)

      # Write to log using UniversalLogger.
      if self._aligned_face_logger is not None:
        univseral_log = archive_entry_pb2.ArchiveEntry();

        univseral_log.aligned_face.process_methods.append(
            openface_common_pb2.DLIB_GET_ALL_FACE_BOUNDINGBOXES)
        univseral_log.aligned_face.process_methods.append(
            openface_common_pb2.DLIB_ALIGN_SHAPE_PREDICTOR_68_LANDMARKS_OUTER_EYES_NOSE)
        univseral_log.aligned_face.process_methods.append(
            openface_common_pb2.OPENFACE_NET_FORWARD_NN4_SMALL2_V1_T7)

        univseral_log.aligned_face.embedding.extend(rep)

        univseral_log.aligned_face.image_format = openface_common_pb2.JPG_IMAGE
        cv_bgrface = cv2.cvtColor(aligned_face, cv2.COLOR_RGB2BGR)
        _, buf = cv2.imencode(".jpg", cv_bgrface)
        univseral_log.aligned_face.image_data = buf.tostring()

        self._aligned_face_logger.Log(univseral_log)

    return response


def main(argv):
  FLAGS(argv)
  server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))

  aligned_face_logger = universal_logger.UniversalLogger(
      "perception/aligned_face");
  openface_server_instance = OpenfaceServer(
    aligned_face_logger=aligned_face_logger)

  openface_service_pb2_grpc.add_OpenfaceServiceServicer_to_server(
      openface_server_instance, server)
  server.add_insecure_port('[::]:7008')
  server.start()

  logging.info("Openface server is running.")
  WaitToTerminate()
  server.stop(1)
  aligned_face_logger.Close()


if __name__ == "__main__":
  main(sys.argv)
