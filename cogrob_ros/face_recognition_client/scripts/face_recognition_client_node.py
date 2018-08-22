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

from cogrob_face_protos.proto.facedb_proto import facedb_service_pb2
from cogrob_face_protos.proto.facedb_proto import facedb_service_pb2_grpc
from cogrob_face_protos.proto.humandb_proto import humandb_record_pb2
from cogrob_face_protos.proto.humandb_proto import humandb_service_pb2
from cogrob_face_protos.proto.humandb_proto import humandb_service_pb2_grpc
from cogrob_face_protos.proto.util import uuid_pb2

import cogrob_face_msgs.msg as face_msgs
import sys
import rospy

logging.set_verbosity(logging.INFO)
FLAGS = flags.FLAGS
flags.DEFINE_string("facedb_server", "localhost:7009",
                    "FaceDB Server host:port")
flags.DEFINE_string("humandb_server", "localhost:7010",
                    "HumanDB Server host:port")


class WaitToTerminate:
  def __init__(self):
    self._kill_now = False
    signal.signal(signal.SIGINT, self.ExitGracefully)
    signal.signal(signal.SIGTERM, self.ExitGracefully)
    while not self._kill_now:
      signal.pause()


  def ExitGracefully(self, signum, frame):
    self._kill_now = True


def UuidPbToUuidMsg(uuid_pb):
  uuid_msg = face_msgs.Uuid()
  uuid_msg.most_significant_bits = uuid_pb.most_significant_bits
  uuid_msg.least_significant_bits = uuid_pb.least_significant_bits
  return uuid_msg


def UuidPbIsZero(uuid_pb):
  return (uuid_pb.most_significant_bits == 0
          and uuid_pb.least_significant_bits == 0)


def ProtoShortDebugString(proto):
  return " ".join(str(proto).split())


def PopulateHumanInformationMsgWithProto(dest, humandb_record_pb):
  dest.human_uuid = UuidPbToUuidMsg(humandb_record_pb.human_uuid)
  dest.human_uuid_aliases = map(UuidPbToUuidMsg,
                                  humandb_record_pb.human_uuid_aliases)
  dest.human_labels = list(humandb_record_pb.human_labels)
  dest.created_timestamp = rospy.Time.from_sec(
      humandb_record_pb.created_timestamp)
  dest.modified_timestamp = map(rospy.Time.from_sec,
                                  humandb_record_pb.modified_timestamp)

  dest.nicknames = list(humandb_record_pb.nicknames)
  dest.facedb_uuids = map(UuidPbToUuidMsg,
                            humandb_record_pb.facedb_uuids)


class OpenFaceToIdentityTranslator(object):

  def __init__(self):
    facedb_channel = grpc.insecure_channel(FLAGS.facedb_server)
    self._facedb_stub = facedb_service_pb2_grpc.FaceDbServiceStub(
        facedb_channel)

    humandb_channel = grpc.insecure_channel(FLAGS.humandb_server)
    self._humandb_stub = humandb_service_pb2_grpc.HumanDbServiceStub(
        humandb_channel)

    self._latest_embedding_msg = None

    self._openface_embedding_sub = rospy.Subscriber(
        '/cogrob/detected_openface_embedding',
        face_msgs.DetectedOpenFaceEmbedding, self._EmbeddingTopicCallback)

    self._detected_face_uuid_pub = rospy.Publisher(
      '/cogrob/detected_face_uuid',
      face_msgs.DetectedFaceUuid, queue_size=10)
    self._detected_face_uuid_pub_seq = 0

    self._detected_human_uuid_pub = rospy.Publisher(
      '/cogrob/detected_human_uuid',
      face_msgs.DetectedHumanUuid, queue_size=10)
    self._detected_human_uuid_pub_seq = 0

    self._human_information_pub = rospy.Publisher(
      '/cogrob/detected_human_information',
      face_msgs.HumanInformation, queue_size=10)
    self._human_information_pub_seq = 0

    self._db_client_thread = threading.Thread(
        target=self._FaceDbHumanDbClientThread)
    self._db_client_thread.daemon = True
    self._db_client_thread.start()


  def _FaceDbHumanDbClientThread(self):
    while True:
      if not self._latest_embedding_msg:
        # Skips this iteration if there is no embedding come in.
        time.sleep(0.05)
        continue

      embedding_msg = self._latest_embedding_msg
      self._latest_embedding_msg = None

      start_time = time.time()

      facedb_query = facedb_service_pb2.QueryRequest()
      facedb_query.embedding.extend(embedding_msg.embedding)
      try:
        facedb_result = self._facedb_stub.Query(facedb_query)
      except Exception as e:
        print("FaceDB error", e)
        logging.error(e)
        continue

      face_uuid_pb = facedb_result.facedb_uuid

      # Publishes FaceDb UUID, it should be useful for debugging.
      topub_detected_face_uuid = face_msgs.DetectedFaceUuid()

      topub_detected_face_uuid.header.stamp = rospy.Time.now()
      self._detected_face_uuid_pub_seq += 1
      topub_detected_face_uuid.header.seq = self._detected_face_uuid_pub_seq
      topub_detected_face_uuid.src_header = embedding_msg.src_image_header

      topub_detected_face_uuid.face_uuid = UuidPbToUuidMsg(face_uuid_pb)

      self._detected_face_uuid_pub.publish(topub_detected_face_uuid)

      if UuidPbIsZero(face_uuid_pb):
        # We don't have a match here, don't publish anything.
        print("FaceDB does not have a match for this face.")
        continue
      else:
        print("Lookup FaceUUID [{}] in HumanDB".format(
            ProtoShortDebugString(face_uuid_pb)))
        humandb_query = humandb_service_pb2.QueryRequest()
        humandb_query.facedb_uuid.CopyFrom(face_uuid_pb)
        try:
          humandb_result = self._humandb_stub.Query(humandb_query)
        except Exception as e:
          print("HuamnDB error", e)
          logging.error(e)
          continue

        print("Human DB returns {} records.".format(
            len(humandb_result.records)))
        for record in humandb_result.records:
          # Publishes HumanDb UUID
          topub_detected_human_uuid = face_msgs.DetectedHumanUuid()

          topub_detected_human_uuid.header.stamp = rospy.Time.now()
          self._detected_human_uuid_pub_seq += 1
          topub_detected_human_uuid.header.seq = (
              self._detected_human_uuid_pub_seq)

          topub_detected_human_uuid.src_header = embedding_msg.src_image_header

          topub_detected_human_uuid.human_uuid = UuidPbToUuidMsg(
              record.human_uuid)
          self._detected_human_uuid_pub.publish(topub_detected_human_uuid)


          # Publishes all HuamnDb information. This ideally should be in another
          # node, but for now do it here to make debugging easier.
          topub_human_information = face_msgs.HumanInformation()
          PopulateHumanInformationMsgWithProto(topub_human_information, record)
          topub_human_information.header.stamp = rospy.Time.now()
          self._human_information_pub_seq += 1
          topub_human_information.header.seq = self._human_information_pub_seq
          topub_human_information.src_header = embedding_msg.src_image_header
          self._human_information_pub.publish(topub_human_information)


      elapsed_time = time.time() - start_time


  def _EmbeddingTopicCallback(self, data):
    self._latest_embedding_msg = data


def main(argv):
  FLAGS(argv)
  logging.info("face_recognition_client_node started.")
  rospy.init_node('face_recognition_client_node')
  translator = OpenFaceToIdentityTranslator()

  rospy_spin_thread = threading.Thread(target=rospy.spin)
  rospy_spin_thread.daemon = True
  rospy_spin_thread.start()

  WaitToTerminate()


if __name__ == '__main__':
  main(sys.argv)
