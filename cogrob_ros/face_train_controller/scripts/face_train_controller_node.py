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
import Queue
import signal
import sys
import time
import threading

import rospy
import std_msgs.msg as std_msgs
import cogrob_face_msgs.msg as face_msgs
import cogrob_face_msgs.srv as face_srvs

from cogrob_face_protos.proto.facedb_proto import facedb_service_pb2
from cogrob_face_protos.proto.facedb_proto import facedb_service_pb2_grpc
from cogrob_face_protos.proto.humandb_proto import humandb_record_pb2
from cogrob_face_protos.proto.humandb_proto import humandb_service_pb2
from cogrob_face_protos.proto.humandb_proto import humandb_service_pb2_grpc
from cogrob_face_protos.proto.util import uuid_pb2

logging.set_verbosity(logging.INFO)
FLAGS = flags.FLAGS
flags.DEFINE_string("facedb_server", "localhost:7009",
                    "FaceDB Server host:port")
flags.DEFINE_string("humandb_server", "localhost:7010",
                    "HumanDB Server host:port")
flags.DEFINE_integer("embedding_alive_secs", 60, "Embeeding alive lifespan.")
flags.DEFINE_integer("embedding_sufficient_threshold", 15,
                     "Sufficent embeeding count threshold.")


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
  dest.facedb_uuids = map(UuidPbToUuidMsg, humandb_record_pb.facedb_uuids)


class OpenFaceFaceDbHumanDbTrainningController(object):

  def __init__(self):
    facedb_channel = grpc.insecure_channel(FLAGS.facedb_server)
    self._facedb_stub = facedb_service_pb2_grpc.FaceDbServiceStub(
        facedb_channel)

    humandb_channel = grpc.insecure_channel(FLAGS.humandb_server)
    self._humandb_stub = humandb_service_pb2_grpc.HumanDbServiceStub(
        humandb_channel)

    self._embedding_msg_queue = Queue.PriorityQueue()

    self._openface_embedding_sub = rospy.Subscriber(
        "/cogrob/detected_openface_embedding",
        face_msgs.DetectedOpenFaceEmbedding, self._EmbeddingTopicCallback)

    self._label_seen_person_sub = rospy.Subscriber(
        "/cogrob/label_seen_person",
        std_msgs.String, self._LabelSeenPersonCallback)

    self._add_new_human_with_recent_face_srv = rospy.Service(
        "/cogrob/add_new_human_with_recent_face",
        face_srvs.AddNewHumanWithRecentFace,
        self._AddNewHumanWithRecentFaceSrvCallback)

    self._clear_embedding_cache_srv = rospy.Service(
        "/cogrob/clear_face_embedding_cache",
        face_srvs.ClearEmbeddingCache,
        self._ClearEmbeddingCacheCallback)

    self._get_available_embedding_count_srv = rospy.Service(
        "/cogrob/get_available_face_embedding_count",
        face_srvs.GetAvailableEmbeddingCount,
        self._GetAvailableEmbeddingCountCallback)


  def _LabelSeenPersonCallback(self, msg):
    self._Train(labels=[msg.data])


  def _AddNewHumanWithRecentFaceSrvCallback(self, req):
    humandb_uuid = self._Train(req.human_labels, req.nicknames)
    result = face_srvs.AddNewHumanWithRecentFaceResponse()
    if humandb_uuid is None:
      result.error = True
    else:
      result.human_uuid = UuidPbToUuidMsg(humandb_uuid)
    return result


  def _Train(self, labels=None, nicknames=None):
    if labels is None:
      labels = []
    if nicknames is None:
      nicknames = []

    # For now, we always create a new person in HumanDB and tag him/her with a
    # label or nickname.

    print("Got a trainning request, labels are {}, nicknames are {}".format(
        labels, nicknames))

    self._ClearExpiredEmbeddings()

    # Takes a snapshot of the current embedding msg queue.
    # This is not listed in API page, but it works. There could be some
    # concurrency issue, but rospy is single-threaded. For simplicity use this
    # for now.
    trainning_samples = map(lambda x:x[1], self._embedding_msg_queue.queue)

    if len(trainning_samples) == 0:
      print("Trainning rejected, no samples")
      return None
    else:
      print("Trainning on {} samples.".format(len(trainning_samples)))

    # First, ask FaceDB to generate a face UUID for this human.
    facedb_request = facedb_service_pb2.RegisterRequest()
    for embedding_msg in trainning_samples:
      facedb_request.embeddings.add().embedding.extend(embedding_msg.embedding)

    # Contact the FaceDB and register the face.
    # TODO(shengye): Set a deadline for FaceDB and HumanDB. We can't block
    # forever on these requests.
    print("Register with FaceDB, request:{}".format(str(facedb_request)))
    try:
      print("Register with FaceDB, request:{}".format(str(facedb_request)))
      facedb_response = self._facedb_stub.Register(facedb_request)
      print("Registered with FaceDB, response: {}".format(str(facedb_response)))
    except Exception as e:
      print(e)
      logging.error(e)
      return None

    # Now contact HumanDB and create a new human entry.
    humandb_request = humandb_service_pb2.CreateOrAppendRequest()
    humandb_record = humandb_request.record
    humandb_record.human_labels.extend(labels)
    humandb_record.nicknames.extend(nicknames)
    humandb_record.facedb_uuids.add().CopyFrom(facedb_response.facedb_uuid)
    try:
      humandb_response = self._humandb_stub.CreateOrAppend(humandb_request)
    except Exception as e:
      print(e)
      logging.error(e)
      return None

    return humandb_response.result.human_uuid


  def _ClearExpiredEmbeddings(self):
    try:
      oldest_img = self._embedding_msg_queue.get_nowait()
      while (rospy.Time.now() - oldest_img[1].src_image_header.stamp >
             rospy.Duration(FLAGS.embedding_alive_secs)):
        oldest_img = self._embedding_msg_queue.get_nowait()
      # get() removes the item from the queue, so we need to put it back.
      self._embedding_msg_queue.put(oldest_img)
    except Queue.Empty as e:
      # The queue is empty, it is ok.
      return


  def _ClearAllEmbeddings(self):
    while True:
      try:
        # Drain the queue
        self._embedding_msg_queue.get_nowait()
      except Queue.Empty as e:
        break
    return True


  def _GetAvailableEmbeddingCount(self):
    return self._embedding_msg_queue.qsize()


  def _GetAvailableEmbeddingCountCallback(self, req):
    del req
    result = face_srvs.GetAvailableEmbeddingCountResponse()
    result.count = self._GetAvailableEmbeddingCount()
    if result.count >= FLAGS.embedding_sufficient_threshold:
      result.is_sufficient = True
    else:
      result.is_sufficient = False
    return result


  def _ClearEmbeddingCacheCallback(self, req):
    del req
    result = face_srvs.ClearEmbeddingCacheResponse()
    result.is_success = self._ClearAllEmbeddings()
    return result


  def _EmbeddingTopicCallback(self, data):
    self._embedding_msg_queue.put((data.src_image_header.stamp, data))
    self._ClearExpiredEmbeddings()


def main(argv):
  FLAGS(argv)
  logging.info("face_train_controller_node started.")
  rospy.init_node('face_train_controller_node')
  controller = OpenFaceFaceDbHumanDbTrainningController()

  rospy_spin_thread = threading.Thread(target=rospy.spin)
  rospy_spin_thread.daemon = True
  rospy_spin_thread.start()

  WaitToTerminate()


if __name__ == '__main__':
  main(sys.argv)
