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
import cogrob_face_msgs.msg as face_msgs
import collections
from greeting_bot_state_machine import proto_utils
import grpc
import rospy
import time

import use_cogrob_workspace
from cogrob.perception.face_db.proto import facedb_service_pb2
from cogrob.perception.face_db.proto import facedb_service_pb2_grpc
from cogrob.identity.human_db.proto import humandb_service_pb2
from cogrob.identity.human_db.proto import humandb_service_pb2_grpc

FLAGS = flags.FLAGS

flags.DEFINE_string("facedb_server", "localhost:7009",
                    "FaceDB Server host:port")
flags.DEFINE_string("humandb_server", "localhost:7010",
                    "HumanDB Server host:port")


class TestRecentSeenIdentityHelper(object):
  CandidateRecord = collections.namedtuple(
      "CandidateRecord",
      ["capture_time", "face_uuid", "human_uuid", "human_nicknames"])

  def __init__(self, time_tolerance_sec=3):
    self._time_tolerance_sec = time_tolerance_sec

    self._face_sub = rospy.Subscriber(
        "/cogrob/detected_openface_embedding",
        face_msgs.DetectedOpenFaceEmbedding, self.FaceCallback)

    facedb_channel = grpc.insecure_channel(FLAGS.facedb_server)
    self._facedb_stub = facedb_service_pb2_grpc.FaceDbServiceStub(
        facedb_channel)
    humandb_channel = grpc.insecure_channel(FLAGS.humandb_server)
    self._humandb_stub = humandb_service_pb2_grpc.HumanDbServiceStub(
        humandb_channel)

    self._candidate_queue = []


  def ClearExpiredCandidates(self):
    self._candidate_queue = filter(
        lambda x: x.capture_time > time.time() - self._time_tolerance_sec,
        self._candidate_queue)


  def FaceCallback(self, msg):
    # Insert to queue, clear expired item in the queue
    self.ClearExpiredCandidates()

    capture_time = msg.src_image_header.stamp.to_sec()
    face_uuid = None
    human_uuid = None
    human_nicknames = None

    fail_flag = False

    facedb_query = facedb_service_pb2.QueryRequest()
    facedb_query.embedding.extend(msg.embedding)
    try:
      facedb_result = self._facedb_stub.Query(facedb_query)
    except Exception as e:
      rospy.logerr("FaceDB error: %s", e)
      fail_flag = True

    face_uuid = facedb_result.facedb_uuid

    if proto_utils.UuidPbIsZero(facedb_result.facedb_uuid):
      fail_flag = True

    if not fail_flag:
      humandb_query = humandb_service_pb2.QueryRequest()
      humandb_query.facedb_uuid.CopyFrom(face_uuid)
      try:
        humandb_result = self._humandb_stub.Query(humandb_query)
      except Exception as e:
        rospy.logerr("HumanDB error: %s", e)
        fail_flag = True

      rospy.loginfo(
          "Human DB returned {} records.".format(len(humandb_result.records)))

      if len(humandb_result.records) == 0:
        fail_flag = True
      else:

        # For now, only use the first record with a nick name. If there is no
        # such record, try human_label, otherwise use the first record.
        # TODO(shengye): This needs to be improved.
        humandb_record = humandb_result.records[0]
        human_uuid = humandb_record.human_uuid
        human_nicknames = humandb_record.nicknames

        humandb_records = filter(
            lambda x: len(x.human_labels) > 0, humandb_result.records)
        if humandb_records:
          human_uuid = humandb_record.human_uuid
          human_nicknames = humandb_record.human_labels

        humandb_records = filter(
            lambda x: len(x.nicknames) > 0, humandb_result.records)
        if humandb_records:
          human_uuid = humandb_record.human_uuid
          human_nicknames = humandb_record.nicknames

    # Let's assume there will not be a face that has a record in the FaceDB,
    # but no record in the HumanDB. If that is the case, we should remove the
    # FaceDB record. We can make a script and "audit" FaceDB nightly and clear
    # these records.

    self._candidate_queue.append(
        self.CandidateRecord(capture_time=capture_time, face_uuid=face_uuid,
                             human_uuid=human_uuid,
                             human_nicknames=human_nicknames))


  def GetMostVotedHuman(self):
    self.ClearExpiredCandidates()
    candidates = list(self._candidate_queue)
    candidates.sort()

    if not candidates:
      return None  # No person

    # Here performance is not cirtical, use an O(n^2) algorithm to find the best
    # match.
    most_voted_count = 0
    most_voted_item = None

    for target in candidates:
      current_count = 0
      for item in candidates:
        if proto_utils.UuidPbEqual(item.human_uuid, target.human_uuid):
          current_count += 1
      if current_count >= most_voted_count:
        most_voted_count = current_count
        most_voted_item = target

    return most_voted_item
