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
from greeting_bot_state_machine import general_states
import rospy
import smach
import sys
import std_msgs.msg
import time

FLAGS = flags.FLAGS

class CheckRecentWakewordState(smach.State):
  def __init__(self, time_tolerance_sec=3):
    smach.State.__init__(
        self, outcomes=["wakeword_activated", "expired", "no_finding"],
        input_keys=["last_wakeword_timestamp"],
        output_keys=["last_wakeword_timestamp"])
    self._time_tolerance_sec = time_tolerance_sec
    self._updated_wakeword_timestamp = None
    self._wakeword_sub = rospy.Subscriber(
        "/cogrob/snowboy", std_msgs.msg.Header, self.WakewordCallback)


  def WakewordCallback(self, msg):
    self._updated_wakeword_timestamp = msg.stamp


  def execute(self, userdata):
    if self._updated_wakeword_timestamp is not None and (
        userdata.last_wakeword_timestamp is None or
        self._updated_wakeword_timestamp != userdata.last_wakeword_timestamp):
      userdata.last_wakeword_timestamp = self._updated_wakeword_timestamp
      if ((rospy.get_rostime() - userdata.last_wakeword_timestamp).to_sec()
          < self._time_tolerance_sec):
        rospy.loginfo("Found wakeword.")
        return "wakeword_activated"
      else:
        return "expired"
    return "no_finding"


class CheckRecentSeenFaceState(smach.State):
  def __init__(self, time_tolerance_sec=3):
    smach.State.__init__(
        self, outcomes=["face_activated", "expired", "no_finding"],
        input_keys=["last_face_timestamp"],
        output_keys=["last_face_timestamp"])
    self._time_tolerance_sec = time_tolerance_sec
    self._updated_face_timestamp = None
    self._face_sub = rospy.Subscriber(
        "/cogrob/detected_openface_embedding",
        face_msgs.DetectedOpenFaceEmbedding, self.FaceCallback)


  def FaceCallback(self, msg):
    self._updated_face_timestamp = msg.src_image_header.stamp


  def execute(self, userdata):
    if (self._updated_face_timestamp is not None
        and (userdata.last_face_timestamp is None or
        self._updated_face_timestamp != userdata.last_face_timestamp)):
      userdata.last_face_timestamp = self._updated_face_timestamp
      if ((rospy.get_rostime() - userdata.last_face_timestamp).to_sec()
          < self._time_tolerance_sec):
        rospy.loginfo("Found a face.")
        return "face_activated"
      else:
        return "expired"
    return "no_finding"


class TriggerCmdState(smach.State):
  def __init__(self, expected_cmds, valid_period=5):
    assert isinstance(expected_cmds, list)
    self._topic_sub = rospy.Subscriber(
        "/cogrob/trigger_cmd", std_msgs.msg.String, self._TopicCallback)
    self._valid_period = valid_period
    self._expected_cmds = expected_cmds
    self._last_cmd = None
    self._recv_time = None
    smach.State.__init__(self, outcomes=self._expected_cmds + ["none"])

  def _TopicCallback(self, msg):
    self._last_cmd = msg.data
    self._recv_time = time.time()


  def execute(self, userdata):
    if self._last_cmd is None:
      return "none"
    if time.time() - self._recv_time > self._valid_period:
      return "none"
    if self._last_cmd not in self._expected_cmds:
      return "none"
    result = self._last_cmd
    self._last_cmd = None
    return result


def GetWaitForTriggerMachine():
  sm = smach.StateMachine(
      outcomes=["wakeword", "face", "man_go_prep", "man_go_charge", "nothing"])

  sm.userdata.last_wakeword_timestamp = None
  sm.userdata.last_face_timestamp = None

  with sm:
    smach.StateMachine.add(
        "check_wakeword", CheckRecentWakewordState(),
        transitions={"wakeword_activated": "wakeword",
                     "expired": "check_face",
                     "no_finding": "check_face"},
        remapping={"last_wakeword_timestamp": "last_wakeword_timestamp"})
    sm.set_initial_state(["check_wakeword"])
    smach.StateMachine.add(
        "check_face", CheckRecentSeenFaceState(),
        transitions={"face_activated": "face",
                     "expired": "check_trigger_cmd",
                     "no_finding": "check_trigger_cmd"},
        remapping={"last_face_timestamp": "last_face_timestamp"})
    smach.StateMachine.add(
        "check_trigger_cmd", TriggerCmdState(["prep_serve", "charge"]),
        transitions={"prep_serve": "man_go_prep",
                     "charge": "man_go_charge",
                     "none": "nothing"})

  return sm


def main(argv):
  rospy.init_node("wait_trigger_states_test_node")

  sm = GetWaitForFaceOrWakewordStateMachine()
  while not rospy.is_shutdown():
    outcome = sm.execute()
    rospy.loginfo("wait_trigger_states outcome: %s", outcome)
    time.sleep(5)


if __name__ == "__main__":
  main(sys.argv)
