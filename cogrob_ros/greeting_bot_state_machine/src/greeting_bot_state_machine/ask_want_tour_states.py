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
import greeting_bot_state_machine
import greeting_bot_state_machine.robot_speak
from greeting_bot_state_machine import general_states
import rospy
import smach
import smach_ros
import sys
import time

from cogrob_listen_intent import common_intent_listener

SayText = greeting_bot_state_machine.robot_speak.SayText

FLAGS = flags.FLAGS


class AskWantTourState(smach.State):
  def __init__(self):
    smach.State.__init__(
      self, outcomes=["want_tour", "dont_want_tour", "silence"])


  def execute(self, userdata):
    SayText("I am a tour guide. Can I show you around?")
    listener = common_intent_listener.GetCommonIntentListener(
        ["yes_no"], blank_timeout=3)
    result = listener.ListenForIntent()
    if result.intent.HasField("silence"):
      return "silence"
    elif result.intent.yes_no.is_yes:
      return "want_tour"
    else:
      return "dont_want_tour"


def GetAskWantTourStateMachine():
  sm = smach.StateMachine(outcomes=["want_tour", "dont_want_tour", "silence"])

  with sm:
    smach.StateMachine.add(
        "reset_ask_want_tour_retried",
        general_states.ResetRetryCounter("ask_want_tour_retry", 2),
        transitions={"next": "decrease_and_test_ask_want_tour"},
        remapping={"ask_want_tour_retry": "ask_want_tour_retry"})
    sm.set_initial_state(["reset_ask_want_tour_retried"])

    smach.StateMachine.add(
        "decrease_and_test_ask_want_tour",
        general_states.DecreaseAndTestRetry("ask_want_tour_retry"),
        transitions={"continue": "ask_want_tour",
                     "give_up": "silence"},
        remapping={"ask_want_tour_retry": "ask_want_tour_retry"})

    smach.StateMachine.add(
        "ask_want_tour", AskWantTourState(),
        transitions={"want_tour": "want_tour",
                     "dont_want_tour": "dont_want_tour",
                     "silence": "decrease_and_test_ask_want_tour"})
  return sm


def main(argv):
  rospy.init_node("ask_want_tour_states_test_node")
  FLAGS(argv)

  sm = GetAskWantTourStateMachine()

  smach_introspection_server = smach_ros.IntrospectionServer(
      "ask_want_tour_states_test", sm, "/SM_ROOT")
  smach_introspection_server.start()

  outcome = sm.execute()
  rospy.loginfo("State machine finished with outcome: %s", outcome)
  time.sleep(1)

  # Prevent ROS from crashing at shutdown.
  # https://github.com/ros/ros_comm/issues/527
  smach_introspection_server.stop()
  time.sleep(.1)


if __name__ == "__main__":
  main(sys.argv)
