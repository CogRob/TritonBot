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
import cogrob_face_msgs.msg as face_msgs
import cogrob_face_msgs.srv as face_srvs
import collections
import datetime
import greeting_bot_state_machine.robot_speak
from greeting_bot_state_machine import face_utils
from greeting_bot_state_machine import general_states
from greeting_bot_state_machine import proto_utils
from greeting_bot_state_machine import robot_utils
import grpc
import random
import rospy
import smach
import std_msgs.msg as std_msgs
import sys
import time
import threading

from cogrob_listen_intent import common_intent_listener

FLAGS = flags.FLAGS
SayText = greeting_bot_state_machine.robot_speak.SayText


class LookUpRecentSeenIdentityState(smach.State):
  _helper = None
  def __init__(self):
    if LookUpRecentSeenIdentityState._helper is None:
      LookUpRecentSeenIdentityState._helper = (
          face_utils.TestRecentSeenIdentityHelper())

    smach.State.__init__(
        self, outcomes=["known_person", "unknown_person", "error"],
        output_keys=["last_seen_human_uuid",
                     "last_seen_human_timestamp",
                     "last_seen_human_nicknames"])


  def execute(self, userdata):
    userdata.last_seen_human_uuid = None
    userdata.last_seen_human_timestamp = None
    userdata.last_seen_human_nicknames = None

    recent_seen_human = self._helper.GetMostVotedHuman()

    if recent_seen_human is None:
      return "error"

    if recent_seen_human.human_uuid is None:
      return "unknown_person"

    userdata.last_seen_human_uuid = recent_seen_human.human_uuid
    userdata.last_seen_human_timestamp = recent_seen_human.capture_time
    userdata.last_seen_human_nicknames = recent_seen_human.human_nicknames
    return "known_person"


class GreetPeopleState1(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def GetPartOfDay(self):
    currentTime = datetime.datetime.now()
    if currentTime.hour < 12:
      return 'morning'
    elif 12 <= currentTime.hour < 18:
      return 'afternoon'
    else:
      return 'evening'


  def execute(self, userdata):
    SayText("Good {}.".format(self.GetPartOfDay()))
    return "next"


class GreetPeopleState2(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    if robot_utils.GetRobotModel() == "freight":
      SayText("My name is BoxBot.")
    else:
      SayText("My name is TritonBot.")
    return "next"


class GreetPeopleState3(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("I am a robot working here at the Contextual Robotics Institute.")
    return "next"


class GreetKnownPeopleState(smach.State):
  def __init__(self):
    smach.State.__init__(
        self, outcomes=["next"], input_keys=["last_seen_human_nicknames"])


  def execute(self, userdata):
    if len(userdata.last_seen_human_nicknames) > 0:
      SayText("Nice to see you again, {}.".format(
          random.choice(userdata.last_seen_human_nicknames)))
      # TODO(shengye): Ask whether the name is correct if not so confident.
    else:
      SayText("My aplogise but I didn't remember your name. "
              "Nice to see you again.")
      # TODO(shengye): In this case, we should ask the human his/her name and
      # associate with HumanDB.
    return "next"


class AskForNameState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("May I ask what is your name?")
    return "next"


class ListenForHumanNameState(smach.State):
  def __init__(self):
    smach.State.__init__(
      self, outcomes=["got_name", "dont_understand", "silence"],
      input_keys=["human_name"], output_keys=["human_name"])


  def execute(self, userdata):
    # TODO(shengye): Also listen for yes/no, etc.
    listener = common_intent_listener.GetCommonIntentListener(
        ["human_name"], ignore_intrim_results=True)

    userdata.human_name = None
    result = listener.ListenForIntent()
    if result.intent.WhichOneof("intent") is None:
      return "dont_understand"
    elif result.intent.HasField("silence"):
      # TODO(shengye): Deal with Google Cloud failure (Internet failure, etc.)
      return "silence"
    elif result.intent.HasField("human_name"):
      userdata.human_name = result.intent.human_name.name
      assert userdata.human_name.strip() != ""
      return "got_name"
    else:
      raise NotImplementedError("Unknown intent: {}".format(str(result)))


class GreetHumanWithNameState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["human_name"])


  def execute(self, userdata):
    SayText("Nice to meet you, {}.".format(userdata.human_name))
    return "next"


class AskNameAgainForSilenceState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("I am sorry but I didn't hear you. What is your name?")
    return "next"


class AskNameAgainForDontUnderstandState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("I am sorry but I didn't understand. What is your name?")
    return "next"


class ApologizeErrorState(smach.State):
  def __init__(self, say_words=False):
    smach.State.__init__(self, outcomes=["next"])
    self._say_words = say_words


  def execute(self, userdata):
    if self._say_words:
      SayText("I am sorry but I am experiencing some difficulties. "
              "I will talk to you next time.")
    return "next"


class SayHaveGoodDayState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("Take care and have a great day.")
    return "next"


class SayOkayNoQuizState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["human_name"])


  def execute(self, userdata):
    if userdata.human_name:
      SayText("No worries, {}.".format(userdata.human_name))
    else:
      SayText("No worries.")
    return "next"


class SayOkayNoTourState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["human_name"])


  def execute(self, userdata):
    if userdata.human_name:
      SayText("Okay, no worries, {}.".format(userdata.human_name))
    else:
      SayText("Okay, no worries.")
    return "next"


def GetGreetPeopleWithFaceOrRememberFaceStateMachine():
  sm = smach.StateMachine(
      outcomes=["known_person", "unknown_person", "error"],
      output_keys=["human_name"])

  sm.userdata.last_seen_human_uuid = None
  sm.userdata.last_seen_human_timestamp = None
  sm.userdata.last_seen_human_nicknames = None
  sm.userdata.human_name = None

  with sm:
    smach.StateMachine.add(
        "greet_people_state1", GreetPeopleState1(),
        transitions={"next": "lookup_recent_seen_human_id_1"})

    smach.StateMachine.add(
        "lookup_recent_seen_human_id_1", LookUpRecentSeenIdentityState(),
        transitions={"known_person": "greet_known_people",
                     "unknown_person": "greet_people_state2",
                     "error": "apologize_error"},
        remapping={"last_seen_human_uuid": "last_seen_human_uuid",
                   "last_seen_human_timestamp": "last_seen_human_timestamp",
                   "last_seen_human_nicknames": "last_seen_human_nicknames"})
    sm.set_initial_state(["greet_people_state1"])

    smach.StateMachine.add(
        "greet_people_state2", GreetPeopleState2(),
        transitions={"next": "lookup_recent_seen_human_id_2"})

    smach.StateMachine.add(
        "lookup_recent_seen_human_id_2", LookUpRecentSeenIdentityState(),
        transitions={"known_person": "greet_known_people",
                     "unknown_person": "greet_people_state3",
                     "error": "apologize_error"},
        remapping={"last_seen_human_uuid": "last_seen_human_uuid",
                   "last_seen_human_timestamp": "last_seen_human_timestamp",
                   "last_seen_human_nicknames": "last_seen_human_nicknames"})

    smach.StateMachine.add(
        "greet_people_state3", GreetPeopleState3(),
        transitions={"next": "lookup_recent_seen_human_id_3"})

    smach.StateMachine.add(
        "lookup_recent_seen_human_id_3", LookUpRecentSeenIdentityState(),
        transitions={"known_person": "greet_known_people",
                     "unknown_person": "ask_for_name",
                     "error": "apologize_error"},
        remapping={"last_seen_human_uuid": "last_seen_human_uuid",
                   "last_seen_human_timestamp": "last_seen_human_timestamp",
                   "last_seen_human_nicknames": "last_seen_human_nicknames"})
    smach.StateMachine.add(
        "greet_known_people", GreetKnownPeopleState(),
        transitions={"next": "known_person"},
        remapping={"last_seen_human_nicknames": "last_seen_human_nicknames"})

    smach.StateMachine.add(
        "ask_for_name", AskForNameState(),
        transitions={"next": "reset_ask_name_retry"})

    smach.StateMachine.add(
        "reset_ask_name_retry",
        general_states.ResetRetryCounter("ask_name_retry", 2 - 1),
        transitions={"next": "listen_for_human_name"},
        remapping={"ask_name_retry": "ask_name_retry"})

    smach.StateMachine.add(
        "listen_for_human_name", ListenForHumanNameState(),
        transitions={
          "got_name": "greet_unknown_person_with_name",
          "dont_understand": "decrease_and_test_ask_name_retry_dont_understand",
          "silence": "decrease_and_test_ask_name_retry_silence"},
        remapping={"human_name": "human_name"})

    smach.StateMachine.add(
        "decrease_and_test_ask_name_retry_dont_understand",
        general_states.DecreaseAndTestRetry("ask_name_retry"),
        transitions={"continue": "ask_name_again_for_dont_understand",
                     "give_up": "apologize_error"},
        remapping={"ask_name_retry": "ask_name_retry"})

    smach.StateMachine.add(
        "decrease_and_test_ask_name_retry_silence",
        general_states.DecreaseAndTestRetry("ask_name_retry"),
        transitions={"continue": "ask_name_again_for_silence",
                     "give_up": "apologize_error"},
        remapping={"ask_name_retry": "ask_name_retry"})

    smach.StateMachine.add(
        "ask_name_again_for_dont_understand",
        AskNameAgainForDontUnderstandState(),
        transitions={"next": "listen_for_human_name"})

    smach.StateMachine.add(
        "ask_name_again_for_silence",
        AskNameAgainForSilenceState(),
        transitions={"next": "listen_for_human_name"})

    smach.StateMachine.add(
        "greet_unknown_person_with_name",
        GreetHumanWithNameState(),
        transitions={"next": "unknown_person"},
        remapping={"human_name": "human_name"})

    smach.StateMachine.add(
        "apologize_error", ApologizeErrorState(),
        transitions={"next": "error"})

  return sm


def main(argv):
  rospy.init_node("engage_people_states_test_node")
  FLAGS(argv)

  sm = GetGreetPeopleWithFaceOrRememberFaceStateMachine()

  outcome = sm.execute()
  rospy.loginfo("State machine finished with outcome: %s", outcome)


if __name__ == "__main__":
  main(sys.argv)
