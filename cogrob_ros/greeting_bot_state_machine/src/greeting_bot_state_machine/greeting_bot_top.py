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

import smach
import smach_ros

from greeting_bot_state_machine import ask_want_quiz_states
from greeting_bot_state_machine import ask_want_tour_states
from greeting_bot_state_machine import auto_charge_states
from greeting_bot_state_machine import cogrob_rosbag_states
from greeting_bot_state_machine import engage_people_states
from greeting_bot_state_machine import face_states
from greeting_bot_state_machine import general_states
from greeting_bot_state_machine import quiz_states
from greeting_bot_state_machine import robot_pose_states
from greeting_bot_state_machine import tour_states
from greeting_bot_state_machine import trivia_mode
from greeting_bot_state_machine import wait_trigger_states


class ResetSessionData(smach.State):
  def __init__(self):
    smach.State.__init__(
        self, outcomes=["next"], output_keys=["should_train", "human_name"])


  def execute(self, userdata):
    userdata.should_train = False
    userdata.human_name = None
    return "next"


class SetShouldTrainState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], output_keys=["should_train"])


  def execute(self, userdata):
    userdata.should_train = True
    return "next"


class TestShouldTrainState(smach.State):
  def __init__(self):
    smach.State.__init__(
        self, outcomes=["affirmative", "negative"], input_keys=["should_train"])


  def execute(self, userdata):
    if userdata.should_train:
      return "affirmative"
    else:
      return "negative"


def GetGreetingBotTopStateMachine():
  sm = smach.StateMachine(outcomes=["finished", "error"])
  sm.userdata.human_name = None
  sm.userdata.should_train = False

  with sm:
    smach.StateMachine.add(
        "finish_rosbag_cleanup",
        cogrob_rosbag_states.FinishRecording(),
        transitions={"next":"test_charging"})
    sm.set_initial_state(["finish_rosbag_cleanup"])

    test_charging_sm = auto_charge_states.GetAutoChargingStateMachine()
    smach.StateMachine.add(
        "test_charging", test_charging_sm,
        transitions={"no_need_charge": "wait_for_trigger",
                     "finished_charging": "go_prepare",
                     "already_charging": "already_charging_delay",
                     "interrupt": "wait_for_trigger",
                     "failed": "error"})

    smach.StateMachine.add(
        "already_charging_delay", general_states.WaitTimeState(1),
        transitions={"next": "finish_rosbag_cleanup"})

    go_prepare_sm = (
        robot_pose_states.GetGoToReadyPositionStateAndPrepareStateMachine())
    smach.StateMachine.add(
        "go_prepare", go_prepare_sm,
        transitions={"success": "wait_for_trigger",
                     "fail": "wait_for_trigger"})

    wait_for_trigger_sm = (
        wait_trigger_states.GetWaitForTriggerMachine())
    smach.StateMachine.add(
        "wait_for_trigger", wait_for_trigger_sm,
        transitions={"wakeword": "start_rosbag_recording_wakeword",
                     "face": "start_rosbag_recording_face",
                     "man_go_prep": "go_prepare",
                     "man_go_charge": "go_charge_1",
                     "nothing": "delay_restart"})

    smach.StateMachine.add(
        "go_charge_1", robot_pose_states.GetPrepareToMoveStateMachine(),
        transitions={"success": "go_charge_2", "fail": "finish_rosbag_cleanup"})

    smach.StateMachine.add(
        "go_charge_2", auto_charge_states.GoDockingState(),
        transitions={"done": "finish_rosbag_cleanup",
                     "failed": "finish_rosbag_cleanup"})

    smach.StateMachine.add(
        "delay_restart", general_states.WaitTimeState(.1),
        transitions={"next": "finish_rosbag_cleanup"})

    smach.StateMachine.add(
        "start_rosbag_recording_face", cogrob_rosbag_states.StartRecording(),
        transitions={"next":"reset_session_data_face"})

    smach.StateMachine.add(
        "start_rosbag_recording_wakeword",
        cogrob_rosbag_states.StartRecording(),
        transitions={"next":"reset_session_data_wakeword"})

    smach.StateMachine.add(
        "reset_session_data_face", ResetSessionData(),
        transitions={"next":"greet_people_with_face"},
        remapping={"should_train":"should_train", "human_name":"human_name"})

    smach.StateMachine.add(
        "reset_session_data_wakeword", ResetSessionData(),
        transitions={"next":"ask_want_tour"},
        remapping={"should_train":"should_train", "human_name":"human_name"})

    greet_people_with_face_sm = (
        engage_people_states.GetGreetPeopleWithFaceOrRememberFaceStateMachine())
    if trivia_mode.TRIVIA_MODE:
      smach.StateMachine.add(
          "greet_people_with_face", greet_people_with_face_sm,
          transitions={"known_person": "ask_want_quiz",
                       "unknown_person": "set_should_train",
                       "error": "finish_rosbag_recording_error"},
          remapping={"human_name":"human_name"})
    else:
      smach.StateMachine.add(
          "greet_people_with_face", greet_people_with_face_sm,
          transitions={"known_person": "ask_want_tour",
                       "unknown_person": "set_should_train",
                       "error": "finish_rosbag_recording_error"},
          remapping={"human_name":"human_name"})

    smach.StateMachine.add(
        "set_should_train", SetShouldTrainState(),
        transitions={"next":"ask_want_quiz"},
        remapping={"should_train":"should_train"})

    ask_want_quiz_sm = ask_want_quiz_states.GetAskWantQuizStateMachine()
    smach.StateMachine.add(
        "ask_want_quiz", ask_want_quiz_sm,
        transitions={"want_quiz": "play_quiz",
                     "dont_want_quiz": "ask_want_tour",
                     "silence": "finish_rosbag_recording_error"})

    quiz_sm = quiz_states.GetQuizStateMachine()
    smach.StateMachine.add(
        "play_quiz", quiz_sm,
        transitions={"finished": "test_should_train",
                     "silence": "finish_rosbag_recording_error"})

    smach.StateMachine.add(
        "test_should_train", TestShouldTrainState(),
        transitions={
          "affirmative": "train_on_face_with_name",
          "negative": "ask_want_tour"},
        remapping={"should_train": "should_train"})

    smach.StateMachine.add(
        "train_on_face_with_name",
        face_states.TrainOnFaceWithNameState(),
        transitions={"next": "ask_want_tour"},
        remapping={"human_name": "human_name"})

    if trivia_mode.TRIVIA_MODE:
      smach.StateMachine.add(
          "ask_want_tour", general_states.BypassState(),
          transitions={"next": "say_have_good_day"})
    else:
      ask_want_tour_sm = ask_want_tour_states.GetAskWantTourStateMachine()
      smach.StateMachine.add(
          "ask_want_tour", ask_want_tour_sm,
          transitions={"want_tour": "guide_tour",
                       "dont_want_tour": "say_have_good_day",
                       "silence": "finish_rosbag_recording_error"})

    tour_sm = tour_states.GetTourGuideStateMachine()
    smach.StateMachine.add(
        "guide_tour", tour_sm,
        transitions={"finished": "say_have_good_day_tour",
                     "failed": "finish_rosbag_recording_before_go_prepare"})

    smach.StateMachine.add(
        "say_have_good_day_tour", engage_people_states.SayHaveGoodDayState(),
        transitions={"next": "finish_rosbag_recording_before_go_prepare"})

    smach.StateMachine.add(
        "finish_rosbag_recording_before_go_prepare",
        cogrob_rosbag_states.FinishRecording(),
        transitions={"next":"go_prepare"})

    smach.StateMachine.add(
        "say_have_good_day", engage_people_states.SayHaveGoodDayState(),
        transitions={"next": "finish_rosbag_recording_finished"})

    smach.StateMachine.add(
        "finish_rosbag_recording_finished",
        cogrob_rosbag_states.FinishRecording(), transitions={"next":"finished"})

    smach.StateMachine.add(
        "finish_rosbag_recording_error",
        cogrob_rosbag_states.FinishRecording(), transitions={"next":"error"})

  return sm
