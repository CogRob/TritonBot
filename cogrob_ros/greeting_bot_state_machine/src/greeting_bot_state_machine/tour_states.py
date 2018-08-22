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

import actionlib
import actionlib_msgs.msg
import cogrob_navigation_msgs.msg
import greeting_bot_state_machine.robot_speak
from greeting_bot_state_machine import general_states
from greeting_bot_state_machine import robot_pose_states
from greeting_bot_state_machine import location_of_map
import rospy
import smach
import smach_ros
import time

import use_cogrob_workspace
from cogrob.navigation.tour import tour_info

GoalStatus = actionlib_msgs.msg.GoalStatus
SayText = greeting_bot_state_machine.robot_speak.SayText

class LoadTourPointsList(smach.State):
  def __init__(self):
    smach.State.__init__(
        self, outcomes=["next"], output_keys=["remaining_tour_points"])


  def execute(self, userdata):
    userdata.remaining_tour_points = [
        "prototyping_lab", "robot_zoo", "smart_home", "elevator_entrance"]
    #
    # userdata.remaining_tour_points = [
    #     "upper_left_corner", "lower_right_corner", "upper_right_corner",
    #     "lower_left_corner"]
    return "next"
    # TODO(shengye): Check all of these keys has a NavPoint when loaded.


class GetNextTourPoint(smach.State):
  def __init__(self):
    smach.State.__init__(
        self, outcomes=["next", "empty"], input_keys=["remaining_tour_points"],
        output_keys=["tour_point_id", "remaining_tour_points"])


  def execute(self, userdata):
    if len(userdata.remaining_tour_points) > 0:
      userdata.tour_point_id = userdata.remaining_tour_points.pop(0)
      return "next"
    else:
      return "empty"


class GoToTourPointState(smach.State):
  def __init__(self, max_time=60):
    smach.State.__init__(
        self, outcomes=["done", "failed"], input_keys=["tour_point_id"])
    self._max_time = max_time

    if not hasattr(GoToTourPointState, "_move_client"):
      GoToTourPointState._move_client = actionlib.SimpleActionClient(
          "/cogrob/move_to_nav_point",
          cogrob_navigation_msgs.msg.MoveToNavPointAction)
      GoToTourPointState._move_client.wait_for_server()
      rospy.loginfo("MoveToNavPoint action ready.")


  def execute(self, userdata):
    tour_point = tour_info.GetAllTourPointsMap(
        location_of_map.GetLocationOfMap())[userdata.tour_point_id]
    goal = cogrob_navigation_msgs.msg.MoveToNavPointGoal()
    goal.target_nav_point = tour_point.nav_point
    self._move_client.send_goal(goal)
    start_time = time.time()
    while self._move_client.get_state() in [
        GoalStatus.PENDING, GoalStatus.ACTIVE]:
      time.sleep(0.1)
      if time.time() - start_time > self._max_time:
        self._move_client.cancel_goal()
        return "failed"

    # Get result and decide whether to return "failed" or "next"
    self._move_client.wait_for_result()
    result = self._move_client.get_result()
    if result.is_success:
      return "done"
    else:
      rospy.logerr(result.error_msg)
      return "failed"


class SayMoveAwayState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("I need some personal space to maneuver. "
            "Could you please step back?")
    return "next"


class SayTourWords(smach.State):
  def __init__(self, max_time=60):
    smach.State.__init__(self, outcomes=["next"], input_keys=["tour_point_id"])


  def execute(self, userdata):
    tour_point = tour_info.GetAllTourPointsMap(
        location_of_map.GetLocationOfMap())[userdata.tour_point_id]
    if not tour_point.intro_skip_short_name:
      SayText("We are now at {}.".format(tour_point.short_name))
    SayText(tour_point.intro_words)
    return "next"


class SayFollowMe(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("Glad to hear that. Please follow me.")
    return "next"


class SayThatsAll(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("Please enjoy your stay.")
    return "next"


class ApologizeErrorState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    SayText("I am sorry but I am experiencing some difficulties. "
            "I will serve you next time.")
    return "next"


def GetTourGuideStateMachine():
  sm = smach.StateMachine(outcomes=["finished", "failed"])
  with sm:
    smach.StateMachine.add(
        "say_follow_me", SayFollowMe(),
        transitions={"next": "prepare_to_move"})
    sm.set_initial_state(["say_follow_me"])

    smach.StateMachine.add(
        "prepare_to_move", robot_pose_states.GetPrepareToMoveStateMachine(),
        transitions={"success": "load_tour_points",
                     "fail": "apologize_error"})

    smach.StateMachine.add(
        "load_tour_points", LoadTourPointsList(),
        transitions={"next": "get_next_tour_point"},
        remapping={"remaining_tour_points": "remaining_tour_points"})

    smach.StateMachine.add(
        "get_next_tour_point", GetNextTourPoint(),
        transitions={"next": "reset_go_to_tour_point_retry",
                     "empty": "say_thats_all"},
        remapping={"tour_point_id": "tour_point_id",
                   "remaining_tour_points": "remaining_tour_points"})

    smach.StateMachine.add(
        "reset_go_to_tour_point_retry",
        general_states.ResetRetryCounter("go_to_tour_point_retry", 3 - 1),
        transitions={"next": "go_to_tour_point"},
        remapping={"go_to_tour_point_retry": "go_to_tour_point_retry"})

    smach.StateMachine.add(
        "go_to_tour_point", GoToTourPointState(),
        transitions={"done": "say_tour_words",
                     "failed": "decrease_and_test_go_to_tour_point"},
        remapping={"tour_point_id": "tour_point_id"})

    smach.StateMachine.add(
        "decrease_and_test_go_to_tour_point",
        general_states.DecreaseAndTestRetry("go_to_tour_point_retry"),
        transitions={"continue": "say_move_away",
                     "give_up": "apologize_error"},
        remapping={"go_to_tour_point_retry": "go_to_tour_point_retry"})

    smach.StateMachine.add(
        "say_move_away", SayMoveAwayState(),
        transitions={"next": "go_to_tour_point"})

    smach.StateMachine.add(
        "say_tour_words", SayTourWords(),
        transitions={"next": "get_next_tour_point"},
        remapping={"tour_point_id": "tour_point_id"})

    smach.StateMachine.add(
        "say_thats_all", SayThatsAll(),
        transitions={"next": "finished"})

    smach.StateMachine.add(
        "apologize_error", ApologizeErrorState(),
        transitions={"next": "failed"})

  return sm
