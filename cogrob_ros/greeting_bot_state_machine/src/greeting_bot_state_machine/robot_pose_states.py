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
import rospy
import smach
import sys
import time

from greeting_bot_state_machine import arm_control_states
from greeting_bot_state_machine import move_head_states
from greeting_bot_state_machine import robot_speak
from greeting_bot_state_machine import robot_utils

GoalStatus = actionlib_msgs.msg.GoalStatus
SayText = robot_speak.SayText


class BypassState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    return "next"


def _GetBypassStateMachine():
  sm = smach.StateMachine(outcomes=["success", "fail"])
  with sm:
    smach.StateMachine.add(
        "bypass", BypassState(),
        transitions={"next": "success"})
    sm.set_initial_state(["bypass"])
  return sm


def GetPrepareToServeStateMachine():
  if robot_utils.GetRobotModel() == "freight":
    return _GetBypassStateMachine()

  sm = smach.StateMachine(outcomes=["success", "fail"])

  with sm:
    smach.StateMachine.add(
        "torso_up_pose", arm_control_states.SetArmToTorsoUpState(),
        transitions={"success": "move_head_and_success",
                     "fail": "move_head_and_fail"})
    sm.set_initial_state(["torso_up_pose"])

    smach.StateMachine.add(
        "move_head_and_success", move_head_states.MoveHeadState(),
        transitions={"success": "success", "fail": "fail"})

    smach.StateMachine.add(
        "move_head_and_fail", move_head_states.MoveHeadState(),
        transitions={"success": "fail", "fail": "fail"})

  return sm


def GetPrepareToMoveStateMachine():
  if robot_utils.GetRobotModel() == "freight":
    return _GetBypassStateMachine()

  sm = smach.StateMachine(outcomes=["success", "fail"])

  with sm:
    smach.StateMachine.add(
        "arm_reset_pose", arm_control_states.SetArmToResetState(),
        transitions={"success": "success", "fail": "fail"})
    sm.set_initial_state(["arm_reset_pose"])

  return sm


class GoToReadyPositionState(smach.State):
  def __init__(self, max_time=180, nav_point="WNZ"):
    smach.State.__init__(
        self, outcomes=["done", "failed"])
    self._max_time = max_time
    self._nav_point = nav_point

    if not hasattr(GoToReadyPositionState, "_move_client"):
      GoToReadyPositionState._move_client = actionlib.SimpleActionClient(
          "/cogrob/move_to_nav_point",
          cogrob_navigation_msgs.msg.MoveToNavPointAction)
      GoToReadyPositionState._move_client.wait_for_server()
      rospy.loginfo("GoToReadyPositionState MoveToNavPoint action ready.")


  def execute(self, userdata):
    goal = cogrob_navigation_msgs.msg.MoveToNavPointGoal()
    goal.target_nav_point = self._nav_point
    # TODO(shengye): Specify orientation for the NavPoints
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


def GetGoToReadyPositionStateAndPrepareStateMachine():
  sm = smach.StateMachine(outcomes=["success", "fail"])

  with sm:
    smach.StateMachine.add(
        "prepare_to_move", GetPrepareToMoveStateMachine(),
        transitions={"success": "go_to_ready_position_stage_1",
                     "fail": "fail"})
    sm.set_initial_state(["prepare_to_move"])

    smach.StateMachine.add(
        "go_to_ready_position_stage_1",
        GoToReadyPositionState(nav_point="HSV"),
        transitions={"done": "go_to_ready_position_stage_2",
                     "failed": "prepare_to_serve_fail"})

    smach.StateMachine.add(
        "go_to_ready_position_stage_2",
        GoToReadyPositionState(nav_point="WNZ"),
        transitions={"done": "prepare_to_serve",
                     "failed": "prepare_to_serve_fail"})

    smach.StateMachine.add(
        "prepare_to_serve", GetPrepareToServeStateMachine(),
        transitions={"success": "success", "fail": "fail"})

    smach.StateMachine.add(
        "prepare_to_serve_fail", GetPrepareToServeStateMachine(),
        transitions={"success": "fail", "fail": "fail"})

  return sm


def main(argv):
  rospy.init_node("robot_pose_states_test_node")

  # sm = GetPrepareToServeStateMachine()
  # outcome = sm.execute()
  # rospy.loginfo("GetPrepareToServeStateMachine outcome: %s", outcome)

  # sm = GetPrepareToMoveStateMachine()
  # outcome = sm.execute()
  # rospy.loginfo("GetPrepareToMoveStateMachine outcome: %s", outcome)

  # TODO(shengye): Use a flag to choose this.
  sm = GetGoToReadyPositionStateAndPrepareStateMachine()
  outcome = sm.execute()
  rospy.loginfo("GetGoToReadyPositionStateAndPrepareStateMachine outcome: %s",
                outcome)


if __name__ == "__main__":
  main(sys.argv)
