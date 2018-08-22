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
import cogrob_robot_state_msgs.srv
from greeting_bot_state_machine import general_states
from greeting_bot_state_machine import wait_trigger_states
import rospy
import smach
import smach_ros
import sys
import time

GoalStatus = actionlib_msgs.msg.GoalStatus

class TestBatteryLevelState(smach.State):
  def __init__(self):
    smach.State.__init__(
        self, outcomes=["full", "normal", "low", "unknown"])
    if not hasattr(TestBatteryLevelState, "_get_battery_status"):
      GET_BATTERY_STATUS_SRV_NAME = "/cogrob/get_battery_status"
      rospy.wait_for_service(GET_BATTERY_STATUS_SRV_NAME)
      TestBatteryLevelState._get_battery_status = rospy.ServiceProxy(
          GET_BATTERY_STATUS_SRV_NAME,
          cogrob_robot_state_msgs.srv.GetBatteryStatus)
    self._FULL_LEVEL = 0.95
    self._NORMAL_LEVEL = 0.2


  def execute(self, userdata):
    try:
      battery_result = self._get_battery_status()
      if battery_result.charge_level > self._FULL_LEVEL:
        return "full"
      elif battery_result.charge_level > self._NORMAL_LEVEL:
        return "normal"
      else:
        return "low"
    except rospy.ServiceException as e:
      rospy.logerr(str(e))
      return "unknown"


class TestBatteryIsChargingState(smach.State):
  def __init__(self):
    smach.State.__init__(
        self, outcomes=["charging", "not_charging", "unknown"])
    if not hasattr(TestBatteryIsChargingState, "_get_battery_status"):
      GET_BATTERY_STATUS_SRV_NAME = "/cogrob/get_battery_status"
      rospy.wait_for_service(GET_BATTERY_STATUS_SRV_NAME)
      TestBatteryIsChargingState._get_battery_status = rospy.ServiceProxy(
          GET_BATTERY_STATUS_SRV_NAME,
          cogrob_robot_state_msgs.srv.GetBatteryStatus)


  def execute(self, userdata):
    try:
      battery_result = self._get_battery_status()
      if battery_result.is_charging:
        return "charging"
      else:
        return "not_charging"
    except rospy.ServiceException as e:
      rospy.logerr(str(e))
      return "unknown"


class GoDockingState(smach.State):
  def __init__(self, max_time=180):
    smach.State.__init__(
        self, outcomes=["done", "failed"])
    self._max_time = max_time

    if not hasattr(GoDockingState, "_move_client"):
      GoDockingState._move_client = actionlib.SimpleActionClient(
          "/cogrob/move_to_nav_point",
          cogrob_navigation_msgs.msg.MoveToNavPointAction)
      GoDockingState._move_client.wait_for_server()
      rospy.loginfo("MoveToNavPoint action ready.")


  def execute(self, userdata):
    goal = cogrob_navigation_msgs.msg.MoveToNavPointGoal()
    goal.target_nav_point = "DOCK"  # Magic NavPoint that docks the robot.
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


def GetAutoChargingStateMachine():
  sm = smach.StateMachine(
      outcomes=["no_need_charge", "finished_charging",
                "already_charging", "interrupt", "failed"])
  with sm:
    smach.StateMachine.add(
        "check_trigger_cmd",
        wait_trigger_states.TriggerCmdState(["prep_serve"]),
        transitions={"prep_serve": "interrupt",
                     "none": "test_already_charging"})
    sm.set_initial_state(["check_trigger_cmd"])

    smach.StateMachine.add(
      "test_already_charging", TestBatteryIsChargingState(),
      transitions={
          "charging": "already_charging",
          "not_charging": "decide_need_charging",
          "unknown": "failed",
      })

    smach.StateMachine.add(
      "decide_need_charging", TestBatteryLevelState(),
      transitions={
          "full": "no_need_charge",
          "normal": "no_need_charge",
          "low": "go_dock_retry_reset",
          "unknown": "failed",
      })

    smach.StateMachine.add(
      "go_dock_retry_reset",
      general_states.ResetRetryCounter("go_dock_retry", 3),
      transitions={"next": "go_dock_retry_test"},
      remapping={"go_dock_retry": "go_dock_retry"})

    smach.StateMachine.add(
      "go_dock_retry_test",
      general_states.DecreaseAndTestRetry("go_dock_retry"),
      transitions={
        "continue": "go_dock",
        "give_up": "failed"},
      remapping={"go_dock_retry": "go_dock_retry"})

    smach.StateMachine.add(
      "go_dock", GoDockingState(),
      transitions={"done": "test_is_charging",
                   "failed": "go_dock_retry_test"})

    smach.StateMachine.add(
      "test_is_charging", TestBatteryIsChargingState(),
      transitions={"charging": "test_finished_charging",
                   "not_charging": "go_dock_retry_test",
                   "unknown": "failed"},
      remapping={"tour_point_id": "tour_point_id"})

    smach.StateMachine.add(
      "test_finished_charging", TestBatteryLevelState(),
      transitions={"full": "finished_charging",
                   "normal": "wait_finish_charging",
                   "low": "wait_finish_charging",
                   "unknown": "failed"},
      remapping={"tour_point_id": "tour_point_id"})

    smach.StateMachine.add(
      "wait_finish_charging", general_states.WaitTimeState(5),
      transitions={"next": "test_finished_charging"})

  return sm


def main(argv):
  rospy.init_node("auto_charge_states_test_node")

  sm = GetAutoChargingStateMachine()

  smach_introspection_server = smach_ros.IntrospectionServer(
      "auto_charge_state_machine_test", sm, "/SM_ROOT")
  smach_introspection_server.start()

  while not rospy.is_shutdown():
    outcome = sm.execute()
    rospy.loginfo("State machine finished with outcome: %s", outcome)
    time.sleep(1)

  # Prevent ROS from crashing at shutdown.
  # https://github.com/ros/ros_comm/issues/527
  smach_introspection_server.stop()
  time.sleep(.1)


if __name__ == "__main__":
  main(sys.argv)
