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

from greeting_bot_state_machine import arm_control_utils
import rospy
import smach
import sys


class SetArmToResetState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["success", "fail"])
    self._arm_controller = arm_control_utils.GetArmController()


  def execute(self, userdata):
    if self._arm_controller.SetPoseWithRetry(arm_control_utils.RESET_POSITION):
      return "success"
    return "fail"


class SetArmToTorsoUpState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["success", "fail"])
    self._arm_controller = arm_control_utils.GetArmController()


  def execute(self, userdata):
    if self._arm_controller.SetPoseWithRetry(
        arm_control_utils.TORSO_UP_POSITION):
      return "success"
    return "fail"


def main(argv):
  rospy.init_node("arm_control_states_test_node")

  sm = smach.StateMachine(outcomes=["success", "fail"])
  with sm:
    smach.StateMachine.add(
        "torso_up_pose", SetArmToTorsoUpState(),
        transitions={"success": "arm_reset_pose", "fail": "fail"})
    sm.set_initial_state(["torso_up_pose"])

    smach.StateMachine.add(
        "arm_reset_pose", SetArmToResetState(),
        transitions={"success": "success", "fail": "fail"})

  outcome = sm.execute()
  rospy.loginfo("move_head_states outcome: %s", outcome)


if __name__ == "__main__":
  main(sys.argv)
