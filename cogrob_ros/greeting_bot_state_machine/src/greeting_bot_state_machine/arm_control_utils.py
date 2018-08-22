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

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

RESET_POSITION = {
  'torso_lift_joint':   0.05287206172943115,
  'shoulder_pan_joint':   1.3197248284439087,
  'shoulder_lift_joint':  1.3999865214523315,
  'upperarm_roll_joint':  -0.19936644117534638,
  'elbow_flex_joint':   1.719395410321045,
  'forearm_roll_joint':   -0.0010107747018098835,
  'wrist_flex_joint':   1.659553606298828,
  'wrist_roll_joint':   -0.00010822674834728291,
}

TORSO_UP_POSITION = {
  'torso_lift_joint':   0.3742319345474243,
  'shoulder_pan_joint':   1.3197248284439087,
  'shoulder_lift_joint':  1.3999865214523315,
  'upperarm_roll_joint':  -0.19936644117534638,
  'elbow_flex_joint':   1.719395410321045,
  'forearm_roll_joint':   -0.0010107747018098835,
  'wrist_flex_joint':   1.659553606298828,
  'wrist_roll_joint':   -0.00010822674834728291,
}

class ArmController(object):
  def __init__(self):
    # Create move group interface for a fetch robot
    self._move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    self._planning_scene = PlanningSceneInterface("base_link")
    self._planning_scene.removeCollisionObject("my_front_ground")
    self._planning_scene.removeCollisionObject("my_back_ground")
    self._planning_scene.removeCollisionObject("my_right_ground")
    self._planning_scene.removeCollisionObject("my_left_ground")
    self._planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    self._planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    self._planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    self._planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

  def SetPose(self, target_pose):
    # Plans the joints in joint_names to angles in pose
    self._move_group.moveToJointPosition(
        target_pose.keys(), target_pose.values(), wait=False)

    # Since we passed in wait=False above we need to wait here
    self._move_group.get_move_action().wait_for_result()
    result = self._move_group.get_move_action().get_result()

    return_result = False

    if result:
      # Checking the MoveItErrorCode
      if result.error_code.val == MoveItErrorCodes.SUCCESS:
        rospy.loginfo("MoveIt pose reached.")
        return_result = True
      else:
        # If you get to this point please search for:
        # moveit_msgs/MoveItErrorCodes.msg
        rospy.logerr("Arm goal in state: %s",
                     self._move_group.get_move_action().get_state())
    else:
      rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    self._move_group.get_move_action().cancel_all_goals()

    return return_result


  def SetPoseWithRetry(self, target_pose, max_time=3):
    for _ in range(max_time):
      if self.SetPose(target_pose):
        return True
    return False


def GetArmController():
  if not hasattr(GetArmController, "_arm_controller"):
    GetArmController._arm_controller = ArmController()
  return GetArmController._arm_controller


def main():
  rospy.init_node("arm_control_utils_test_node")

  con = GetArmController()
  rospy.loginfo("Ready!")

  con.SetPoseWithRetry(TORSO_UP_POSITION)
  con.SetPoseWithRetry(RESET_POSITION)

if __name__ == '__main__':
  main()
