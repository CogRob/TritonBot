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
import control_msgs.msg
import geometry_msgs.msg
import rospy
import smach
import sys

PointHeadAction = control_msgs.msg.PointHeadAction
PointHeadGoal = control_msgs.msg.PointHeadGoal
PointStamped = geometry_msgs.msg.PointStamped


class MoveHeadState(smach.State):
  def __init__(self, x=1.0, y=0.0, z=0.7):
    smach.State.__init__(self, outcomes=["success", "fail"])
    self._x = x
    self._y = y
    self._z = z

    if not hasattr(MoveHeadState, "_point_head_client"):
      MoveHeadState._point_head_client = actionlib.SimpleActionClient(
          "head_controller/point_head", PointHeadAction)
      MoveHeadState._point_head_client.wait_for_server()
      rospy.loginfo("MoveHeadState point_head action ready.")


  def execute(self, userdata):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "torso_lift_link"
    point.point.x = self._x
    point.point.y = self._y
    point.point.z = self._z

    goal = PointHeadGoal()
    goal.target = point
    goal.max_velocity = 0.5

    self._point_head_client.send_goal(goal)
    self._point_head_client.wait_for_result()
    return "success"


def main(argv):
  rospy.init_node("move_head_states_test_node")

  sm = smach.StateMachine(outcomes=["success", "fail"])
  with sm:
    smach.StateMachine.add(
        "move_head", MoveHeadState(),
        transitions={"success": "success", "fail": "fail"})
    sm.set_initial_state(["move_head"])

  outcome = sm.execute()
  rospy.loginfo("move_head_states outcome: %s", outcome)


if __name__ == "__main__":
  main(sys.argv)
