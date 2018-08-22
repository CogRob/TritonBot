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

import cogrob_rosbag_msgs.srv
import rospy
import smach

class StartRecording(smach.State):

  def __init__(self, config_name="greeting_bot", restart=True):
    smach.State.__init__(self, outcomes=["next"])
    self._restart = restart
    self._config_name = config_name
    START_RECORDING_SRV_PATH = "/cogrob/rosbag/start_recording"
    rospy.wait_for_service(START_RECORDING_SRV_PATH)
    self._StartRecording = rospy.ServiceProxy(
        START_RECORDING_SRV_PATH, cogrob_rosbag_msgs.srv.StartRecording)


  def execute(self, userdata):
    response = self._StartRecording(
        config_name=self._config_name, restart=self._restart)
    rospy.loginfo("Start recording: {}".format(str(response)))
    return "next"


class FinishRecording(smach.State):

  def __init__(self, config_name="greeting_bot"):
    smach.State.__init__(self, outcomes=["next"])
    self._config_name = config_name
    FINISH_RECORDING_SRV_PATH = "/cogrob/rosbag/finish_recording"
    rospy.wait_for_service(FINISH_RECORDING_SRV_PATH)
    self._FinishRecording = rospy.ServiceProxy(
        FINISH_RECORDING_SRV_PATH, cogrob_rosbag_msgs.srv.FinishRecording)


  def execute(self, userdata):
    response = self._FinishRecording(config_name=self._config_name)
    rospy.loginfo("Finish recording: {}".format(str(response)))
    return "next"
