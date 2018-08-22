#! /usr/bin/env python
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
import actionlib
import std_msgs.msg as std_msgs

import speak_text_msgs.msg as speak_text_msgs

class SpeakTextTopicHandler(object):

    def __init__(self):
        self._action_client = actionlib.SimpleActionClient(
            'cogrob/speak_text_action', speak_text_msgs.SpeakTextAction)
        self._action_client.wait_for_server()

        self._subscriber = rospy.Subscriber(
                "cogrob/speak_text", std_msgs.String, self._callback,
                queue_size=1)

    def _callback(self, data):
        rospy.loginfo(
                "{} wants to say {}".format(rospy.get_caller_id(), data.data))

        self._action_client.send_goal(
            speak_text_msgs.SpeakTextGoal(text=data.data))

        self._action_client.wait_for_result()
        rospy.loginfo(str(self._action_client.get_result()))

if __name__ == '__main__':
    rospy.init_node('speak_text_topic_node')
    topic_handler = SpeakTextTopicHandler()
    rospy.spin()
