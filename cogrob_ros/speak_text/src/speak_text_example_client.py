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

import speak_text_msgs.msg as speak_text_msgs

NIGHT_WATCH_VOW = (
    'Night gathers, and now my watch begins. It shall not end until my death. '
    'I shall take no wife, hold no lands, father no children. I shall wear no '
    'crowns and win no glory. I shall live and die at my post. I am the sword '
    'in the darkness. I am the watcher on the walls. I am the shield that '
    'guards the realms of men. I pledge my life and honor to the Night\'s '
    'Watch, for this night and all the nights to come.')

if __name__ == '__main__':
    rospy.init_node('speak_text_example_client', anonymous=True)
    client = actionlib.SimpleActionClient(
        'cogrob/speak_text_action', speak_text_msgs.SpeakTextAction)
    client.wait_for_server()
    goal = speak_text_msgs.SpeakTextGoal(text=NIGHT_WATCH_VOW)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo(str(client.get_result()))
