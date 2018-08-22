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

from __future__ import print_function

import actionlib
import actionlib_msgs.msg as actionlib_msgs
import collections
import gcloud_speech_msgs.msg as gcloud_speech_msgs
import os
import rospy
import signal
import sys
import threading
import time

import use_cogrob_workspace
from cogrob.dialogue.speech_recognition import speech_interface

GoalStatus = actionlib_msgs.GoalStatus
SpeechRecognizerInterface = speech_interface.SpeechRecognizerInterface
RecognitionResult = speech_interface.RecognitionResult


class GcloudSpeechRecognizer(SpeechRecognizerInterface):

  def __init__(self, hints=None, max_alternatives=1, listen_duration_sec=15,
               max_recognition_duration_sec=17, suppress_interim_results=False):
    super(GcloudSpeechRecognizer, self).__init__()

    if hints is None:
      hints = []
    self._hints = hints
    self._max_alternatives = max_alternatives
    self._listen_duration_sec = listen_duration_sec
    self._max_recognition_duration_sec = max_recognition_duration_sec
    self._suppress_interim_results = suppress_interim_results

    self._stop_called = False
    self._thread = None
    self._lock = threading.Lock()

    self._gcloud_client = actionlib.SimpleActionClient(
      "/cogrob/speech_to_text", gcloud_speech_msgs.SpeechToTextAction)

    # TODO(shengye): set a deadline for wait_for_server, in case the actionlib
    # server is not responding, we could crash here.
    self._gcloud_client.wait_for_server()

    self._stop_called = False


  def ExecutionThread(self, result_queue):

    def DoneCallback(state, result):
      rospy.loginfo("\n\nDone, state {}, result:\n{}\n".format(state, result))


    def ActiveCallback():
      rospy.loginfo("GcloudSpeechRecognizer goal is now active.\n")


    def FeedbackCallback(feedback):
      rospy.loginfo("{}\n".format(feedback))
      for hypothesis in feedback.hypotheses:
        # Converts feedback to RecognitionResult and put into queue.
        result = RecognitionResult(
            source_transcript = hypothesis.transcript,
            source_confidence_score = hypothesis.confidence,
            source_is_complete_utterance = feedback.is_portion_final,
            source_timestamp = time.time()
        )
        result_queue.put(result)


    # It is our responsibility to force cancel the aciton and put None into
    # result_queue after self._max_recognition_duration_sec, even if the action
    # server fails to respond.
    start_time = time.time()
    deadline = start_time + self._max_recognition_duration_sec

    goal = gcloud_speech_msgs.SpeechToTextGoal()
    goal.hints = self._hints
    goal.max_alternatives = self._max_alternatives
    goal.listen_duration_sec = self._listen_duration_sec
    goal.max_recognition_duration_sec = self._max_recognition_duration_sec
    goal.suppress_interim_results = self._suppress_interim_results

    self._gcloud_client.send_goal(
        goal, done_cb=DoneCallback, active_cb=ActiveCallback,
        feedback_cb=FeedbackCallback)

    while (self._gcloud_client.get_state() in [GoalStatus.PENDING,
        GoalStatus.ACTIVE] and time.time() < deadline
        and not self._stop_called):
      # Wait for a while before polling so that we don't consume 100% CPU.
      # TODO(shengye): Set this time out via command line flag (absl.flags).
      time.sleep(0.2)

    # If we are out of the loop and the goal is still active, we need to force
    # cancel the goal.
    if self._gcloud_client.get_state() in [
        GoalStatus.PENDING, GoalStatus.ACTIVE]:
      self._gcloud_client.cancel_goal()

    if self._gcloud_client.wait_for_result(rospy.Duration.from_sec(0.5)):
      rospy.loginfo("Goal finished successfully, time {}s.".format(
          time.time() - start_time))
    else:
      rospy.logerr("Goal failed to finish, time {}s.".format(
          time.time() - start_time))

    result_queue.put(None)


  def StartImpl(self, result_queue):
    self._lock.acquire()
    self._stop_called = False
    self._thread = threading.Thread(target=self.ExecutionThread,
                                    args=(result_queue, ))
    self._thread.start()


  def StopImpl(self):
    self._stop_called = True
    self._thread.join()
    self._lock.release()
