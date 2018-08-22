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

import collections
import threading
import time

from cogrob.dialogue.speech_recognition import speech_interface

SpeechRecognizerInterface = speech_interface.SpeechRecognizerInterface
RecognitionResult = speech_interface.RecognitionResult


class MockSpeechRecognizer(SpeechRecognizerInterface):

  def __init__(self, utterances, need_intrim_result=True, sec_per_utterance=0):
    super(MockSpeechRecognizer, self).__init__()

    assert not isinstance(utterances, str)

    self._utterances = utterances
    self._sec_per_utterance = sec_per_utterance
    self._need_intrim_result = need_intrim_result

    self._stop_called = False
    self._thread = None
    self._lock = threading.Lock()


  def GeneratingThread(self, result_queue):
    utterances_queue = collections.deque(self._utterances)
    while len(utterances_queue):
      current_utterance = utterances_queue.popleft()
      words = current_utterance.split()
      return_parts = []
      i = 0

      if self._need_intrim_result:
        while i < len(words):
          return_parts.append(words[i])
          if i + 1 < len(words):
            return_parts.append("{} {}".format(words[i], words[i + 1]))
          if i + 2 < len(words):
            return_parts.append(
                "{} {} {}".format(words[i], words[i + 1], words[i + 2]))
            return_parts.append(
                "{} {}".format(words[i + 1], words[i + 2]))
          i += 2

      return_parts.append(current_utterance)

      interval = float(self._sec_per_utterance) / len(return_parts)

      for i in range(len(return_parts)):
        time.sleep(interval)

        if self._stop_called:
          break

        result = RecognitionResult(
            source_transcript = return_parts[i],
            source_confidence_score = 1.0,
            source_is_complete_utterance = (
                True if i == len(return_parts) - 1 else False),
            source_timestamp = time.time()
        )
        result_queue.put(result)

    result_queue.put(None)


  def StartImpl(self, result_queue):
    self._lock.acquire()
    self._stop_called = False
    self._thread = threading.Thread(target=self.GeneratingThread,
                                    args=(result_queue, ))
    self._thread.start()


  def StopImpl(self):
    self._stop_called = True
    self._thread.join()
    self._lock.release()
