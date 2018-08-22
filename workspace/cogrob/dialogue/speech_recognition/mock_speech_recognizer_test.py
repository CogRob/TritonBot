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

from absl import logging
import collections
import time
import unittest

logging.set_verbosity(logging.INFO)

from cogrob.dialogue.speech_recognition import mock_speech_recognizer
from cogrob.dialogue.speech_recognition import speech_interface

MockSpeechRecognizer = mock_speech_recognizer.MockSpeechRecognizer
RecognitionResult = speech_interface.RecognitionResult


class TestIntents(unittest.TestCase):
  def testTiming(self):
    has_intrim = False
    has_final = False

    recognizer = MockSpeechRecognizer(
        ["Please take a selfie with me."], need_intrim_result = True,
        sec_per_utterance = 1)

    with recognizer as result_queue:
      start_time = time.time()
      while True:
        current = result_queue.get()
        logging.info("Time %.2f, received: %s",
                     time.time() - start_time, str(current))
        if current is None:
          break
        if current.source_is_complete_utterance == False:
          has_intrim = True
        if current.source_is_complete_utterance == True:
          has_final = True

    elapsed_time = time.time() - start_time
    self.assertGreater(elapsed_time, 0.9)
    self.assertLess(elapsed_time, 1.1)
    self.assertTrue(has_intrim)
    self.assertTrue(has_final)


  def testFast(self):
    has_intrim = False
    has_final = False

    recognizer = MockSpeechRecognizer(
        ["Please take a selfie with me."], need_intrim_result = True,
        sec_per_utterance = 0)

    with recognizer as result_queue:
      start_time = time.time()
      while True:
        current = result_queue.get()
        logging.info("Time %.2f, received: %s",
                     time.time() - start_time, str(current))
        if current is None:
          break
        if current.source_is_complete_utterance == False:
          has_intrim = True
        if current.source_is_complete_utterance == True:
          has_final = True

    elapsed_time = time.time() - start_time
    self.assertLess(elapsed_time, 0.1)
    self.assertTrue(has_intrim)
    self.assertTrue(has_final)


  def testMultiple(self):
    has_intrim = False
    final_count = 0

    recognizer = MockSpeechRecognizer(
        ["Please take a selfie with me.", "Please wave at the guests."],
        need_intrim_result = True, sec_per_utterance = 0)

    with recognizer as result_queue:
      start_time = time.time()
      while True:
        current = result_queue.get()
        logging.info("Time %.2f, received: %s",
                     time.time() - start_time, str(current))
        if current is None:
          break
        if current.source_is_complete_utterance == False:
          has_intrim = True
        if current.source_is_complete_utterance == True:
          final_count += 1

    elapsed_time = time.time() - start_time
    self.assertLess(elapsed_time, 0.1)
    self.assertTrue(has_intrim)
    self.assertEqual(final_count, 2)


  def testNoIntrim(self):
    has_intrim = False
    has_final = False

    recognizer = MockSpeechRecognizer(
        ["Please take a selfie with me."], need_intrim_result = False,
        sec_per_utterance = 0)

    with recognizer as result_queue:
      start_time = time.time()
      while True:
        current = result_queue.get()
        logging.info("Time %.2f, received: %s",
                     time.time() - start_time, str(current))
        if current is None:
          break
        if current.source_is_complete_utterance == False:
          has_intrim = True
        if current.source_is_complete_utterance == True:
          has_final = True

    elapsed_time = time.time() - start_time
    self.assertLess(elapsed_time, 0.1)
    self.assertFalse(has_intrim)
    self.assertTrue(has_final)


if __name__ == '__main__':
  unittest.main()
