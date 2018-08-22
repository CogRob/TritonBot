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

from absl import flags
import sys
import time
import unittest

from cogrob.dialogue.intent_extractors import action_extractor
from cogrob.dialogue.intent_extractors import name_extractor
from cogrob.dialogue.intent_selector import timed_best_intent_selector
from cogrob.dialogue.listen_for_intent import intent_listener
from cogrob.dialogue.speech_recognition import mock_speech_recognizer

MockSpeechRecognizer = mock_speech_recognizer.MockSpeechRecognizer
TimedBestIntentSelector = timed_best_intent_selector.TimedBestIntentSelector
IntentListener = intent_listener.IntentListener


class TestIntentListener(unittest.TestCase):

  def setUp(self):
    self._action_extractor = action_extractor.ActionExtractor()
    self._name_extractor = name_extractor.NameExtractor()


  def testGeneral(self):
    recognizer = MockSpeechRecognizer(
        ["Please take a selfie with me.", "My name is Jack."],
        need_intrim_result = True, sec_per_utterance = 0.5)

    selector = TimedBestIntentSelector(
        extractors = [self._action_extractor, self._name_extractor],
        intent_types = ["want_selfie"],
        init_wait_time = 0.6);

    listener = IntentListener(recognizer, selector)

    start_time = time.time()
    result = listener.ListenForIntent()
    elapsed_time = time.time() - start_time

    self.assertTrue(result is not None)
    self.assertTrue(result.intent.HasField("want_selfie"))

    self.assertGreater(elapsed_time, 0.6)

    # 0.6 + 0.1 (polling gap) + 0.05 (safety margin)
    self.assertLess(elapsed_time, 0.75)


  def testAfterInitDeadline(self):
    recognizer = MockSpeechRecognizer(
        ["Please take a selfie with me.", "My name is Jack."],
        need_intrim_result = True, sec_per_utterance = 0.5)

    selector = TimedBestIntentSelector(
        extractors = [self._action_extractor, self._name_extractor],
        intent_types = ["want_selfie"],
        init_wait_time = 0.4);

    listener = IntentListener(recognizer, selector)

    start_time = time.time()
    result = listener.ListenForIntent()
    elapsed_time = time.time() - start_time

    self.assertTrue(result is not None)
    self.assertTrue(result.intent.HasField("want_selfie"))

    self.assertGreater(elapsed_time, 0.5)
    # 0.5 + 0.1 (polling gap) + 0.05 (safty margin)
    self.assertLess(elapsed_time, 0.65)


  def testEarlyFinish(self):
    recognizer = MockSpeechRecognizer(
        ["Please take a selfie with me.", "My name is Jack."],
        need_intrim_result = True, sec_per_utterance = 0.5)

    selector = TimedBestIntentSelector(
        extractors = [self._action_extractor, self._name_extractor],
        intent_types = ["want_selfie"],
        init_wait_time = 2);

    listener = IntentListener(recognizer, selector)

    start_time = time.time()
    result = listener.ListenForIntent()
    elapsed_time = time.time() - start_time

    self.assertTrue(result is not None)
    self.assertTrue(result.intent.HasField("want_selfie"))

    self.assertGreater(elapsed_time, 1.0)
    # 1 + 0.1 (polling gap) + 0.05 (safty margin)
    self.assertLess(elapsed_time, 1.15)


if __name__ == '__main__':
  flags.FLAGS(sys.argv)
  unittest.main()
