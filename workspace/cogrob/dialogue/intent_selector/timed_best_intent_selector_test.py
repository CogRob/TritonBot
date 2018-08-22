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
from absl import logging
import Queue
import sys
import threading
import time
import unittest

from cogrob.dialogue.intent_extractors import number_extractor
from cogrob.dialogue.intent_extractors import name_extractor
from cogrob.dialogue.intent_extractors import combined_extractor
from cogrob.dialogue.intent_selector import intent_selector_base
from cogrob.dialogue.intent_selector import timed_best_intent_selector

FLAGS = flags.FLAGS

RecognitionResult = intent_selector_base.RecognitionResult
TimedBestIntentSelector = timed_best_intent_selector.TimedBestIntentSelector

class TestTimedBestIntentSelector(unittest.TestCase):
  def setUp(self):
    FLAGS(sys.argv)
    self._number_extractor = number_extractor.NumberExtractor()
    self._name_extractor = name_extractor.NameExtractor()
    self._combined_extractor = combined_extractor.CombinedExtractor(
        self._number_extractor, self._name_extractor)


  def testGeneral(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["number"],
        init_wait_time = 1)

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertGreater(elapsed_time, 0.8)
    self.assertLess(elapsed_time, 1.2)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.number.number, 2)


  def testEarlyFinish(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["number", "human_name"],
        init_wait_time = 1)

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "My name is two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    input_queue.put(None)

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertLess(elapsed_time, 0.2)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.human_name.name, "two")


  def testChooseOnly(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["human_name"],
        init_wait_time = 1)

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    input_queue.put(None)

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertLess(elapsed_time, 0.2)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.human_name.name, "two")


  def testBlankTimeout(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["human_name"],
        init_wait_time = .1,
        blank_timeout=.2,
    )

    input_queue = Queue.Queue()
    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertGreater(elapsed_time, 0.1)
    self.assertLess(elapsed_time, 0.35)
    self.assertEqual(select_result.intent.WhichOneof("intent"), "silence")


  def testBlankTimeoutWithInput(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["human_name"],
        init_wait_time = .1,
        blank_timeout=.2,
        confidence_threshold=0.95,
    )

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "two two two two two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertGreater(elapsed_time, 0.1)
    self.assertLess(elapsed_time, 0.35)
    self.assertTrue(select_result.intent.WhichOneof("intent") is None)


  def testBlankTimeoutStillSelect(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["number"],
        init_wait_time = 1,
        blank_timeout=.2,
    )

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertGreater(elapsed_time, 0.8)
    self.assertLess(elapsed_time, 1.2)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.number.number, 2)


  def testChooseBetter(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["number", "human_name"],
        init_wait_time = 1)

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "My name is two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    input_queue.put(None)

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertLess(elapsed_time, 0.2)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.human_name.name, "two")


  def testLateEnter(self):
    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["human_name"],
        init_wait_time = 1)

    time_start = time.time()

    input_queue = Queue.Queue()
    result_holder = [None]
    def WaitThread(input, output):
      output[0] = selector.Select(input)

    t = threading.Thread(target=WaitThread, args=(input_queue, result_holder))
    t.start()

    time.sleep(1.5)
    input_queue.put(RecognitionResult(
        source_transcript = "My name is two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    t.join()
    select_result = result_holder[0]

    elapsed_time = time.time() - time_start

    self.assertGreater(elapsed_time, 1.5)
    self.assertLess(elapsed_time, 1.7)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.human_name.name, "two")


  def testMap(self):

    def MapFunc(intent):
      if intent.intent.HasField("human_name"):
        intent.metadata.confidence_score = 0
      return intent

    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["number", "human_name"],
        init_wait_time = 1, map_func=MapFunc)

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "My name is two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    input_queue.put(None)

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertLess(elapsed_time, 0.2)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.number.number, 2)


  def testMapToNone(self):

    def MapFunc(intent):
      if intent.intent.HasField("human_name"):
        return None
      return intent

    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["number", "human_name"],
        init_wait_time = 1, map_func=MapFunc)

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "My name is two",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    input_queue.put(None)

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertLess(elapsed_time, 0.2)
    self.assertTrue(select_result is not None)
    self.assertEqual(select_result.intent.number.number, 2)


  def testConfidenceThreshold(self):

    def MapFunc(intent):
      if intent.intent.HasField("human_name"):
        return None
      return intent

    selector = TimedBestIntentSelector(
        extractors = [self._combined_extractor],
        intent_types = ["human_name"],
        init_wait_time = 1, confidence_threshold=0.95)

    input_queue = Queue.Queue()
    input_queue.put(RecognitionResult(
        source_transcript = "two three four",
        source_confidence_score = 0.8,
        source_is_complete_utterance = True,
        source_timestamp = time.time()))
    input_queue.put(None)

    time_start = time.time()
    select_result = selector.Select(input_queue)
    elapsed_time = time.time() - time_start

    self.assertLess(elapsed_time, 0.2)
    self.assertTrue(select_result.intent.WhichOneof("intent") is None)


if __name__ == '__main__':
  unittest.main()
