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
import math
import Queue
import time

from cogrob.dialogue.intent import hypothesis_pb2
from cogrob.dialogue.intent_selector import intent_selector_base
from cogrob.dialogue.intent_extractors import extractor_base

FLAGS = flags.FLAGS

flags.DEFINE_float("timed_best_intent_selector_polling_interval_secs", 0.1,
                   "Interval for polling the recognition result queue.")

IntentSelectorBase = intent_selector_base.IntentSelectorBase
RecognitionResult = intent_selector_base.RecognitionResult

class TimedBestIntentSelector(IntentSelectorBase):

  def __init__(
      self, extractors, intent_types, init_wait_time=0, confidence_threshold=0,
      map_func=lambda x:x, blank_timeout=0):
    # map_func allows modifying intents, e.g. remove some intents, or change
    # confidence_score based on a user-provided function. If such function
    # returns None, that intent is removed.
    # If after init_wait_time, we have not received an intrim utterance in
    # blank_timeout seconds, we will give up and return None. A zero value
    # disables this feature.
    super(TimedBestIntentSelector, self).__init__()
    self._extractors = extractors
    self._init_wait_time = float(init_wait_time)
    self._intent_types = intent_types
    self._confidence_threshold = confidence_threshold
    self._map_func = map_func
    self._blank_timeout = blank_timeout

    for extractor in self._extractors:
      assert isinstance(extractor, extractor_base.ExtractorBase)

    for intent_type in self._intent_types:
      assert isinstance(intent_type, str)


  def SelectImpl(self, input_queue):
    # input_queue is a Queue. This function returns an IntentHypothesis or None.

    start_time = time.time()
    local_input_queue = []
    input_stopped = False
    speaking_stopped = False
    last_received_time = time.time()
    is_silence = True

    while True:
      # Caller would set a deadline, and if reading from input_queue is None,
      # this function must return.

      # Moves everything from input_queue to local_input_queue.
      while True:
        try:
          recognition_result = input_queue.get(
              block=True,
              timeout=FLAGS.timed_best_intent_selector_polling_interval_secs)
          last_received_time = time.time()
        except Queue.Empty:
          break

        if recognition_result is None:
          # This means we have reached the end of the input queue, should try
          # no more.
          input_stopped = True
          break
        else:
          assert isinstance(recognition_result, RecognitionResult)
          is_silence = False
          local_input_queue.append(recognition_result)
          if time.time() - start_time > self._init_wait_time:
            break

      time_since_start = time.time() - start_time

      if not input_stopped and time_since_start < self._init_wait_time:
        continue
      elif (self._blank_timeout > 0
            and time.time() - last_received_time > self._blank_timeout):
        # This means we have been waiting too long, the human should have
        # stopped speaking.
        speaking_stopped = True

      # Starting here, we can compare all the results and select the best one.
      newly_extracted_results = []

      for recognition_result in local_input_queue:

        # Creates a metadata object for the extractors to use as input.
        input_metadata = hypothesis_pb2.IntentHypothesisMetadata()
        input_metadata.source_transcript = recognition_result.source_transcript
        input_metadata.source_confidence_score = (
            recognition_result.source_confidence_score)
        input_metadata.source_is_complete_utterance = (
            recognition_result.source_is_complete_utterance)
        input_metadata.source_timestamp.seconds = int(math.floor(
            recognition_result.source_timestamp))
        input_metadata.source_timestamp.nanos = int(1000000000 * (
            recognition_result.source_timestamp -
            input_metadata.source_timestamp.seconds))

        for extractor in self._extractors:
          extractor.Extract(input_metadata, newly_extracted_results.append)

      newly_extracted_results = map(self._map_func, newly_extracted_results)
      newly_extracted_results = [
          r for r in newly_extracted_results if r is not None]

      newly_extracted_results = [
          r for r in newly_extracted_results if (
          r.intent.WhichOneof("intent") in self._intent_types
          and r.metadata.confidence_score > self._confidence_threshold)]

      if len(newly_extracted_results):
        return max(newly_extracted_results,
                   key=lambda x:x.metadata.confidence_score)
      else:
        if input_stopped or speaking_stopped:
          result = hypothesis_pb2.IntentHypothesis()
          result.metadata.extractor_name = self.__class__.__name__
          current_time = time.time()
          result.metadata.source_timestamp.seconds = int(
              math.floor(current_time))
          result.metadata.source_timestamp.nanos = int(1000000000 * (
              current_time - result.metadata.source_timestamp.seconds))
          # Only set resutl.intent.intent when silence, if none of the extractor
          # are able to find out anything, not to set result.intent so
          # "WhichOneof" would return None, and we can tell it is unknown.
          if is_silence:
            result.intent.silence.SetInParent()
          return result
        else:
          # Clears the local_input_queue. The previously received results are
          # all useless now.
          local_input_queue = []
