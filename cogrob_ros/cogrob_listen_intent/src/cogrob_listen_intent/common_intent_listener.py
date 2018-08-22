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

import use_cogrob_workspace
from cogrob.dialogue.intent_extractors import action_extractor
from cogrob.dialogue.intent_extractors import combined_extractor
from cogrob.dialogue.intent_extractors import general_text_extractor
from cogrob.dialogue.intent_extractors import how_are_you_extractor
from cogrob.dialogue.intent_extractors import name_extractor
from cogrob.dialogue.intent_extractors import number_extractor
from cogrob.dialogue.intent_extractors import quiz_extractor
from cogrob.dialogue.intent_extractors import user_question_extractor
from cogrob.dialogue.intent_extractors import yes_no_extractor
from cogrob.dialogue.listen_for_intent import intent_listener
from cogrob.dialogue.intent_selector import timed_best_intent_selector

from cogrob_listen_intent import gcloud_speech_recognizer

IntentListener = intent_listener.IntentListener
GcloudSpeechRecognizer = gcloud_speech_recognizer.GcloudSpeechRecognizer
TimedBestIntentSelector = timed_best_intent_selector.TimedBestIntentSelector


def GetAllExtractorsInst():
  if not hasattr(GetAllExtractorsInst, "_all_extractor_instances"):
    GetAllExtractorsInst._all_extractor_instances = [
      action_extractor.ActionExtractor(),
      general_text_extractor.GeneralTextExtractor(),
      how_are_you_extractor.HowAreYouExtractor(),
      name_extractor.NameExtractor(),
      number_extractor.NumberExtractor(),
      quiz_extractor.QuizExtractor(),
      user_question_extractor.UserQuestionExtractor(),
      yes_no_extractor.YesNoExtractor(),
      yes_no_extractor.DontKnowExtractor(),
      yes_no_extractor.SayAgainExtractor(),
    ]
  return GetAllExtractorsInst._all_extractor_instances


def GetDefaultSpeechRecognizer():
  if not hasattr(GetDefaultSpeechRecognizer, "_recognizer"):
    GetDefaultSpeechRecognizer._recognizer = GcloudSpeechRecognizer()
  return GetDefaultSpeechRecognizer._recognizer


def GetDefaultSpeechRecognizerNoIntrimResults():
  if not hasattr(GetDefaultSpeechRecognizerNoIntrimResults, "_recognizer"):
    GetDefaultSpeechRecognizerNoIntrimResults._recognizer = (
        GcloudSpeechRecognizer(suppress_interim_results=True))
  return GetDefaultSpeechRecognizerNoIntrimResults._recognizer


# Usage:
# listner = GetCommonIntentListener(["expected", "intent", "types"])
# result = listner.ListenForIntent()
def GetCommonIntentListener(
    intent_types, init_wait_time=2, confidence_threshold=0.7,
    map_func=lambda x:x, blank_timeout=10, ignore_intrim_results=False):
  selector = TimedBestIntentSelector(
      extractors=GetAllExtractorsInst(), intent_types=intent_types,
      init_wait_time=init_wait_time, confidence_threshold=confidence_threshold,
      map_func=map_func, blank_timeout=blank_timeout)

  if ignore_intrim_results:
    recognizer = GetDefaultSpeechRecognizerNoIntrimResults()
  else:
    recognizer = GetDefaultSpeechRecognizer()

  return IntentListener(recognizer, selector)
