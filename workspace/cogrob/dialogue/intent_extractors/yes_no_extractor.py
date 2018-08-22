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

from cogrob.dialogue.intent import hypothesis_pb2
from cogrob.dialogue.intent import yes_no_pb2
from cogrob.dialogue.intent_extractors import extractor_base
from cogrob.dialogue.intent_extractors import simple_lcs_extractor
from cogrob.dialogue.intent_util import lcs_matcher


class YesNoExtractor(extractor_base.ExtractorBase):

  YES_LCS_TEMPLATES = [
    "Sure(10)", "Yes(10)", "Yes I would", "Yep(10)", "Yeah(10)",
    "Definitely(10)", "Alright(10)", "Why not(10)", "okie dokie(10)",
    "okay", "OK", "Please", "of course"]

  NO_LCS_TEMPLATES = [
    "No(10)", "Not(10) right now", "No(10) thanks", "No(10) thank you",
    "Maybe later(10)", "maybe not", "maybe no", "maybe some other time",
    "Not(10) now(10)", "next time"]

  MAYBE_LCS_TEMPLATES = ["Maybe", "possibly"]

  def __init__(self):
    super(YesNoExtractor, self).__init__()
    self._lcs_matcher = lcs_matcher.LcsMatcher()


  def ExtractImpl(self, transcript, metadata):
    del metadata
    yes_result = self._lcs_matcher.MatchMultiTemplateStr(
        self.YES_LCS_TEMPLATES, transcript)
    no_result = self._lcs_matcher.MatchMultiTemplateStr(
        self.NO_LCS_TEMPLATES, transcript)
    maybe_result = self._lcs_matcher.MatchMultiTemplateStr(
        self.MAYBE_LCS_TEMPLATES, transcript)

    hypothesis = None

    if yes_result is not None:
      hypothesis = hypothesis_pb2.IntentHypothesis()
      hypothesis.intent.yes_no.is_yes = True
      hypothesis.metadata.confidence_score = yes_result.likelihood

    if no_result is not None and (
        hypothesis is None
        or hypothesis.metadata.confidence_score < no_result.likelihood):
      hypothesis = hypothesis_pb2.IntentHypothesis()
      hypothesis.intent.yes_no.is_no = True
      hypothesis.metadata.confidence_score = no_result.likelihood

    if maybe_result is not None and (
        hypothesis is None
        or hypothesis.metadata.confidence_score < maybe_result.likelihood):
      hypothesis = hypothesis_pb2.IntentHypothesis()
      hypothesis.intent.maybe.SetInParent()
      hypothesis.metadata.confidence_score = maybe_result.likelihood

    if hypothesis is not None:
      return [hypothesis]

    return []


class DontKnowExtractor(simple_lcs_extractor.SimpleLcsExtractor):

  TEMPLATES = {
    "dont_know" : [
        "I dont(5) know(5)", "Do not know", "Pass(10)", "You tell me",
        "i have no idea", "no idea", "i have no clue", "no clue",
        "no", "i am not sure", "im not sure", "not sure"
    ],
  }

  def __init__(self):
    super(DontKnowExtractor, self).__init__(self.TEMPLATES)


class SayAgainExtractor(simple_lcs_extractor.SimpleLcsExtractor):

  TEMPLATES = {
    "say_again" : ["say again(10)", "whats that", "repeat(10)"],
  }

  def __init__(self):
    super(SayAgainExtractor, self).__init__(self.TEMPLATES)
