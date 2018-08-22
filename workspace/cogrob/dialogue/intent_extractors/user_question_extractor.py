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
from cogrob.dialogue.intent_extractors import extractor_base
from cogrob.dialogue.intent_util import lcs_matcher


class UserQuestionExtractor(extractor_base.ExtractorBase):

  def __init__(self):
    super(UserQuestionExtractor, self).__init__()
    self._lcs_matcher = lcs_matcher.LcsMatcher()


  def TryPresident(self, transcript):
    ORDER_UTTERANCE = {
      "first": 1,
      "1st": 1,
      "second": 2,
      "2nd": 2,
      "third": 3,
      "3rd": 2,
      "current": 45,
      "45th": 45,
    }

    ORDER_REGEX = "'{}'<order>".format("|".join(ORDER_UTTERANCE))

    templates = [
        "who is the {}(5) president(2)".format(ORDER_REGEX),
        "who is the {}(5) president(2) of the united(2) states(2)".format(
            ORDER_REGEX),
        "{}(5) president(2)".format(ORDER_REGEX),
    ]

    hypothesis = None
    match_result = self._lcs_matcher.MatchMultiTemplateStr(
        templates, transcript)

    if match_result is not None and "order" in match_result.fields:
      if (hypothesis is None or
          hypothesis.metadata.confidence_score < match_result.likelihood):
        hypothesis = hypothesis_pb2.IntentHypothesis()
        hypothesis.intent.president_question.president_number = (
            ORDER_UTTERANCE[match_result.fields["order"]])
        hypothesis.metadata.confidence_score = match_result.likelihood

    if hypothesis is not None:
      return [hypothesis]

    return []


  def ExtractImpl(self, transcript, metadata):
    del metadata

    results = []
    results += self.TryPresident(transcript)

    return results
