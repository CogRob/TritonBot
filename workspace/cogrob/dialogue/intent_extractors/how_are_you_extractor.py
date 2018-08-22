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


class HowAreYouExtractor(extractor_base.ExtractorBase):

  POSTIVE_LCS_TEMPLATES = [
    "I am 'awesome|great|good'(3)", "pretty good", "not bad", "good",
    "great", "pretty good(3)", "im ok", "'.*'(.2) ok", "'.*'(.2) good",
    "'.*'(.2) good" , "very well(10)", "i am doing fine(10)", "doing fine"]

  NEGATIVE_LCS_TEMPLATES = [
    "Not so well", "Not good", "just so so", "meh", "could use a beer(3)",
    "bad", "terrible"]

  def __init__(self):
    super(HowAreYouExtractor, self).__init__()
    self._lcs_matcher = lcs_matcher.LcsMatcher()


  def ExtractImpl(self, transcript, metadata):
    del metadata
    postive_result = self._lcs_matcher.MatchMultiTemplateStr(
        self.POSTIVE_LCS_TEMPLATES, transcript)
    negative_result = self._lcs_matcher.MatchMultiTemplateStr(
        self.NEGATIVE_LCS_TEMPLATES, transcript)

    hypothesis = None

    if postive_result is not None:
      hypothesis = hypothesis_pb2.IntentHypothesis()
      hypothesis.intent.how_are_you_response.is_positive = True
      hypothesis.metadata.confidence_score = postive_result.likelihood

    if negative_result is not None and (
        hypothesis is None
        or hypothesis.metadata.confidence_score < negative_result.likelihood):
      hypothesis = hypothesis_pb2.IntentHypothesis()
      hypothesis.intent.how_are_you_response.is_negative = True
      hypothesis.metadata.confidence_score = negative_result.likelihood

    if hypothesis is not None:
      return [hypothesis]

    return []
