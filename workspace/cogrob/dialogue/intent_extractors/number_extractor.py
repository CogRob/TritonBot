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


class NumberExtractor(extractor_base.ExtractorBase):

  NUMBER_UTTERANCE = {
    0 : ["zero", "0", "o", "oh", ],
    1 : ["one", "1", ],
    2 : ["two", "2", ],
    3 : ["three", "3", "tree", ],
    4 : ["four", "4", ],
    5 : ["five", "5", "fif", ],
    6 : ["six", "6", ],
    7 : ["seven", "7", ],
    8 : ["eight", "8", "ate", ],
    9 : ["nine", "9", "niner", ],
    10 : ["ten", "10", ],
  }

  def __init__(self):
    super(NumberExtractor, self).__init__()
    self._lcs_matcher = lcs_matcher.LcsMatcher()


  def ExtractImpl(self, transcript, metadata):
    del metadata

    hypothesis = None
    for number, templates in self.NUMBER_UTTERANCE.items():
      match_result = self._lcs_matcher.MatchMultiTemplateStr(
          templates, transcript)

      if match_result is not None:
        if (hypothesis is None or
            hypothesis.metadata.confidence_score < match_result.likelihood):
          hypothesis = hypothesis_pb2.IntentHypothesis()
          hypothesis.intent.number.number = number
          hypothesis.metadata.confidence_score = match_result.likelihood

    if hypothesis is not None:
      return [hypothesis]

    return []
