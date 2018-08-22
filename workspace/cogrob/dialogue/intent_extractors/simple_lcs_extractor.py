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


class SimpleLcsExtractor(extractor_base.ExtractorBase):

  def __init__(self, intent_name_to_lcs_templates_map, lcs_threshold=0.7):
    # intent_name_type_to_lcs_templates_map is something like
    # {"want_selfie": ["I want a selfie"]}

    super(SimpleLcsExtractor, self).__init__()
    self._intent_to_templates = intent_name_to_lcs_templates_map
    self._lcs_matcher = lcs_matcher.LcsMatcher()
    self._lcs_threshold = lcs_threshold


  def ExtractImpl(self, transcript, metadata):
    del metadata

    results = []
    for intent_name, templates in self._intent_to_templates.items():
      match_result = self._lcs_matcher.MatchMultiTemplateStr(
          templates, transcript)

      if match_result is not None:
        if match_result.likelihood < self._lcs_threshold:
          continue
        hypothesis = hypothesis_pb2.IntentHypothesis()
        getattr(hypothesis.intent, intent_name).SetInParent()
        assert hypothesis.intent.HasField(intent_name)
        hypothesis.metadata.confidence_score = match_result.likelihood

        results.append(hypothesis)

    return results
