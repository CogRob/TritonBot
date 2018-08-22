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
from cogrob.navigation.tour import tour_info


class TourExtractor(extractor_base.ExtractorBase):

  def __init__(self):
    super(TourExtractor, self).__init__()
    self._all_tour_points = tour_info.GetAllTourPoints()
    self._lcs_matcher = lcs_matcher.LcsMatcher()


  def ExtractImpl(self, transcript, metadata):
    del metadata

    results = []

    # Generates WantToGoLocation.
    for tour_point in self._all_tour_points:
      # Match correct answers.
      match_result = self._lcs_matcher.MatchMultiTemplateStr(
          tour_point.lcs_templates, transcript)
      if match_result is not None:
        hypothesis = hypothesis_pb2.IntentHypothesis()
        hypothesis.intent.want_to_go_location.location_id = tour_point.id
        hypothesis.metadata.confidence_score = match_result.likelihood
        results.append(hypothesis)

    # Generates WantTour.
    WANT_TOUR_TEMPLATES = [
        "show(10) around(10)", "give us a tour(10)",
        "give(10) us(10) tour(10)", "give tour", "tour(.1)"
    ]
    match_result = self._lcs_matcher.MatchMultiTemplateStr(
        WANT_TOUR_TEMPLATES, transcript)
    if match_result is not None:
      hypothesis = hypothesis_pb2.IntentHypothesis()
      hypothesis.intent.want_tour.SetInParent()
      hypothesis.metadata.confidence_score = match_result.likelihood
      results.append(hypothesis)

    return results
