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

# Extract intent with template matching using LCS-like (longgest common
# subsequence) method.

# Template example:
# "my(.5) name(3) is 'Bill|Shawn'<name>(10)"
# Element syntax: 'regex'<capture_name>(weight)
# The single quote (') around 'regex' can be omitted
# weight is a float number, and can be omitted
# capture_name is and identifier, and can be omitted
# Matched regex: ^(\w+|'.+')(?:<(\w+)>)?(?:\(([-+]?\d*\.\d+|\d+)\))?$
# Each element must be fully matched

from absl import logging
import collections
import re
import string

# Represents matching result of a string and a template
MatchResult = collections.namedtuple("MatchResult", ["likelihood", "fields"])
# Represents an element in the template string
TemplateElement = collections.namedtuple(
  "TemplateElement", ["regex", "name", "weight"])
# Represents an element in the template string
Template= collections.namedtuple("Template", ["elements", "total_weight"])


class LcsMatcher(object):

    ELEMENT_REGEX = r"^(\w+|'.+')(?:<(\w+)>)?(?:\(([-+]?\d*\.\d+|\d+)\))?$"
    _element_matcher = re.compile(ELEMENT_REGEX)
    _regex_cache = {}

    def __init__(self):
      # Prepare facilities for regex matching.
      self._cached_parse_template = {}


    @classmethod
    def CachedCompileRegex(cls, pattern):
      if pattern not in cls._regex_cache:
        cls._regex_cache[pattern] = re.compile(pattern, flags=re.IGNORECASE)
      return cls._regex_cache[pattern]


    def ParseTemplate(self, template_str):
      if template_str not in self._cached_parse_template:
        self._cached_parse_template[template_str] = (
            self._ParseTemplate(template_str))

      return self._cached_parse_template[template_str]


    def _ParseTemplate(self, template_str):
      # Converts template_str (str) to Template, or returns None if it fails.

      template_elements = []
      weight_sum = 0.0

      # Chop the template and parse them seperately.
      element_strs = template_str.strip().split()
      for element_str in element_strs:
        # Matches against element regex.
        match_result = self._element_matcher.match(element_str)
        if match_result is None:
          logging.error(
              "Pattern '{}' in '{}' does not match ELEMENT_REGEX.".format(
              element_str, template_str))
          return None

        # Extracts the pattern string.
        pattern = match_result.group(1)
        if pattern.startswith("'"):
          if pattern[-1] != "'":
            logging.error(
                "Pattern \"{}\" in \"{}\" has an open single quote.".format(
                pattern, template_str))
            return None
          pattern = pattern[1:-1]
        regex = self.CachedCompileRegex("^" + pattern + "$")

        # Extracts captured name.
        capture_name = match_result.group(2)
        if capture_name == "":
          capture_name = None

        # Extracts weight, default to 1
        weight = 1.0
        if match_result.group(3) is not None:
          try:
            weight = float(match_result.group(3))
          except Exception as e:
            logging.error(
                "Pattern \"{}\" in \"{}\" has invalid weight.".format(
                pattern, template_str))
            return None

        element = TemplateElement(regex=regex, name=capture_name, weight=weight)

        # Collects all the template_elements.
        template_elements.append(element)
        weight_sum += weight

      return Template(elements=template_elements, total_weight=weight_sum)


    def MatchTemplate(self, template, input):
      # Type: (Template, str) -> MatchResult

      # Note: The likelihood is used as confidence score. It is calculated as:
      # min(matched_weight / total_weight,
      #     matched_weight / (num_input_words - num_matched_words
      #         + matched_weight))

      # First, filters the input so each part only contains letters(lowercase)
      # and numbers.
      input_elements = map(
        lambda s: filter(
          lambda c: c in (string.ascii_lowercase + string.digits),
          s),
        input.lower().split())

      # Append in the front so that 0 represents nothing
      input_elements = [None] + input_elements
      template_elements = [None] + template.elements

      # Classic longest common substring (LCS) dynamic programming.
      best = [[0 for _ in range(len(template_elements))]
              for _ in range(len(input_elements))]
      best_from = [[(0, 0) for _ in range(len(template_elements))]
                   for _ in range(len(input_elements))]
      for i in range(1, len(input_elements)):
        for j in range(1, len(template_elements)):
          # Default is skipping current template element or input string.
          best[i][j] = best[i][j - 1]
          best_from[i][j] = best_from[i][j - 1]
          if (best[i - 1][j] > best[i][j]):
            best[i][j] = best[i - 1][j]
            best_from[i][j] = best_from[i - 1][j]
          # If the template matches the input string, we can update the result.
          if template_elements[j].regex.match(input_elements[i]):
            if (best[i - 1][j - 1] + template_elements[j].weight > best[i][j]):
              best[i][j] = (best[i - 1][j - 1] + template_elements[j].weight)
              best_from[i][j] = (i, j)

      # Traceback to get the result.
      result = {}
      matched_weight = (
          best[len(input_elements) - 1][len(template_elements) - 1])
      total_weight = template.total_weight

      num_matched_words = 0

      # If the last one is not a match, find the last match and save to the
      # result, otherwise i, j will not change.
      i, j = best_from[len(input_elements) - 1][len(template_elements) - 1]

      while i != 0 and j != 0:
        num_matched_words += 1
        if template_elements[j].name is not None:
          result[template_elements[j].name] = input_elements[i]
        i, j = best_from[i - 1][j - 1]

      num_input_words = len(input_elements) - 1

      if (total_weight == 0 or
          num_input_words - num_matched_words + matched_weight == 0):
        return None
      # Calculate the likelihood.
      likelihood = min(matched_weight / total_weight,
                       matched_weight / (num_input_words - num_matched_words
                                         + matched_weight))

      return MatchResult(likelihood=likelihood, fields=result)


    def MatchTemplateStr(self, template_str, input):
      # Type: (str, str) -> MatchResult
      template = self.ParseTemplate(template_str)
      if template is None:
        logging.error("Template \"{}\" contains errors.".format(template_str))
        return None
      else:
        return self.MatchTemplate(template, input)


    def MatchMultiTemplateStr(self, templates, input):
      # Type: (List(str), str) -> MatchResult
      """ Match input against all of the templates and returns the optimal."""
      results = map(lambda s: self.MatchTemplateStr(s, input), templates)
      results = filter(lambda x: x is not None, results)
      if results:
        return max(results, key=lambda m: m.likelihood)
      return None
