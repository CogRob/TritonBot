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

import unittest

from cogrob.dialogue.intent import hypothesis_pb2
from cogrob.dialogue.intent_extractors import yes_no_extractor

class TestYesNoExtractor(unittest.TestCase):
  def setUp(self):
    self._yes_no_extractor = yes_no_extractor.YesNoExtractor()


  def testYes(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Yes"
    output = []
    self._yes_no_extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.yes_no.is_yes)
      self.assertFalse(result.intent.yes_no.is_no)


  def testYesAlternative(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Yeah"
    output = []
    self._yes_no_extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.yes_no.is_yes)
      self.assertFalse(result.intent.yes_no.is_no)


  def testNo(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "No"
    output = []
    self._yes_no_extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.yes_no.is_no)
      self.assertFalse(result.intent.yes_no.is_yes)


  def testMaybeNot(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Maybe not"
    output = []
    self._yes_no_extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.yes_no.is_no)
      self.assertFalse(result.intent.yes_no.is_yes)


  def testMaybe(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Maybe"
    output = []
    self._yes_no_extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.HasField("maybe"))
      self.assertFalse(result.intent.HasField("yes_no"))


class TestDontKnowExtractor(unittest.TestCase):
  def setUp(self):
    self._dont_know_extractor = yes_no_extractor.DontKnowExtractor()

  def testDontKnow(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "I dont know"
    output = []
    self._dont_know_extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.HasField("dont_know"))


class TestSayAgainExtractor(unittest.TestCase):
  def setUp(self):
    self._say_again_extractor = yes_no_extractor.SayAgainExtractor()

  def testDontKnow(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Can you say that again"
    output = []
    self._say_again_extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.HasField("say_again"))


if __name__ == '__main__':
  unittest.main()
