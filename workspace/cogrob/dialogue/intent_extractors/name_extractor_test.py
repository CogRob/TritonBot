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
from cogrob.dialogue.intent_extractors import name_extractor

class TestIntents(unittest.TestCase):
  def setUp(self):
    self._extractor = name_extractor.NameExtractor()


  def testMyNameIs(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "My name is Shengye"
    output = []
    self._extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.HasField("human_name"))
      self.assertEqual(result.intent.human_name.name.lower(), "shengye")


  def testNameTwoWords(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Shengye Wang"
    output = []
    self._extractor.Extract(input, output.append)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.HasField("human_name"))
      self.assertEqual(result.intent.human_name.name.lower(), "shengye wang")


  def testNotFamiliarNamePass(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "isy Xheye"
    output = []
    self._extractor.Extract(input, output.append)
    output = filter(lambda x:x.metadata.confidence_score > 0.7, output)
    self.assertGreater(len(output), 0)

    input.source_transcript = "Xheye"
    output = []
    self._extractor.Extract(input, output.append)
    output = filter(lambda x:x.metadata.confidence_score > 0.7, output)
    self.assertGreater(len(output), 0)


  def testNotFamiliarNameFail(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Ba namy isy Xheye"
    output = []
    self._extractor.Extract(input, output.append)
    output = filter(lambda x:x.metadata.confidence_score > 0.3, output)
    self.assertEqual(len(output), 0)


  def testFamiliarName(self):
    input = hypothesis_pb2.IntentHypothesisMetadata()
    input.source_transcript = "Ba namy isy Shengye"
    output = []
    self._extractor.Extract(input, output.append)
    output = filter(lambda x:x.metadata.confidence_score > 0.3, output)
    self.assertGreater(len(output), 0)
    for result in output:
      self.assertTrue(result.intent.HasField("human_name"))
      self.assertEqual(result.intent.human_name.name.lower(), "shengye")


if __name__ == '__main__':
  unittest.main()
