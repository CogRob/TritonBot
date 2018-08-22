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

from absl import logging
import collections
import os
import StringIO
import sys
import traceback

from cogrob.dialogue.intent import hypothesis_pb2


class ExtractorBase(object):
  # Base class for all extractors.

  def __init__(self):
    self._base_initialized = True
    self._extractor_name = self.__class__.__name__


  def VerifyExtractorBaseInit(self):
    try:
      assert self._base_initialized
    except:
      raise AssertionError(
        "ExtractorBase not initialized, did you call ExtractorBase.__init__ " +
        "on {}?".format(self.__class__.__name__))


  def GetExtractorName(self):
    self.VerifyExtractorBaseInit()
    return self._extractor_name


  def Extract(self, metadata_input, output_func):
    self.VerifyExtractorBaseInit()

    # metadata_input is the input, which contains all the source
    # information. At minimum, it should contain source_transcript.
    # output_func takes a IntentHypothesis as parameter. We don't use Queue
    # here to also allow synchronized output (e.g. publish to a pub/sub
    # platform like ROS)
    is_error = False

    if not metadata_input.source_transcript:
      raise ValueError("source_transcript does not exist in the input.")

    hypotheses = []
    # Returns a IntentHypothesis
    try:
      hypotheses = self.ExtractImpl(
        metadata_input.source_transcript, metadata_input)
    except Exception as err:
      # Don't die here. Extractor code could be imperfect, but we cannot afford
      # killing the program.
      is_error = True
      error_info = "{}, stack trace: \n{}".format(
          str(err), traceback.format_exc())
      logging.error(
          "ExtractImpl on %s failed: %s", self.GetExtractorName(), error_info)

    # hypotheses should be a [IntentHypothesis]
    if not is_error:
      if not isinstance(hypotheses, collections.Iterable):
        is_error = True
        logging.error(
            "ExtractImpl on %s returned non-Iterable object: %s",
            self.GetExtractorName(), str(hypotheses))
      else:
        for hypothesis in hypotheses:
          if not isinstance(hypothesis, hypothesis_pb2.IntentHypothesis):
            is_error = True
            logging.error(
                "ExtractImpl on %s returned non IntentHypothesis object: %s",
                self.GetExtractorName(), str(hypothesis))

    if not is_error:
      for hypothesis in hypotheses:
        output_hypothesis = hypothesis_pb2.IntentHypothesis()
        output_hypothesis.metadata.MergeFrom(metadata_input)
        if output_hypothesis.metadata.extractor_name is None:
          output_hypothesis.metadata.extractor_name = self.GetExtractorName()
        output_hypothesis.MergeFrom(hypothesis)
        output_func(output_hypothesis)
    else:
      logging.error("Error on %s, result not saved.", self.GetExtractorName())


  def ExtractImpl(self, transcript, metadata):
    # metadata only contains source information, and overriden function is not
    # required to use it.
    # Extract intent from transcript.
    # Returns a list of IntentHypothesis, it is fine not to set any metadata.

    # Subclass should implement this method.
    raise NotImplementedError(
        "{} should implement ExtractImpl()".format(GetExtractorName()))
