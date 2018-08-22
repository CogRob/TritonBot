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
import Queue

RecognitionResult = collections.namedtuple(
    "RecognitionResult", ["source_transcript", "source_confidence_score",
                          "source_is_complete_utterance", "source_timestamp"])


class SpeechRecognizerInterface(object):
  # The derived class should implement StartImpl() and StartImpl().

  def __init__(self):
    self._interface_initialized = True


  def StartImpl(self, result_queue):
    raise NotImplementedError(
        "{} should implement StartImpl()".format(self.__class__.__name__))


  def StopImpl(self):
    raise NotImplementedError(
        "{} should implement StopImpl()".format(self.__class__.__name__))


  def __enter__(self):
    try:
      assert self._interface_initialized
    except:
      raise AssertionError(
        "IntentSelectorBase not initialized, did you call " +
        "IntentSelectorBase.__init__ on {}?".format(self.__class__.__name__))

    result_queue = Queue.Queue()
    self.StartImpl(result_queue)

    return result_queue


  def __exit__(self, type, value, traceback):
    try:
      assert self._interface_initialized
    except:
      raise AssertionError(
        "IntentSelectorBase not initialized, did you call " +
        "IntentSelectorBase.__init__ on {}?".format(self.__class__.__name__))

    self.StopImpl()
