#!/usr/bin/env python
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


from __future__ import print_function
from cogrob_listen_intent import gcloud_speech_recognizer
import os
import rospy
import signal
import threading
import time

GcloudSpeechRecognizer = gcloud_speech_recognizer.GcloudSpeechRecognizer


def SignalIntHandler(signal, frame):
  print("Received SIGINT.")
  os._exit(-1)


def main():
  signal.signal(signal.SIGINT, SignalIntHandler)

  rospy.init_node('gcloud_speech_recognizer_demo', anonymous=True)

  spin_thread = threading.Thread(target=rospy.spin)
  spin_thread.daemon = True
  spin_thread.start()

  recognizer = GcloudSpeechRecognizer()
  with recognizer as q:
    time_start = time.time()

    while time.time() < time_start + 10:
      try:
        result = q.get(block=True, timeout=0.1)
      except:
        continue
      if result is not None:
        print("Result in the queue: \n{}\n".format(result))
      else:
        break

    print("Done")


if __name__ == "__main__":
  main()
