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

from absl import logging
from absl import flags
import math
import os
import os.path
import rospy
import signal
import subprocess
import sys
import socket
import time

import cogrob_rosbag_msgs.srv

FLAGS = flags.FLAGS
flags.DEFINE_string("rosbag_record_cmd", "/opt/ros/indigo/lib/rosbag/record",
                    "rosbag record executable file name.")
flags.DEFINE_string("config_dir", "", "Configuration directory.")
flags.DEFINE_float(
    "die_too_soon_time", 0.1,
    "Wait time after rosbag is spawned, if it dies too soon, return error")
flags.DEFINE_float("kill_wait_time", 3,
                   "Wait time in secs after sending SIGINT/SIGKILL.")
flags.DEFINE_string("universal_logger_prefix", "/home/cogrob_log",
                    "Root directory for universal_logger, absolute path.")

class RosBagProcessManager(object):

  def __init__(self, config_name):
    self._config_name = config_name
    self._process = None


  def GetBagFilename(self):
    path_prefix = FLAGS.universal_logger_prefix
    source_id = socket.gethostname()
    date_string = time.strftime("%Y/%m/%d")
    log_namespace = "rosbag/{}".format(self._config_name)
    final_filename = "{}.bag".format(int(math.floor(time.time())))
    return os.path.join(
        path_prefix, source_id, date_string, log_namespace, final_filename)


  def MakeParentDir(self, filename):
    dir_name = os.path.dirname(filename)
    if not os.path.exists(dir_name):
      os.makedirs(dir_name)


  def Start(self):
    config_path = os.path.join(FLAGS.config_dir, self._config_name)
    with open(config_path, "r") as fp:
      args = fp.read().split()
    rospy.loginfo("Arguments are %s.", " ".join(args))
    bag_filepath = self.GetBagFilename()
    self.MakeParentDir(bag_filepath)
    output_name_args = ["--output-name={}".format(bag_filepath)]
    self._process = subprocess.Popen(
        [FLAGS.rosbag_record_cmd] + output_name_args + args,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    start_time = time.time()
    while (self._process.poll() is None
           and time.time() - start_time < FLAGS.die_too_soon_time):
      time.sleep(0.01)

    if self._process.poll() is not None:
      stdout = self._process.stdout.read()
      rospy.logerr("rosbag finished prematurely with stdout: \n%s", stdout)
      raise RuntimeError("rosbag exited prematurely.")


  def CheckAlive(self):
    if self._process is None:
      return False
    if self._process.poll() is not None:
      stdout = self._process.stdout.read()
      rospy.loginfo("rosbag finished with stdout: \n%s", stdout)
      self._process = None
      return False
    return True


  def Stop(self):
    if self._process is None:
      raise RuntimeError("Already stopped.")

    rospy.loginfo("Prepare to send SIGINT")
    os.kill(self._process.pid, signal.SIGINT)
    rospy.loginfo("Sent SIGINT")
    start_time = time.time()
    while (self._process.poll() is None
           and time.time() - start_time < FLAGS.kill_wait_time):
      time.sleep(0.1)

    rospy.loginfo("Poll now returns {}".format(self._process.poll()))

    used_sigkill = False
    if self._process.poll() is None:
      # SIGINT failed, let's send SIGKILL.
      self._process.kill()
      start_time = time.time()
      while (self._process.poll() is None
             and time.time() - start_time < FLAGS.kill_wait_time):
        time.sleep(0.1)
      if self._process.poll() is None:
        raise RuntimeError("Even SIGKILL won't kill rosbag.")
      else:
        used_sigkill = True

    stdout = self._process.stdout.read()
    rospy.loginfo("rosbag finished with stdout: \n%s", stdout)
    if used_sigkill:
      raise RuntimeError("Had to use SIGKILL to kill rosbag.")


class RosBagServicer(object):
  def __init__(self):
    self._process_managers = {}
    self._start_recording = rospy.Service(
        "/cogrob/rosbag/start_recording",
        cogrob_rosbag_msgs.srv.StartRecording, self.StartRecording)
    self._finish_recording = rospy.Service(
        "/cogrob/rosbag/finish_recording",
        cogrob_rosbag_msgs.srv.FinishRecording, self.FinishRecording)


  def StartRecording(self, request):
    # Clean up if already finished.
    if request.config_name in self._process_managers:
      if not self._process_managers[request.config_name].CheckAlive():
        self._process_managers.pop(request.config_name)

    # Handle restart request.
    if request.config_name in self._process_managers:
      if request.restart:
        try:
          self._process_managers[request.config_name].Stop()
        except RuntimeError as e:
          return cogrob_rosbag_msgs.srv.StartRecordingResponse(
              success=False,
              error_info="Error stopping existing process: {}".format(str(e)))
        self._process_managers.pop(request.config_name)
      else:
        return cogrob_rosbag_msgs.srv.StartRecordingResponse(
            success=False, error_info="Already running.")

    assert request.config_name not in self._process_managers
    self._process_managers[request.config_name] = RosBagProcessManager(
        request.config_name)

    try:
      self._process_managers[request.config_name].Start()
    except RuntimeError as e:
      return cogrob_rosbag_msgs.srv.StartRecordingResponse(
          success=False,
          error_info="Error starting rosbag: {}".format(str(e)))

    return cogrob_rosbag_msgs.srv.StartRecordingResponse(success=True)


  def FinishRecording(self, request):
    rospy.loginfo("FinishRecording request received.")
    # Clean up if already finished.
    if request.config_name in self._process_managers:
      if not self._process_managers[request.config_name].CheckAlive():
        self._process_managers.pop(request.config_name)

    if request.config_name in self._process_managers:
      try:
        self._process_managers[request.config_name].Stop()
      except RuntimeError as e:
        return cogrob_rosbag_msgs.srv.FinishRecordingResponse(
            success=False,
            error_info="Error stopping rosbag: {}".format(str(e)))
      self._process_managers.pop(request.config_name)
      return cogrob_rosbag_msgs.srv.FinishRecordingResponse(success=True)

    return cogrob_rosbag_msgs.srv.FinishRecordingResponse(
        success=False, error_info="Not running.")


def main(argv):
  FLAGS(argv)
  rospy.init_node("cogrob_rosbag_servicer")
  servicer = RosBagServicer()
  rospy.loginfo("Ready to serve.")
  rospy.spin()


if __name__ == "__main__":
  main(sys.argv)
