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

import rospy
import smach

import use_cogrob_workspace
from cogrob.executive.proto import smach_log_pb2
from cogrob.universal_logger import universal_logger
from cogrob.universal_logger.proto import archive_entry_pb2

class SmachLogger(object):

  def __init__(self):
    self._logger = universal_logger.UniversalLogger("executive/smach")


  def _LogSmachLog(self, container_id, event, active_states=None, outcome=None):
    log_entry = archive_entry_pb2.ArchiveEntry()
    smach_log = log_entry.smach_log

    smach_log.container_id.extend(container_id)

    smach_log.event = event

    if active_states is not None:
      smach_log.active_states.extend(active_states)

    if outcome is not None:
      smach_log.outcome = outcome

    self._logger.Log(log_entry)


  def _GetStartCallback(self, container_id):
    def Callback(userdata, initial_states):
      del userdata
      self._LogSmachLog(container_id, smach_log_pb2.SmachLog.START_EVENT,
                        active_states=initial_states)
    return Callback


  def _GetTransitionCallback(self, container_id):
    def Callback(userdata, active_states):
      del userdata
      self._LogSmachLog(container_id, smach_log_pb2.SmachLog.TRANSITION_EVENT,
                        active_states=active_states)
    return Callback


  def _GetTerminationCallback(self, container_id):
    def Callback(userdata, terminal_states, outcome):
      del userdata
      self._LogSmachLog(container_id, smach_log_pb2.SmachLog.TERMINATION_EVENT,
                        active_states=terminal_states, outcome=outcome)
    return Callback


  def RegisterSmachContainer(self, current_sm, path=None):
    if path is None:
      path = ["ROOT"]
    rospy.loginfo("Registering {}".format("/".join(path)))
    current_sm.register_start_cb(self._GetStartCallback(path))
    current_sm.register_transition_cb(self._GetTransitionCallback(path))
    current_sm.register_termination_cb(self._GetTerminationCallback(path))
    for child_name, child_state in current_sm.get_children().items():
      if isinstance(child_state, smach.Container):
        self.RegisterSmachContainer(child_state, path + [child_name])
