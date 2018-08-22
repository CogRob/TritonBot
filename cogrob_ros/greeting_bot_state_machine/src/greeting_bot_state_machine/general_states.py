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

import smach
import time

class WaitTimeState(smach.State):
  """
  Waits for a few (wait_time_sec) seconds before transfer to next state.
  """

  def __init__(self, wait_time_sec=3):
    smach.State.__init__(self, outcomes=["next"])
    self._wait_time_sec = wait_time_sec


  def execute(self, userdata):
    del userdata
    time.sleep(self._wait_time_sec)
    return "next"


class ResetRetryCounter(smach.State):
  """
  Resets retry counter, name of the counter is given by retry_counter_name.
  """

  def __init__(self, retry_counter_name, reset_value):
    assert isinstance(retry_counter_name, str)
    assert isinstance(reset_value, int)
    smach.State.__init__(
        self, outcomes=["next"], output_keys=[retry_counter_name])
    self._retry_counter_name = retry_counter_name
    self._reset_value = reset_value


  def execute(self, userdata):
    setattr(userdata, self._retry_counter_name, self._reset_value)
    return "next"


class DecreaseAndTestRetry(smach.State):
  """
  Decreases retry counter, name of the counter is given by retry_counter_name.
  When reaches 0, give up retrying.
  """

  def __init__(self, retry_counter_name):
    assert isinstance(retry_counter_name, str)
    smach.State.__init__(
        self, outcomes=["continue", "give_up"],
        input_keys=[retry_counter_name], output_keys=[retry_counter_name])
    self._retry_counter_name = retry_counter_name


  def execute(self, userdata):
    if getattr(userdata, self._retry_counter_name) > 0:
      setattr(userdata, self._retry_counter_name,
              getattr(userdata, self._retry_counter_name) - 1)
      return "continue"
    return "give_up"


class SetVariables(smach.State):
  """
  Sets userdata.
  """

  def __init__(self, var_dict):
    assert isinstance(var_dict, dict)
    for key in var_dict.keys():
      assert isinstance(key, str)
    smach.State.__init__(self, outcomes=["next"], output_keys=var_dict.keys())
    self._var_dict = dict


  def execute(self, userdata):
    for k, v in self._var_dict:
      setattr(userdata, k, v)
    return "next"


class VariableSwitch(smach.State):
  """
  Tests userdata.
  """

  def __init__(self, var_name, var_values):
    assert isinstance(var_name, str)
    assert isinstance(var_values, list)
    for value in var_values.keys():
      assert isinstance(var_values, str)
    smach.State.__init__(
        self, outcomes=var_values+["_other"], input_keys=[var_name])
    self._var_name = var_name
    self._var_values = var_values


  def execute(self, userdata):
    if getattr(userdata, self._var_name) in self._var_values:
      return getattr(userdata, self._var_name)
    return "_other"


class BypassState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])


  def execute(self, userdata):
    return "next"
