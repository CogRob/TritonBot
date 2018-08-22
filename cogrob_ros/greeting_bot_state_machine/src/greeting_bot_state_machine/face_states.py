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

import cogrob_face_msgs.srv as face_srvs
import rospy
import smach


class ClearEmbeddingCacheState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"])
    self.ClearEmbeddingCache = rospy.ServiceProxy(
        "/cogrob/clear_face_embedding_cache",
        face_srvs.ClearEmbeddingCache)


  def execute(self, userdata):
    self.ClearEmbeddingCache(face_srvs.ClearEmbeddingCacheRequest())
    return "next"


class TestEmbeddingSufficentState(smach.State):
  def __init__(self, sufficent_threshold=None):
    # If sufficent_threshold is None, will be decided by the server.
    smach.State.__init__(self, outcomes=["sufficent", "not_sufficent"])
    self.GetAvailableEmbeddingCount = rospy.ServiceProxy(
        "/cogrob/get_available_face_embedding_count",
        face_srvs.GetAvailableEmbeddingCount)
    self._sufficent_threshold = sufficent_threshold


  def execute(self, userdata):
    response = self.GetAvailableEmbeddingCount(
        face_srvs.GetAvailableEmbeddingCountRequest())
    if self._sufficent_threshold is None:
      if response.is_sufficient:
        return "sufficent"
      return "not_sufficent"
    else:
      if response.count >= self._sufficent_threshold:
        return "sufficent"
      return "not_sufficent"


class TrainOnFaceWithNameState(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=["next"], input_keys=["human_name"])
    self.AddNewHumanWithRecentFace = rospy.ServiceProxy(
        "/cogrob/add_new_human_with_recent_face",
        face_srvs.AddNewHumanWithRecentFace)


  def execute(self, userdata):
    assert userdata.human_name is not None
    assert isinstance(userdata.human_name, basestring)
    self.AddNewHumanWithRecentFace(
        face_srvs.AddNewHumanWithRecentFaceRequest(
        human_labels=[], nicknames=[userdata.human_name]))
    return "next"
