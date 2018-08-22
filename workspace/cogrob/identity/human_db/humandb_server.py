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

from absl import flags
from absl import logging
import concurrent.futures
import grpc
import sys

from cogrob.identity.human_db import humandb_model
from cogrob.identity.human_db.proto import humandb_record_pb2
from cogrob.identity.human_db.proto import humandb_service_pb2
from cogrob.identity.human_db.proto import humandb_service_pb2_grpc
import util.wait_to_terminate

FLAGS = flags.FLAGS


class HumanDbServer(humandb_service_pb2_grpc.HumanDbServiceServicer):
  def __init__(self, model):
    self._model = model


  def Query(self, request, context):
    response = humandb_service_pb2.QueryResponse()

    if request.HasField("human_uuid"):
      result = self._model.GetHumanRecord(request.human_uuid)
      if result:
        response.records.add().CopyFrom(result)
    elif request.HasField("label"):
      result = self._model.GetHumanRecordWithLabel(request.label)
      if result:
        response.records.add().CopyFrom(result)
    elif request.HasField("facedb_uuid"):
      humandb_uuids = (
          self._model.GetHumanUuidsWithFaceDbUuid(request.facedb_uuid))
      for humandb_uuid in humandb_uuids:
        result = self._model.GetHumanRecord(humandb_uuid)
        if result:
          response.records.add().CopyFrom(result)

    return response


  def CreateOrAppend(self, request, context):
    response = humandb_service_pb2.CreateOrAppendResponse()
    new_uuid = self._model.NewOrAppendUpdate(request.record)
    response.result.CopyFrom(self._model.GetHumanRecord(new_uuid))
    return response


def main(argv):
  FLAGS(argv)
  server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))

  model = humandb_model.HumanDbModel()
  humandb_server_instance = HumanDbServer(model=model)

  humandb_service_pb2_grpc.add_HumanDbServiceServicer_to_server(
      humandb_server_instance, server)
  server.add_insecure_port('[::]:7010')
  server.start()

  logging.info("HumanDB server is running.")
  util.wait_to_terminate.WaitToTerminate()
  server.stop(1)


if __name__ == "__main__":
  main(sys.argv)
