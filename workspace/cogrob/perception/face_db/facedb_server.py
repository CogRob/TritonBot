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

import numpy
import sklearn.neighbors

from cogrob.perception.face_db import facedb_storage
from cogrob.perception.face_db.proto import facedb_service_pb2
from cogrob.perception.face_db.proto import facedb_service_pb2_grpc
import cogrob.perception.face_db.proto.storage_pb2 as facedb_storage_pb2
from util import uuid_util
import util.wait_to_terminate

FLAGS = flags.FLAGS
flags.DEFINE_float("knn_distance_threshold", 0.5,
                   "Distance threshold for kNN.")

class ClassifierManager(object):

  def __init__(self, storage):
    self._storage = storage
    self._knn_cls = None
    self._dist_threshold = FLAGS.knn_distance_threshold
    self._ReTrain()


  def _ReTrain(self):
    # TODO(shengye): Profile the performance of this function, export to
    # UniversalLogger.
    df = self._storage.GetDataFrame()
    try:
      knn_cls = sklearn.neighbors.KNeighborsClassifier()
      knn_cls.fit(df.drop('label', axis=1), df['label'])
      self._knn_cls = knn_cls
    except Exception as err:
      logging.error("Can not train kNN model, error: {}".format(str(err)))


  def Predict(self, embedding):
    recog_uuid = uuid_util.IntToUuid(0)

    # Makes a copy of _knn_cls, so that it won't be replaced by trainning.
    knn_cls = self._knn_cls

    if knn_cls is not None:
      # Only predict if there is a classifier.
      recog_label = knn_cls.predict([embedding])[0]

      # Caculates the distance and compares with the threshold.
      dist = knn_cls.kneighbors([embedding], n_neighbors=1,
                                return_distance=True)[0][0][0]
      if dist < self._dist_threshold:
        recog_uuid = uuid_util.IntToUuid(int(recog_label))

    return recog_uuid


  def Train(self, embeddings):
    # TODO(shengye): Filter outliers.
    # TODO(shengye): Optionally allow join existing Face-UUID groups.

    new_uuid = uuid_util.GetNewUuid()

    records = []
    for embedding in embeddings:
      record = facedb_storage_pb2.KnnRecord()
      record.embedding.extend(embedding)
      record.facedb_uuid.CopyFrom(new_uuid)
      records.append(record)
    self._storage.AddRecords(records)

    self._ReTrain()

    return new_uuid


class FaceDbServer(facedb_service_pb2_grpc.FaceDbServiceServicer):
  def __init__(self, cls_manager):
    self._cls_manager = cls_manager


  def Query(self, request, context):
    response = facedb_service_pb2.QueryResponse()
    embd = list(request.embedding)
    recog_uuid = self._cls_manager.Predict(embd)
    response.facedb_uuid.CopyFrom(recog_uuid)
    return response


  def Register(self, request, context):
    response = facedb_service_pb2.RegisterResponse()
    embeddings = [x.embedding for x in request.embeddings]
    new_uuid = self._cls_manager.Train(embeddings)
    response.facedb_uuid.CopyFrom(new_uuid)
    return response


def main(argv):
  FLAGS(argv)
  logging.set_verbosity(logging.INFO)
  server = grpc.server(concurrent.futures.ThreadPoolExecutor(max_workers=10))

  facedb_storage_inst = facedb_storage.FaceDbStorage()
  classifier_manager_inst = ClassifierManager(storage=facedb_storage_inst)
  facedb_server_instance = FaceDbServer(cls_manager=classifier_manager_inst)

  facedb_service_pb2_grpc.add_FaceDbServiceServicer_to_server(
      facedb_server_instance, server)
  server.add_insecure_port('[::]:7009')
  server.start()

  logging.info("FaceDB server is running.")
  util.wait_to_terminate.WaitToTerminate()
  server.stop(1)


if __name__ == "__main__":
  main(sys.argv)
