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
import os.path
import pandas
import threading

import cogrob.perception.face_db.proto.storage_pb2 as facedb_storage_pb2
from util import uuid_util
from util import proto_file_util

FLAGS = flags.FLAGS
flags.DEFINE_string("facedb_storage_filename",
                    "/home/cogrob_local/facedb_storage.pb", "FaceDB filepath.")

class FaceDbStorage(object):

  def __init__(self, dbfile=None):
    if dbfile is None:
      self._dbfile = FLAGS.facedb_storage_filename
    else:
      self._dbfile = dbfile
    self._lock = threading.Lock()


  def GetAllRecords(self):
    # Returns a list of KnnRecord.
    knn_records = []

    if not os.path.isfile(self._dbfile):
      # Returns an empty DataFrame is there is not db file.
      return knn_records

    needs_repair = False

    with open(self._dbfile, "rb") as istream:
      while True:
        try:
          knn_record = proto_file_util.GetNextProto(
              istream, facedb_storage_pb2.KnnRecord)
          if knn_record is None:
            break
          knn_records.append(knn_record)
        except:
          # If reading from the file failed, we set needs_repair.
          needs_repair = True
          break

    if needs_repair:
      # Repairs the DB.
      with self._lock:
        with open(self._dbfile, "wb") as ostream:
          for record in knn_records:
            proto_file_util.WriteToOstream(ostream, record)

    return knn_records


  def GetDataFrame(self):
    # Constructs and returns a pandas DataFrame.
    df_columns = ['label'] + ['embedding_{}'.format(x) for x in range(1, 129)]
    knn_records = self.GetAllRecords()
    df_src = []
    for record in knn_records:
      entry = ((str(uuid_util.UuidToInt(record.facedb_uuid)), )
               + tuple(record.embedding))
      df_src.append(entry)
    df = pandas.DataFrame(df_src, columns=df_columns)
    return df


  def AddRecord(self, record):
    # record is a single KnnRecord
    with self._lock:
      with open(self._dbfile, "ab") as ostream:
        proto_file_util.WriteToOstream(ostream, record)


  def AddRecords(self, records):
    # records is a list of KnnRecord
    with self._lock:
      with open(self._dbfile, "ab") as ostream:
        for record in records:
          proto_file_util.WriteToOstream(ostream, record)
