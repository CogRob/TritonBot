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
import os.path
import threading
import time

from cogrob.identity.human_db.proto import humandb_record_pb2
from util import uuid_util
from util import proto_file_util

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "humandb_storage_filename", "/home/cogrob_local/humandb_storage.pb",
    "HumanDB filepath.")

HumanDbRecord = humandb_record_pb2.HumanDbRecord

IntToUuid = uuid_util.IntToUuid
UuidToInt = uuid_util.UuidToInt


class HumanDbModel(object):

  def __init__(self, db_filename=None):
    if db_filename is None:
      self._db_filename = FLAGS.humandb_storage_filename
    else:
      self._db_filename = db_filename
    self._ClearModel()
    self._filelock = threading.Lock()
    self._LoadFromFile()


  def _ClearModel(self):
    self._records = {}  # Dict, UuidInInt->HumanDbRecord
    self._alias_map = {}  # Dict, Int->Int, AliasUUID->RealUUID
    self._label_map = {}  # Dict, Str->Int, Label->RealUUID
    self._faceuuid_map = {}  # Dict, Int->Set[Int], FaceUUID->[HumanUUID]


  def GetAllRecords(self):
    return self._records.values()


  def _WriteToFile(self):
    with self._filelock:
      with open(self._db_filename, "wb") as fp:
        for proto in self._records.values():
          proto_file_util.WriteToOstream(fp, proto)


  def _UpdateRecordCache(self, human_uuid):
    human_uuid_int = UuidToInt(human_uuid)
    if human_uuid_int in self._alias_map:
      human_uuid_int = self._alias_map[human_uuid_int]
    record = self._records[human_uuid_int]

    # Process human_uuid_aliases field.
    self._alias_map[human_uuid_int] = human_uuid_int
    for human_uuid_alias in record.human_uuid_aliases:
      uuid_alias_int = UuidToInt(human_uuid_alias)
      if (uuid_alias_int in self._alias_map
          and self._alias_map[uuid_alias_int] != human_uuid_int):
        logging.error("Duplicated HumanUuidAlias, ignore %s",
                      str(human_uuid_alias))
      else:
        self._alias_map[uuid_alias_int] = human_uuid_int

    # Process human_labels.
    for label in record.human_labels:
      if label in self._label_map and self._label_map[label] != human_uuid_int:
        logging.error("Duplicated label, ignore %s", label)
      else:
        self._label_map[label] = human_uuid_int

    # Process facedb_uuids.
    for facedb_uuid in record.facedb_uuids:
      facedb_uuid_int = UuidToInt(facedb_uuid)
      if facedb_uuid_int not in self._faceuuid_map:
        self._faceuuid_map[facedb_uuid_int] = set()
      self._faceuuid_map[facedb_uuid_int].add(human_uuid_int)


  def _LoadFromFile(self):
    self._ClearModel()

    # First, load self._records.
    need_repair = False
    if os.path.isfile(self._db_filename):
      with self._filelock:
        with open(self._db_filename, "rb") as fp:
          while True:
            try:
              record = proto_file_util.GetNextProto(fp, HumanDbRecord)
              uuid_int = UuidToInt(record.human_uuid)
              if uuid_int in self._records:
                logging.error("Duplicated HumanUuid, merging %s",
                              str(record.human_uuid))
                self._MergeHumanDbRecord(self._records[uuid_int], record)
                self._records[uuid_int].modified_timestamp.add(int(time.time()))
              else:
                self._records[uuid_int] = record
            except:
              need_repair = True
            if record is None:
              break

    # Repairs the model if necessary.
    if need_repair:
      self._WriteToFile()

    # Update local caches.
    for record in self._records.values():
      self._UpdateRecordCache(record.human_uuid)


  def _GetHumanRecord(self, uuid):
    # Gets a human record form UUID.
    human_uuid_int = UuidToInt(uuid)
    if human_uuid_int in self._alias_map:
      human_uuid_int = self._alias_map[human_uuid_int]
    if human_uuid_int in self._records:
      return self._records[human_uuid_int]
    else:
      return None


  def GetHumanRecord(self, uuid):
    result = self._GetHumanRecord(uuid)
    if result is not None:
      ret_result = HumanDbRecord()
      ret_result.CopyFrom(result)
      return ret_result
    else:
      return None


  def GetHumanRecordWithLabel(self, label):
    # Gets a human record form label.
    if label not in self._label_map:
      return None
    return self.GetHumanRecord(IntToUuid(self._label_map[label]))


  def GetHumanUuidsWithFaceDbUuid(self, facedb_uuid):
    result = []
    facedb_uuid_int = UuidToInt(facedb_uuid)

    if facedb_uuid_int in self._faceuuid_map:
      result = [IntToUuid(human_uuid_int) for human_uuid_int
              in self._faceuuid_map[facedb_uuid_int]]

    return result


  def ForceRefresh(self):
    self._WriteToFile()
    self._LoadFromFile()


  def _MergeHumanDbRecord(self, dest, src):
    # Merge the aliases.
    all_uuid_aliases = map(
        IntToUuid, set(map(UuidToInt, dest.human_uuid_aliases)
                       + map(UuidToInt, src.human_uuid_aliases)))
    dest.ClearField("human_uuid_aliases")
    dest.human_uuid_aliases.extend(all_uuid_aliases)

    # Merge the labels.
    all_human_labels = list(
        set(dest.human_labels).union(src.human_labels))
    dest.ClearField("human_labels")
    dest.human_labels.extend(all_human_labels)

    # Add modified timestamp
    dest.modified_timestamp.append(int(time.time()))
    all_timestamps = sorted(dest.modified_timestamp)
    dest.ClearField("modified_timestamp")
    dest.modified_timestamp.extend(all_timestamps)

    # Merge the nicknames.
    all_nicknames = list(
        set(dest.nicknames).union(src.nicknames))
    dest.ClearField("nicknames")
    dest.nicknames.extend(all_nicknames)

    # Merge the aliases.
    all_facedb_uuids = map(
        IntToUuid, set(map(UuidToInt, dest.facedb_uuids)
                       + map(UuidToInt, src.facedb_uuids)))
    dest.ClearField("facedb_uuids")
    dest.facedb_uuids.extend(all_facedb_uuids)


  def ResolveToExistingUuids(self, record):
    result = set([])
    # First, try to resolve this record to a existing record.
    if record.HasField("human_uuid"):
      human_uuid_int = UuidToInt(record.human_uuid)
      if human_uuid_int in self._alias_map:
        result.add(self._alias_map[human_uuid_int])
    for label in record.human_labels:
      if label in self._label_map:
        result.add(self._label_map[label])
    return result


  def NewOrAppendUpdate(self, record):
    existing_record = None

    existing_int_uuids = self.ResolveToExistingUuids(record)

    if len(existing_int_uuids) == 0:
      if record.HasField("human_uuid"):
        human_uuid_int = UuidToInt(record.human_uuid)
      else:
        human_uuid_int = UuidToInt(uuid_util.GetNewUuid())
      self._records[human_uuid_int] = HumanDbRecord()
      self._records[human_uuid_int].created_timestamp = int(time.time())
      existing_record = self._records[human_uuid_int]
      existing_record.human_uuid.CopyFrom(IntToUuid(human_uuid_int))
    elif len(existing_int_uuids) == 1:
      existing_record = self._GetHumanRecord(
          IntToUuid(list(existing_int_uuids)[0]))

    self._MergeHumanDbRecord(existing_record, record)

    self._UpdateRecordCache(existing_record.human_uuid)

    # This could be a performance bottle neck, then we can do periodic flush.
    self._WriteToFile()

    return existing_record.human_uuid


  def FullUpdate(self, record):
    human_uuid_int = UuidToInt(record.human_uuid)
    self._records[human_uuid_int] = record
    # TODO(shengye): Improve this if performance is a problem, we will have to
    # remove old labels, etc.
    self.ForceRefresh()
