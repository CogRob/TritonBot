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
import google.protobuf.internal.decoder as protobuf_decoder
import google.protobuf.internal.encoder as protobuf_encoder
import gzip
import math
import os
import os.path
import socket
import threading
import time
import uuid

import cogrob.universal_logger.proto.archive_header_pb2 as archive_header_pb2
import cogrob.universal_logger.proto.archive_entry_pb2 as archive_entry_pb2

FLAGS = flags.FLAGS
flags.DEFINE_string("universal_logger_prefix", "/home/cogrob_log",
                     "Root directory for universal_logger, absolute path.")
flags.DEFINE_integer("universal_logger_file_max_bytes", 1073741824,
                      "Max size for a single universal_logger file.",
                      lower_bound=1024)

_ProtoBufDecodeVarint = protobuf_decoder._DecodeVarint
_ProtoBufEncodeVarint = protobuf_encoder._EncodeVarint

class UniversalLogger(object):

  def __init__(self, log_namespace, source_id=None, metadata=None,
               rewrite_uuid=False, rewrite_timestamp=True, gzip_enabled=True):
    self._file_date_packed32 = 0
    self._rewrite_uuid = rewrite_uuid
    self._rewrite_timestamp = rewrite_timestamp
    self._path_prefix = FLAGS.universal_logger_prefix
    self._mutex = threading.Lock()
    self._gzip_enabled = gzip_enabled
    self._log_namespace = log_namespace
    self._file_max_bytes = FLAGS.universal_logger_file_max_bytes
    if source_id:
      self._source_id = source_id
    else:
      self._source_id = socket.gethostname()
    self._log_namespace = log_namespace

    # These fields will be overwritten by CreateNewFile() function.
    self._ostream = None
    self._last_date_string = ""

    self._archive_header = archive_header_pb2.ArchiveHeader()
    self._archive_header.log_namespace = self._log_namespace
    self._archive_header.source_id = self._source_id
    if metadata:
      self._archive_header.general_log.CopyFrom(metadata)
    else:
      self._archive_header.general_log.SetInParent()


  def _GetDateString(self):
    return time.strftime("%Y/%m/%d")


  def _GetCurrentTimestampSec(self):
    return int(math.floor(time.time()))


  def _WriteToOstream(self, proto):
    if not self._ostream:
      self._CreateNewFile()
    binary_proto = proto.SerializeToString()
    _ProtoBufEncodeVarint(self._ostream.write, len(binary_proto), True)
    self._ostream.write(binary_proto)

    # TODO(shengye): Improve the performance if necessary.
    self._ostream.flush()


  def _CreateNewFile(self):
    if (self._ostream):
      self._ostream.close()
      self._ostream = None
    self._last_date_string = self._GetDateString()
    file_dir = os.path.join(self._path_prefix, self._source_id,
                            self._last_date_string, self._log_namespace)
    try:
      os.makedirs(file_dir)
    except OSError:
      if not os.path.isdir(file_dir):
        raise
    filepath = os.path.join(file_dir,
                            str(self._GetCurrentTimestampSec()) + ".pb")
    if (self._gzip_enabled):
      filepath += ".gz"
      self._ostream = gzip.open(filepath, "wb")
    else:
      self._ostream = open(filepath, "wb")
    self._archive_header.create_timestamp_sec = self._GetCurrentTimestampSec()
    self._WriteToOstream(self._archive_header)


  def Close(self):
    if self._ostream:
      self._ostream.close()


  def Log(self, proto):
    self._mutex.acquire()
    try:
      if not self._rewrite_uuid and not self._rewrite_timestamp:
        entry = proto
      else:
        entry = archive_entry_pb2.ArchiveEntry()
        entry.CopyFrom(proto)

      if self._rewrite_uuid:
        msbits = 0
        lsbits = 0
        uuid_bytes = uuid.uuid4().bytes
        for i in range(8):
          msbits = (msbits << 8) | uuid_bytes[i]
          lsbits = (lsbits << 8) | uuid_bytes[i + 8]

      if self._rewrite_timestamp:
        current_timestamp = time.time()
        entry.timestamp.seconds = int(math.floor(current_timestamp))
        entry.timestamp.nanos = int(
            (current_timestamp - entry.timestamp.seconds) * 1000000000)

      if self._GetDateString() != self._last_date_string:
        self._CreateNewFile()

      self._WriteToOstream(entry)
    finally:
      self._mutex.release()
