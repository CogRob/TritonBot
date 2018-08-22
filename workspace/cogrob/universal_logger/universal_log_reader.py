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
import gzip
import sys

import cogrob.universal_logger.proto.archive_header_pb2 as archive_header_pb2
import cogrob.universal_logger.proto.archive_entry_pb2 as archive_entry_pb2

FLAGS = flags.FLAGS

_ProtoBufDecodeVarint = protobuf_decoder._DecodeVarint

class UniversalLogReader(object):

  def __init__(self, filename):
    self._filename_ = filename
    if filename.endswith(".gz"):
      self._istream = gzip.open(filename, "rb")
    else:
      self._istream = open(filename, "rb")


  def _ReadVarint(self):
    buff = self._istream.read(1)
    if buff == b'':
      return 0

    while (ord(buff[-1]) & 0x80) >> 7 == 1:  # Continues if MSB is 1.
      next_byte = self._istream.read(1)
      if next_byte == b'':
        raise EOFError('Unexpected EOF.')
      buff += next_byte

    return _ProtoBufDecodeVarint(buff, 0)[0]


  def _GetNextObject(self):
    size = self._ReadVarint()
    if not size:
      return None
    return self._istream.read(size)


  def Read(self):
    header = archive_header_pb2.ArchiveHeader()
    header.ParseFromString(self._GetNextObject())
    yield header
    while True:
      entry = archive_entry_pb2.ArchiveEntry()
      try:
        buf = self._GetNextObject()
      except (EOFError, IOError) as e:
        break
      if buf is None:
        break
      entry.ParseFromString(buf)
      yield entry


  def Close(self):
    self._istream.close()


def main(argv):
  try:
    argv = FLAGS(argv)  # parse flags
  except flags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  for filename in argv[1:]:
    reader = UniversalLogReader(filename)
    for proto in reader.Read():
      print proto
      print "----------------------------------------"
    reader.Close()


if __name__ == "__main__":
  main(sys.argv)
