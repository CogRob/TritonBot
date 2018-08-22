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

import google.protobuf.internal.decoder as protobuf_decoder
import google.protobuf.internal.encoder as protobuf_encoder

_ProtoBufDecodeVarint = protobuf_decoder._DecodeVarint
_ProtoBufEncodeVarint = protobuf_encoder._EncodeVarint


def ReadVarint(istream):
  buff = istream.read(1)
  if buff == b'':
    return 0

  while (ord(buff[-1]) & 0x80) >> 7 == 1:  # Continues if MSB is 1.
    next_byte = istream.read(1)
    if next_byte == b'':
      raise EOFError('Unexpected EOF.')
    buff += next_byte

  return _ProtoBufDecodeVarint(buff, 0)[0]


def WriteToOstream(ostream, proto):
  binary_proto = proto.SerializeToString()
  _ProtoBufEncodeVarint(ostream.write, len(binary_proto), True)
  ostream.write(binary_proto)


def GetNextRawBuf(istream):
  size = ReadVarint(istream)
  if not size:
    return None
  buf = istream.read(size)
  if len(buf) != size:
    raise EOFError('Unexpected EOF.')
  return buf


def GetNextProto(istream, proto_type):
  buf = GetNextRawBuf(istream)
  if buf is None:
    return None
  result = proto_type()
  result.ParseFromString(buf)
  return result
