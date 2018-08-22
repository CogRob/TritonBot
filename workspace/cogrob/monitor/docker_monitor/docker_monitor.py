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

from absl import logging
from absl import flags
from cogrob.monitor.proto import docker_stats_pb2
from cogrob.universal_logger import universal_logger
from cogrob.universal_logger.proto import archive_entry_pb2
import datetime
import dateutil.parser
import docker
import time
import math

FLAGS = flags.FLAGS

flags.DEFINE_integer("log_all_interval", 60,
                     "The minimal interval of logging all containers.")

class DockerLogger(object):

  def __init__(self):
    self._logger = universal_logger.UniversalLogger("infrastructure/docker")
    self._docker_client = docker.from_env()


  @staticmethod
  def DockerStatsToProto(stats_dict):
    result = docker_stats_pb2.DockerContainerStats()

    read_time = dateutil.parser.parse(stats_dict["read"])
    read_timestamp = (read_time - datetime.datetime(
        1970, 1, 1, tzinfo=read_time.tzinfo)).total_seconds()

    preread_time = dateutil.parser.parse(stats_dict["preread"])
    preread_timestamp = (preread_time - datetime.datetime(
        1970, 1, 1, tzinfo=preread_time.tzinfo)).total_seconds()

    result.timestamp.seconds = int(math.floor(read_timestamp))
    result.timestamp.nanos = int(
        (read_timestamp - result.timestamp.seconds) * 1e9)

    result.name = stats_dict["name"]
    result.container_id = stats_dict["id"]

    if "memory_stats" in stats_dict and "usage" in stats_dict["memory_stats"]:
      result.memory_usage = stats_dict["memory_stats"]["usage"]

    if "cpu_stats" in stats_dict and "cpu_usage" in stats_dict["cpu_stats"]:
      result.cpu_total_usage = (
          stats_dict["cpu_stats"]["cpu_usage"]["total_usage"])
      result.cpu_usage_kernel = (
          stats_dict["cpu_stats"]["cpu_usage"]["usage_in_kernelmode"])
      result.cpu_usage_user = (
          stats_dict["cpu_stats"]["cpu_usage"]["usage_in_usermode"])

    if "pids_stats" in stats_dict and "current" in stats_dict["pids_stats"]:
      result.current_pids_stats = stats_dict["pids_stats"]["current"]

    if ("cpu_stats" in stats_dict and "cpu_usage" in stats_dict["cpu_stats"]
        and "precpu_stats" in stats_dict
        and "cpu_usage" in stats_dict["precpu_stats"]):
      result.cpu_nanos_per_second = (float(
          stats_dict["cpu_stats"]["cpu_usage"]["total_usage"] -
          stats_dict["precpu_stats"]["cpu_usage"]["total_usage"])
          / (read_timestamp - preread_timestamp))
      result.cpu_percentage = (float(
          stats_dict["cpu_stats"]["cpu_usage"]["total_usage"] -
          stats_dict["precpu_stats"]["cpu_usage"]["total_usage"]) /
          (stats_dict["cpu_stats"]["system_cpu_usage"] -
          stats_dict["precpu_stats"]["system_cpu_usage"]))

    return result


  def LoopLogDockerStats(self):
    last_log_start_time = 0
    while True:
      while time.time() - last_log_start_time < FLAGS.log_all_interval:
        time.sleep(1)
      last_log_start_time = time.time()
      try:
        for container in self._docker_client.containers.list():
          stats = container.stats(stream=False, decode=True)
          try:
            stats_pb = self.DockerStatsToProto(stats)
          except KeyError as e:
            logging.error("DockerStatsToProto error: %s", str(e))
            continue

          log_entry = archive_entry_pb2.ArchiveEntry()
          log_entry.docker_container_stats.CopyFrom(stats_pb)

          logging.info("Log: %s", str(log_entry))
          self._logger.Log(log_entry)
      except docker.errors.DockerException as e:
        logging.error("Docker error: %s", str(e))
