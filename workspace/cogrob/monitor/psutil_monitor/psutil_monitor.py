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
from cogrob.monitor.proto import psutil_stats_pb2
from cogrob.universal_logger import universal_logger
from cogrob.universal_logger.proto import archive_entry_pb2
import time
import math
import psutil

FLAGS = flags.FLAGS

flags.DEFINE_integer("log_all_interval", 60,
                     "The minimal interval of logging all containers.")

class PsUtilLogger(object):

  def __init__(self):
    self._logger = universal_logger.UniversalLogger(
        "infrastructure/psutil_stats")


  @staticmethod
  def _PopulateTimestamp(pb_field, timestamp=None):
    if timestamp is None:
      timestamp = time.time()
    pb_field.seconds = int(math.floor(timestamp))
    pb_field.nanos = int((timestamp - pb_field.seconds) * 1e9)


  @staticmethod
  def _CopyFieldIfExist(pb_inst, py_inst, pb_field_name, py_field_name=None):
    if py_field_name is None:
      py_field_name = pb_field_name
    if hasattr(py_inst, py_field_name):
      val = getattr(py_inst, py_field_name)
      if val is not None:
        setattr(pb_inst, pb_field_name, val)


  @staticmethod
  def _CopyFieldsIfExist(pb_inst, py_inst, fields):
    # fields is either a list, or a map from pb_field_name to py_field_name
    if isinstance(fields, list):
      fields = {f : f for f in fields}
    for pb_field_name, py_field_name in fields.items():
      PsUtilLogger._CopyFieldIfExist(pb_inst, py_inst, pb_field_name,
                                     py_field_name)


  @staticmethod
  def GeneratePsUtilStatsProto():
    sample_period = 1.0;

    result = psutil_stats_pb2.PsUtilStats()

    # Time to begin collection
    PsUtilLogger._PopulateTimestamp(result.collect_begin_time)

    cpu_times_fields = [
        "user", "system", "idle", "nice", "iowait", "irq", "softirq", "steal",
        "guest", "guest_nice", "interrupt", "dpc"]

    # cpu_times
    PsUtilLogger._CopyFieldsIfExist(
        result.cpu_times, psutil.cpu_times(percpu=False), cpu_times_fields)

    # cpu_times_per_cpu
    for per_cpu_cpu_times in psutil.cpu_times(percpu=True):
      PsUtilLogger._CopyFieldsIfExist(
          result.cpu_times_per_cpu.add(), per_cpu_cpu_times, cpu_times_fields)

    # cpu_percent
    result.cpu_percent_interval = sample_period
    result.cpu_percent = psutil.cpu_percent(
        interval=result.cpu_percent_interval, percpu=False)

    # cpu_percent_per_cpu
    result.cpu_percent_per_cpu_interval = sample_period
    result.cpu_percent_per_cpu.extend(psutil.cpu_percent(
        interval=result.cpu_percent_per_cpu_interval, percpu=True))

    cpu_times_percent_fields = [
        "user", "system", "idle", "nice", "iowait", "irq", "softirq", "steal",
        "guest", "guest_nice", "interrupt", "dpc"]

    # cpu_times_percent
    result.cpu_times_percent_interval = sample_period
    PsUtilLogger._CopyFieldsIfExist(
        result.cpu_times_percent,
        psutil.cpu_times_percent(percpu=False,
                                 interval=result.cpu_times_percent_interval),
        cpu_times_percent_fields)

    # cpu_times_percent_per_cpu
    result.cpu_times_percent_per_cpu_interval = sample_period
    for per_cpu_cpu_times_percent in psutil.cpu_times_percent(
        percpu=True, interval=result.cpu_times_percent_per_cpu_interval):
      PsUtilLogger._CopyFieldsIfExist(
          result.cpu_times_percent_per_cpu.add(), per_cpu_cpu_times_percent,
          cpu_times_percent_fields)

    # cpu_count_physical
    result.cpu_count_physical = psutil.cpu_count(logical=False)

    # cpu_count_logical
    result.cpu_count_logical = psutil.cpu_count(logical=True)

    # cpu_stats
    PsUtilLogger._CopyFieldsIfExist(
        result.cpu_stats, psutil.cpu_stats(),
        ["ctx_switches", "interrupts", "soft_interrupts", "syscalls"])

    cpu_freq_fields_map = {
        "current_val": "current", "min_val": "min", "max_val": "max"}

    # cpu_freq
    PsUtilLogger._CopyFieldsIfExist(
        result.cpu_freq, psutil.cpu_freq(percpu=False), cpu_freq_fields_map)

    # cpu_freq_per_cpu
    for per_cpu_cpu_freq in psutil.cpu_freq(percpu=True):
      PsUtilLogger._CopyFieldsIfExist(
          result.cpu_freq_per_cpu.add(), per_cpu_cpu_freq, cpu_freq_fields_map)

    # virtual_memory
    virtual_memory_fields = [
        "total", "available", "used", "free", "active", "inactive", "buffers",
        "cached", "shared", "slab", "wired"]
    PsUtilLogger._CopyFieldsIfExist(
        result.virtual_memory, psutil.virtual_memory(), virtual_memory_fields)

    # swap_memory
    swap_memory_fields = ["total", "used", "free", "percent", "sin", "sout"]
    PsUtilLogger._CopyFieldsIfExist(
        result.swap_memory, psutil.swap_memory(), swap_memory_fields)

    disk_io_counters_fields = [
        "read_count", "write_count", "read_bytes", "write_bytes", "read_time",
        "write_time", "busy_time", "read_merged_count", "write_merged_count"]

    # disk_io_counters
    PsUtilLogger._CopyFieldsIfExist(
        result.disk_io_counters, psutil.disk_io_counters(perdisk=False),
        disk_io_counters_fields)

    # disk_io_counters_per_disk
    for disk_name, per_disk_disk_io_counters in (
        psutil.disk_io_counters(perdisk=True).items()):
      PsUtilLogger._CopyFieldsIfExist(
          result.disk_io_counters_per_disk[disk_name],
          per_disk_disk_io_counters,
          disk_io_counters_fields)

    net_io_counters_fields = [
        "bytes_sent", "bytes_recv", "packets_sent", "packets_recv", "errin",
        "errout", "dropin", "dropout"]

    # net_io_counters
    PsUtilLogger._CopyFieldsIfExist(
        result.net_io_counters, psutil.net_io_counters(pernic=False),
        net_io_counters_fields)

    # net_io_counters_per_nic
    for net_name, per_nic_net_io_counters in (
        psutil.net_io_counters(pernic=True).items()):
      PsUtilLogger._CopyFieldsIfExist(
          result.net_io_counters_per_nic[net_name],
          per_nic_net_io_counters,
          net_io_counters_fields)

    # boot_time
    PsUtilLogger._PopulateTimestamp(result.boot_time, psutil.boot_time())

    # Time to end collection
    PsUtilLogger._PopulateTimestamp(result.collect_end_time)
    return result


  def LoopLogPsUtilStats(self):
    last_log_start_time = 0
    while True:
      while time.time() - last_log_start_time < FLAGS.log_all_interval:
        time.sleep(1)
      last_log_start_time = time.time()

      stats_pb = self.GeneratePsUtilStatsProto()

      log_entry = archive_entry_pb2.ArchiveEntry()
      log_entry.psutil_stats.CopyFrom(stats_pb)

      logging.info("Log: %s", str(log_entry))
      self._logger.Log(log_entry)
