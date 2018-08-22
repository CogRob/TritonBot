#!/usr/bin/env python
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


import collections
import datetime
import json
import os
import re
import subprocess

ImageInfo = collections.namedtuple(
    "ImageInfo", ["name", "name_tagged_date", "path", "dep"])

email_content = ""
prepend_before = False

def PrintLog(log, also_prepend=False):
  global email_content
  global prepend_before
  record = "{}] {}\n\n".format(str(datetime.datetime.now()), str(log))
  email_content += record
  if also_prepend:
    if not prepend_before:
      prepend_before = True
      email_content = "=" * 80 + "\n" + email_content
      email_content = "=" * 80 + "\n" + email_content
    email_content = record + email_content
  print record,


def ConstructAllCommands():
  external_deps = set()
  all_images = []
  all_image_names = set()

  no_intdep_images = []
  all_dependents = {}

  root_path = os.path.dirname(os.path.realpath(__file__))
  for dir in os.listdir(root_path):
    expected_dockerfile = os.path.join(
        root_path, dir, "docker_image/Dockerfile")
    if not os.path.isfile(expected_dockerfile):
      continue

    dockerfile_lines = ""
    with open(expected_dockerfile) as fp:
      dockerfile_lines = fp.readlines()

    image_dep = None
    for line in dockerfile_lines:
      re_result = re.match("FROM\s+(\S+)", line)
      if re_result is not None:
        break
    if re_result is None:
      PrintLog("Cannot find dependency for {}.".format(dir))
    else:
      image_dep = re_result.group(1)

    image_name = "tritonbot.github.io/{}".format(dir)
    image_name_tagged_date = "tritonbot.github.io/{}:{}".format(
        dir, datetime.datetime.now().strftime("%Y%m%d"))
    image_root = os.path.join(root_path, dir, "docker_image")

    all_images.append(
        ImageInfo(name=image_name, name_tagged_date=image_name_tagged_date,
                  path=image_root, dep=image_dep))
    all_image_names.add(image_name)

  for image_info in all_images:
    if image_info.dep is not None and image_info.dep not in all_image_names:
      no_intdep_images.append(image_info)
    if image_info.dep in all_image_names:
      if image_info.dep not in all_dependents:
        all_dependents[image_info.dep] = []
      all_dependents[image_info.dep].append(image_info)

  commands = []
  all_pulled = set()
  for image in no_intdep_images:
    if image.dep not in all_pulled:
      commands.append("docker pull {}".format(image.dep))
      all_pulled.add(image.dep)

  bfs_queue = collections.deque()
  bfs_queue_pushed = set()

  for image in no_intdep_images:
    bfs_queue.append(image)
    bfs_queue_pushed.add(image.name)
    if image.name in all_dependents:
      for child_name in all_dependents[image.name]:
        if child_name not in bfs_queue_pushed:
          bfs_queue.append(child_name)
          bfs_queue_pushed.add(child_name)

  build_commands = []
  push_commands = []
  while len(bfs_queue) > 0:
    current_image = bfs_queue.popleft()
    build_commands.append("docker build --tag {} --tag {} {}".format(
        current_image.name, current_image.name_tagged_date, current_image.path))
    # The public version does not use a registry, so we do not push the image.
    # push_commands.append("docker push {}".format(current_image.name))
    # push_commands.append(
    #     "docker push {}".format(current_image.name_tagged_date))
    push_commands.append("docker rmi {}".format(current_image.name_tagged_date))
    if current_image.name in all_dependents:
      for child_name in all_dependents[current_image.name]:
        if child_name not in bfs_queue_pushed:
          bfs_queue.append(child_name)
          bfs_queue_pushed.add(child_name)

  commands += build_commands + push_commands
  return commands


if __name__ == "__main__":
  commands = ConstructAllCommands()
  failure = False
  for command in commands:
    PrintLog("-" * 80)
    PrintLog("Running: {}".format(command))
    try:
      cmd_result = subprocess.check_output(
          command, stderr=subprocess.STDOUT, shell=True)
      PrintLog(cmd_result)
    except Exception as e:
      if isinstance(e, subprocess.CalledProcessError):
        PrintLog(e.output, also_prepend=True)
      failure = True
      PrintLog("[!!FAIL!!] {} failed, exception: {}".format(command, str(e)),
               also_prepend=True)
