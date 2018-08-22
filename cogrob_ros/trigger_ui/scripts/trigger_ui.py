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


import flask
import rospy
import std_msgs.msg

Flask = flask.Flask
flask_app = Flask(__name__)

global_holder = {
  "ros_pub": None,
}

@flask_app.route("/", methods=['GET', 'POST'])
def IndexPage():
  request = flask.request
  if request.method == "POST":
    if "trigger_cmd" in request.form:
      cmd = request.form["trigger_cmd"]
      msg = std_msgs.msg.String(data=cmd)
      global_holder["ros_pub"].publish(msg)
  return flask.render_template("index.html")


def main():
  rospy.init_node("trigger_ui")
  global global_holder
  global_holder["ros_pub"] = rospy.Publisher(
      "/cogrob/trigger_cmd", std_msgs.msg.String, queue_size=1)
  flask_app.run(host="0.0.0.0", port=7014)


if __name__ == '__main__':
  main()
