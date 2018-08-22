#! /usr/bin/env python
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

import actionlib
import grpc
import librosa.core
import numpy as np
import pyaudio
import rospy
import struct
import time
import threading

import speak_text_msgs.msg as speak_text_msgs
import proto.tts_service_pb2 as tts_service_pb2
import proto.tts_service_pb2_grpc as tts_service_pb2_grpc

class SpeakTextActionServer(object):

    class PaStreamWrapper(object):
        def __init__(self, pa_stream):
            self._pa_stream = pa_stream
        def __enter__(self):
            return self._pa_stream
        def __exit__(self, type, value, traceback):
            self._pa_stream.close()


    def __init__(self, name):
        channel = grpc.insecure_channel('localhost:7007')
        self._stub = tts_service_pb2_grpc.TtsServiceStub(channel)
        self._pa_inst = pyaudio.PyAudio()

        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(
            self._action_name,
            speak_text_msgs.SpeakTextAction,
            execute_cb=self.execute_callback,
            auto_start = False)
        self._action_server.start()


    def execute_callback(self, goal):
        result = speak_text_msgs.SpeakTextResult()

        DEV_PRIORITY_LIST = ["Creative", "USB", ""]

        selected_dev_id = None

        dev_count = self._pa_inst.get_device_count()
        for device_candidate in DEV_PRIORITY_LIST:
          for i in range(0, dev_count):
            if self._pa_inst.get_device_info_by_index(i).get(
                'maxOutputChannels') > 0:
              current_devname = (
                  self._pa_inst.get_device_info_by_index(i).get('name'))
              if device_candidate in current_devname:
                selected_dev_id = i
                break
          if selected_dev_id is not None:
            break

        assert selected_dev_id is not None

        rospy.loginfo('Using device: {}'.format(repr(
            self._pa_inst.get_device_info_by_index(
            selected_dev_id).get('name'))))

        with self.PaStreamWrapper(self._pa_inst.open(
                format=pyaudio.paInt16, channels=1, rate=48000, output=True,
                output_device_index=selected_dev_id)) as pa_stream:
            try:
                grpc_response = self._stub.TextToAudio(
                    tts_service_pb2.TextToAudioRequest(text=goal.text))
                assert (grpc_response.audio_format
                        == tts_service_pb2.SIGNED_16BITS_LINEAR_PCM)
                assert (grpc_response.error.error_code
                    == tts_service_pb2.TextToAudioResponse.ErrorInfo.NO_ERROR)
            except Exception as e:
                result.is_success = False
                result.error_info = (
                    'gRPC call to TtsService failed: {}'.format(e))
                rospy.logerr(e)
                self._action_server.set_succeeded(result)
                return

            audio_data = grpc_response.audio_data
            audio_length = len(audio_data) / 32000.0

            rospy.loginfo("Audio length is {0} seconds.".format(audio_length))

            # TODO(shengye): Here we assume little-endian. This should be
            # documented in interface defination.
            audio_data_int16 = struct.unpack(
                '<' + 'h' * (len(audio_data) / 2), audio_data)

            upsample_start_time = time.time()
            audio_48k = librosa.core.resample(
                np.array(map(float, audio_data_int16)), 16000, 48000,
                res_type='kaiser_fast')
            upsample_time = time.time() - upsample_start_time
            print "Upsample from 16K to 48K took {} s.".format(upsample_time)

            audio_48k_buf = struct.pack('h' * len(audio_48k), *audio_48k)

            feedback = speak_text_msgs.SpeakTextFeedback()
            feedback.time_left = audio_length
            self._action_server.publish_feedback(feedback)

            play_thread = threading.Thread(
                target=pa_stream.write, args=(audio_48k_buf, ))

            play_thread.daemon = True
            play_thread.start()
            #TODO(shengye): Implement preempt behavior
            # if self._action_server.is_preempt_requested():
            #     self._action_server.set_preempted()
            time.sleep(audio_length)
            play_thread.join()

            result.total_time = audio_length
            result.is_success = True
            self._action_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('speak_text_action_server')
    server = SpeakTextActionServer('cogrob/speak_text_action')
    rospy.spin()
