#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import actionlib


import pyaudio
import wave
from ctypes import *
from contextlib import contextmanager
import sys
from os import path

from ndev.core import NDEVCredentials, HEADER, UserInput
from ndev.tts import *
from mhri_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback


ERROR_HANDLER_FUNC = CFUNCTYPE(
    None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(file_name, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)


class RenderNASRTTS:
    def __init__(self):
        rospy.init_node('render_speech_nasr_tts', anonymous=False)

        # Real speech config, if config don't exists then set default value
        try:
            config_file = rospy.get_param("~config_file")
        except KeyError as e:
            rospy.logerr('NASR credentials_file is needed...')
            quit()

        self.creds = NDEVCredentials(config_file)
        if not self.creds.has_credentials():
            rospy.logerr("Please provide NDEV credentials.")
            sys.exit(-1)

        self.language_kr = {'display': 'Korean',
                            'properties': {'voice': 'Narae', 'code': 'ko_KR'}}

        self.server = actionlib.SimpleActionServer(
            'render_speech', RenderItemAction, self.execute_callback, False)
        # self.server.register_preempt_callback(self.preempted_callback)
        self.server.start()


    def execute_callback(self, goal):
        rospy.loginfo('%s rendering requested [%s]...' % (rospy.get_name(), goal.data))
        result = RenderItemResult()
        feedback = RenderItemFeedback()
        result.result = True

        file_name = path.expanduser('~/.ros/speech_temp.wav')
        audio_type = TTS.get_audio_type(file_name)
        atype = TTS.Accept[audio_type]
        sample_rate = 22000

        text = unicode(goal.data, 'utf-8')
        synth_req = TTS.make_request(creds=self.creds,
                                     desired_tts_lang=self.language_kr,
                                     sample_rate=sample_rate,
                                     nchannels=1,
                                     sample_width=2,
                                     text=text,
                                     filename=file_name,
                                     audio_type=audio_type)

        if not synth_req.response.was_successful():
            rospy.logerr("\nNDEV TTS Error %s" % synth_req.response.error_message)
            return

        # Send Feedback first.
        feedback.is_rendering = True
        self.server.publish_feedback(feedback)


        chunk_size = 2048
        with noalsaerr():
            paudio = pyaudio.PyAudio()
        wave_file = wave.open(file_name, 'rb')
        stream = paudio.open(format=paudio.get_format_from_width(wave_file.getsampwidth()),
                             channels=wave_file.getnchannels(),
                             rate=wave_file.getframerate(),
                             output=True)

        data = wave_file.readframes(chunk_size)
        while data != '':
            if self.server.is_preempt_requested():                
                self.server.set_preempted()
                result.result = False
                break

            stream.write(data)
            data = wave_file.readframes(chunk_size)
            self.server.publish_feedback(feedback)

        stream.stop_stream()
        stream.close()
        paudio.terminate()

        # Remove temporary speech files.
        os.remove(path.expanduser('~/.ros/speech_temp.wav'))
        if result.result:
            self.server.set_succeeded(result)


if __name__ == '__main__':
    tts = RenderNASRTTS()
    rospy.spin()
