#!/usr/bin/env python
#-*- encoding: utf8 -*-

import sys
import json
import rospy
from os import path
import actionlib
from six.moves import urllib
import pygame
import os

from mhri_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback

class RenderNaverVoiceTTS:
    def __init__(self):
        rospy.init_node('render_speech_naver_voice_tts', anonymous=False)

        # Real speech config, if config don't exists then set default value
        try:
            config_file = rospy.get_param("~config_file")
        except KeyError as e:
            rospy.logerr('Need config_file for this node...')
            quit()

        with open(config_file) as f:
            self.speech_conf = json.load(f)

        if not 'client_secret' in self.speech_conf.keys() or not 'client_id' in self.speech_conf.keys():
            rospy.logerr('The client_id or client_secret is needed for this node...')
            quit()

        # Load config or setting default value
        if not 'language' in self.speech_conf.keys():
            self.speech_conf['language'] = 'kr'
        if not 'speaker' in self.speech_conf.keys():
            if self.speech_conf['language'] is 'kr':
                self.speech_conf['speaker'] = 'jinho'
            elif self.speech_conf['language'] is 'en':
                self.speech_conf['speaker'] = 'clara'
        if not 'speed' in self.speech_conf.keys():
            self.speech_conf['speed'] = 0


        pygame.init()
        pygame.mixer.init()


        self.server = actionlib.SimpleActionServer(
            'render_speech', RenderItemAction, self.execute_callback, False)
        # self.server.register_preempt_callback(self.preempted_callback)
        self.server.start()



        rospy.loginfo('[%s] initialized...' % rospy.get_name())
        rospy.spin()


    def execute_callback(self, goal):
        rospy.loginfo('%s rendering requested [%s]...' % (rospy.get_name(), goal.data))
        result = RenderItemResult()
        feedback = RenderItemFeedback()
        result.result = True

        if goal.data == '':
            rospy.sleep(0.5)
            self.server.set_succeeded(result)
            return

        encText = urllib.parse.quote(goal.data)
        data = "speaker=%s&speed=%s&text="%(self.speech_conf['speaker'], self.speech_conf['speed']) + encText;
        url = "https://openapi.naver.com/v1/voice/tts.bin"
        request = urllib.request.Request(url)
        request.add_header("X-Naver-Client-Id", self.speech_conf['client_id'])
        request.add_header("X-Naver-Client-Secret", self.speech_conf['client_secret'])
        response = urllib.request.urlopen(request, data=data.encode('utf-8'))

        rescode = response.getcode()
        file_name = path.expanduser('~/.ros/speech_temp.mp3')
        if rescode == 200:
            response_body = response.read()
            with open(file_name, 'wb') as f:
                f.write(response_body)
        else:
            rospy.logerr('Error code: %d'%rescode)
            result.result = False

        # Send Feedback first.
        feedback.is_rendering = True
        self.server.publish_feedback(feedback)

        pygame.mixer.music.load(file_name)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            self.server.publish_feedback(feedback)
            pygame.time.Clock().tick(10)
            if self.server.is_preempt_requested():
                pygame.mixer.music.stop()
                self.server.set_preempted()
                result.result = False

        # Remove temporary speech files.
        os.remove(path.expanduser('~/.ros/speech_temp.mp3'))
        if result.result:
            self.server.set_succeeded(result)

    def preempted_callback(self):
        pass


if __name__ == '__main__':
    m = RenderNaverVoiceTTS()
