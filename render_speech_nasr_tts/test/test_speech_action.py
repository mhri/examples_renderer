#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import actionlib

from mhri_msgs.msg import RenderItemAction, RenderItemGoal

def func_done(state, result):
	print 'Done'

def func_feedback(feedback):
	print 'Feedback'

def func_active():
	print "Start"


def main():
	client = actionlib.SimpleActionClient('render_speech', RenderItemAction)
	client.wait_for_server()

	goal = RenderItemGoal(name='say', data="안녕하세요. 음성 합성 테스트입니다.")
	client.send_goal(goal, done_cb=func_done, feedback_cb=func_feedback, active_cb=func_active)

	rospy.sleep(8)
	client.cancel_all_goals()

	client.wait_for_result()
	rospy.signal_shutdown(0)


if __name__ == '__main__':
	rospy.init_node('motion_renderer_node', anonymous=False)
	m = main()
	rospy.spin()
