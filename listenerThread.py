import rospy
import threading
import config
from std_msgs.msg import UInt16

class ListenerThread(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		rospy.Subscriber('head_reset', UInt16, self.callback)

	def run(self):
		rospy.spin()

	def callback(self, data):
		config.yaw_zero = config.head_yaw
		config.pitch_zero = config.head_pitch
		config.roll_zero = config.head_roll
