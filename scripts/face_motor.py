#!/usr/bin/env python
import rospy
import time

from servo import OnohatServo
from opsoro_workbench.srv import EnablePcaPower
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float32

class FaceMotor(object):
	def __init__(self, name):
		self.name = name
		rospy.init_node(self.name)
		self.rate = rospy.Rate(10) # 10hz
		self.initPublishers()
		self.initSubscribers()
		self.initVariables()

	def initPublishers(self):
		self.pubEyesBehavior = rospy.Publisher("/enableDefaultEyes", Bool, queue_size = 10)
		self.pubLEye = rospy.Publisher("/moveLEye", Point, queue_size = 10)
		self.pubREye = rospy.Publisher("/moveREye", Point, queue_size = 10)
		self.pubSizePupils = rospy.Publisher("/size_pupils", Float32, queue_size = 10)

	def initSubscribers(self):
		self.subEmotions = rospy.Subscriber('/emotions', String, self.callbackEmotions)
		self.subStopTalk = rospy.Subscriber('/stopTalk', Bool, self.callbackStopTalk)
		return

	def initVariables(self):
		self.changeEmotions = False
		self.enableEyesBehavior = Bool()
		self.eyesPosition = Point()
		self.stopTalk = Bool()
		self.pupilSize = Float32()
		self.emotionsDict = {
		"happy": self.set_happy,
		"sad": self.set_sad,
		"surprise": self.set_surprise,
		"angry": self.set_angry,
		"neutral": self.set_neutral,
		"demo": self.demo,
		"talk": self.talk
		}

	def set_servos(self):
		#mouth motors
		self.eyeledright = OnohatServo()
		self.eyeledleft  = OnohatServo()
		self.mouth1      = OnohatServo()
		self.mouth2      = OnohatServo()
		self.mouth3      = OnohatServo()

		#eyeled
		self.eyeledright.set_motor_id(4)
		self.eyeledleft.set_motor_id(5)
		self.mouth1.set_motor_id(6)
		self.mouth2.set_motor_id(7)
		self.mouth3.set_motor_id(8)

		#set actuation ranges
		self.mouth1.set_actuation_range(min =1099, max = 1331, origin =1261)
		self.mouth2.set_actuation_range(min =799, max = 1301, origin =950)
		self.mouth3.set_actuation_range(min =899, max = 1501 , origin =1280)
		self.eyeledleft.set_actuation_range(min = 1199, max = 1851, origin =1450)
		self.eyeledright.set_actuation_range(min = 1049, max = 1651, origin =1420)

		#set origin position
		self.set_neutral()

	def activate_mosfet(self, val):
		rospy.wait_for_service('/onohat_controller/torque_enable')
		serviceTorque = rospy.ServiceProxy('/onohat_controller/torque_enable', EnablePcaPower)
		try:
			serviceResponse = serviceTorque(val)
			return serviceResponse.result
		except rospy.ServiceException as exc:
			rospy.logwarn("[%s] Service did not process request: " + str(exc), self.name)
		return

	def eyes(self, x, y, pupilSize):
		self.enableEyesBehavior.data = False
		self.pubEyesBehavior.publish(self.enableEyesBehavior)
		time.sleep(0.5)
		self.eyesPosition.x = x
		self.eyesPosition.y = y
		self.eyesPosition.z = 0
		self.pubLEye.publish(self.eyesPosition)
		time.sleep(0.5)
		self.pupilSize.data = pupilSize
		self.pubSizePupils.publish(self.pupilSize)
		self.rate.sleep()

	#expressions
	def set_happy(self):
		self.eyes(0, 8, 0.7)
		self.mouth1.set_position(command ={'value':1300})
		self.mouth2.set_position(command ={'value':1300})
		self.mouth3.set_position(command ={'value':900})
		self.eyeledleft.set_position(command ={'value':1300})
		self.eyeledright.set_position(command ={'value':1530})

	def set_sad(self):
		self.eyes(0, -20, 0.4)
		self.mouth1.set_position(command ={'value':1100})
		self.mouth2.set_position(command ={'value':800})
		self.mouth3.set_position(command ={'value':1500})
		self.eyeledleft.set_position(command ={'value':1300})
		self.eyeledright.set_position(command ={'value':1530})

	def set_surprise(self):
		self.eyes(0, 15, 0.3)
		self.mouth1.set_position(command ={'value':1330})
		self.mouth2.set_position(command ={'value':800})
		self.mouth3.set_position(command ={'value':1500})
		self.eyeledleft.set_position(command ={'value':1300})
		self.eyeledright.set_position(command ={'value':1530})

	def set_angry(self):
		self.eyes(0, 0, 0.5)
		self.mouth1.set_position(command ={'value':1255})
		self.mouth2.set_position(command ={'value':800})
		self.mouth3.set_position(command ={'value':1500})
		self.eyeledleft.set_position(command ={'value':1800})
		self.eyeledright.set_position(command ={'value':1100})

	def set_neutral(self):
		self.enableEyesBehavior.data = True
		self.pubEyesBehavior.publish(self.enableEyesBehavior)
		self.mouth1.set_origin_position()
		self.mouth2.set_origin_position()
		self.mouth3.set_origin_position()
		self.eyeledleft.set_origin_position()
		self.eyeledright.set_origin_position()

	def talk(self):
		self.eyes(0, 8, 0.7)
		self.mouth2.set_position(command ={'value':950})
		self.mouth3.set_position(command ={'value':1300})
		while not self.stopTalk:
			self.mouth1.set_position(command ={'value':1330})
			time.sleep(0.2)
			self.mouth1.set_position(command ={'value':1300})
			time.sleep(0.2)

	def demo(self):
		print("demo")
		time.sleep(2)
		self.set_happy()
		time.sleep(2)
		self.set_sad()
		time.sleep(2)
		self.set_angry()
		time.sleep(2)
		self.set_surprise()
		time.sleep(2)
		self.set_neutral()
		time.sleep(2)
		self.talk()

	def callbackEmotions(self, msg):
		self.emotion = msg.data
		self.changeEmotions = True
		return

	def callbackStopTalk(self, msg):
		self.stopTalk = msg.data
		return

	def main(self):
		rospy.loginfo("[%s] Facemotor node started ok", self.name)
		active = self.activate_mosfet(True)
		if active:
			while not (rospy.is_shutdown()):
				if self.changeEmotions:
					if self.emotion == "talk":
						self.stopTalk = False
					self.emotionsDict[self.emotion]()
					self.changeEmotions = False
			self.rate.sleep()
		#while not (rospy.is_shutdown()):
		#	if self.changeEmotions:
		#		self.emotionsDict[self.emotion]()
		#		self.changeEmotions = False
		#	self.rate.sleep()
		return


if __name__ == '__main__':
	fm = FaceMotor("motor_face_handler")
	fm.set_servos()
	fm.main()
