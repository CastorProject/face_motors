#!/usr/bin/env python
import rospy
import time

from actuator import Actuator

#custom control service and messages
#from dynamixel_workbench_msgs.srv import JointCommand
from opsoro_workbench.srv import ServoCommand
from opsoro_workbench.srv import EnablePcaPower
#from dynamixel_workbench_msgs.srv import TorqueEnable

print ServoCommand


class DmxServo(Actuator):
    '''
    Dynamixel servo class: Inherits the base Actuator class
    '''
    def __init__(self):
        super(DmxServo, self).__init__()
        #set custom dynamixel motor info 
        self.info = {
				     "ros_label"            : "head",
				     "controller"          : "/dmx_controller",
				     "actuation"           : "electrical",
				     "motion"	          : "rotatory",
				     "hardware"           : "dynamixel",
				     "id" 		                  : 0,
                                     "component_id": 0
				    }

        #deploy actuator
        self.deploy_actuator()

    def set_motor_id(self, id):
        '''
        set dynamixel-motor id
        '''
        if id in range(1, 255):
            self.set_id(id)
            return True
        else:
            return False



class OnohatServo(Actuator):
    '''
    Onohat servo class: inherits the base Actuator class
    '''
    def __init__(self):
        super(OnohatServo, self).__init__()
        self.info = {
					 "ros_label"   : "hand",
				     "controller"  : "/onohat_controller",
					 "actuation"   : "electrical",
					 "motion"	   : "rotatory",
				     "hardware"    : "built-in pca",
				     "id" 		   : 0,
                     "component_id": 0
				    }

	self.set_srv_types(ServoCommand, EnablePcaPower)        
	#deploy actuator
        self.deploy_actuator()


    def set_motor_id(self, id):
        '''
        set pca servo id
        '''
        if id in range(1,15):
            self.set_id(id)
            return True
        else:
            False


if __name__ == '__main__':
	rospy.init_node("servo")
	
	rate = rospy.Rate(10) # 10hz
	
	#servo  = DmxServo()
	#shoulder = DmxServo()
	mouth1 = OnohatServo()
	mouth2 = OnohatServo()
	mouth3 = OnohatServo()
	
	mouth1.set_motor_id(0)
	mouth2.set_motor_id(1)
	mouth3.set_motor_id(2)
	
	mouth1.set_actuation_range(min =790, origin =800, max = 1101)
	mouth2.set_actuation_range(min =790, origin =1200, max = 1700)  #inverted 1700 is sad, 790 is happy
	mouth3.set_actuation_range(min =799, origin =1100, max = 1501) #
	
	time.sleep(3)
	mouth1.set_position(command ={'value':801}) 
	
	
	while not (rospy.is_shutdown()):
		print(hand.get_state())
	
		#print servo.get_info()
		rate.sleep()

