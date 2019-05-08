#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.subscriber_state import SubscriberState
from custom_flexbe_states.publisher_float_state import PublisherFloatState
from flexbe_states.log_key_state import LogKeyState
from custom_flexbe_states.vision_track_state import VisionTrackState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.wait_state import WaitState
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Apr 27 2019
@author: davis
'''
class pub_wait_for_feedbackSM(Behavior):
	'''
	pub and wait for feedback
	'''


	def __init__(self):
		super(pub_wait_for_feedbackSM, self).__init__()
		self.name = 'pub_wait_for_feedback'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 333 20 
		# Get the image, pass it to image recognition and calculate necessary rotation.

		# O 619 629 
		# Publish desired rotation, wait, listen for a result.

		# O 242 276 
		# Decide whether to repeat the process or finished.



	def create(self):
		# x:83 y:425, x:723 y:307
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.fake_data = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:185 y:62
			OperatableStateMachine.add('getImage',
										SubscriberState(topic="/multisense/camera/left/image_color/compressed", blocking=True, clear=False),
										transitions={'received': 'findRedBall', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Low, 'unavailable': Autonomy.Full},
										remapping={'message': 'img'})

			# x:969 y:572
			OperatableStateMachine.add('publishTorsoRotation',
										PublisherFloatState(topic="/values"),
										transitions={'done': 'wait'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'result_data'})

			# x:996 y:65
			OperatableStateMachine.add('logFloat',
										LogKeyState(text="val: {}", severity=Logger.REPORT_HINT),
										transitions={'done': 'publishTorsoRotation'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'result_data'})

			# x:406 y:62
			OperatableStateMachine.add('findRedBall',
										VisionTrackState(),
										transitions={'done': 'calcDesiredTorsoRotation', 'invalid': 'failed'},
										autonomy={'done': Autonomy.Low, 'invalid': Autonomy.Full},
										remapping={'img': 'img', 'value': 'last_data'})

			# x:688 y:62
			OperatableStateMachine.add('calcDesiredTorsoRotation',
										CalculationState(calculation=self.calculate),
										transitions={'done': 'logFloat'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'last_data', 'output_value': 'result_data'})

			# x:729 y:571
			OperatableStateMachine.add('wait',
										WaitState(wait_time=0.5),
										transitions={'done': 'subscribeToResult'},
										autonomy={'done': Autonomy.Off})

			# x:413 y:572
			OperatableStateMachine.add('subscribeToResult',
										SubscriberState(topic="/result", blocking=True, clear=True),
										transitions={'received': 'decisionToRepeatOrExit', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Low, 'unavailable': Autonomy.Full},
										remapping={'message': 'message'})

			# x:238 y:322
			OperatableStateMachine.add('decisionToRepeatOrExit',
										OperatorDecisionState(outcomes=['again', 'done'], hint=None, suggestion=None),
										transitions={'again': 'getImage', 'done': 'finished'},
										autonomy={'again': Autonomy.High, 'done': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def calculate(self, data):
		# from tf import TransformListener
		# import tf, rospy
		# listener = tf.TransformListener("/tf")
		# print("PELVIS: ", listener.frameExists("pelvis"))
		# print("HEAD: ", listener.frameExists("utorso"))
		# # t = listener.getLatestCommonTime("pelvis", "utorso")
		# position, quaternion = listener.lookupTransform("pelvis", "utorso", rospy.Time(0))
		# euler = tf.transformations.euler_from_quaternion(quaternion)
		# print("ORIENTATION OF utorso from PELVIS: ", euler)

		imgW = 1024
		imgCenter = 1024/2

		print("data: ", data)
		print(data)
		print("STUFF: ", -(data-imgCenter)/10)

		return -(data-imgCenter)/10

		pass

	def decision(self, data):
		pass
	# [/MANUAL_FUNC]
