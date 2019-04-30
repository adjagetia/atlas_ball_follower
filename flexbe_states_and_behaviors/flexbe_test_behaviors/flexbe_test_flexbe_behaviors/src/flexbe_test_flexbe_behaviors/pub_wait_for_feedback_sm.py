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
from flexbe_states.decision_state import DecisionState
from flexbe_states.wait_state import WaitState
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

		# O 189 35 
		# tested. good

		# O 429 34 
		# tested. good

		# O 732 36 
		# TODO

		# O 329 284 
		# TODO

		# O 918 639 
		# needs to test with converging to center

		# O 323 624 
		# tested, good. Improvable if publish to this topic at launch



	def create(self):
		# x:118 y:405, x:663 y:332
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

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

			# x:985 y:569
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
										autonomy={'done': Autonomy.Off, 'invalid': Autonomy.Off},
										remapping={'img': 'img', 'value': 'last_data'})

			# x:688 y:62
			OperatableStateMachine.add('calcDesiredTorsoRotation',
										CalculationState(calculation=self.calculate),
										transitions={'done': 'logFloat'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'last_data', 'output_value': 'result_data'})

			# x:241 y:316
			OperatableStateMachine.add('decisionToRunAgain',
										DecisionState(outcomes=['adjust', 'done'], conditions=self.decision),
										transitions={'adjust': 'getImage', 'done': 'finished'},
										autonomy={'adjust': Autonomy.High, 'done': Autonomy.High},
										remapping={'input_value': 'result_data'})

			# x:729 y:571
			OperatableStateMachine.add('wait',
										WaitState(wait_time=0.5),
										transitions={'done': 'subscribeToResult'},
										autonomy={'done': Autonomy.Off})

			# x:413 y:572
			OperatableStateMachine.add('subscribeToResult',
										SubscriberState(topic="/result", blocking=True, clear=True),
										transitions={'received': 'decisionToRunAgain', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Low, 'unavailable': Autonomy.Full},
										remapping={'message': 'message'})


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
