#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

'''
Created on Time

@author: People
'''

class CalculationTwoKeyState(EventState):
	'''
	Implements a state that can perform a calculation based on userdata.
	calculation is a function which takes exactly one parameter, input_value from userdata,
	and its return value is stored in output_value after leaving the state.
	
	-- calculation  function	The function that performs the desired calculation.
								It could be a private function (self.foo) manually defined in a behavior's source code
								or a lambda function (e.g., lambda x: x^2, where x will be the input_value).

	># input_value_one  object		Input to the calculation function.
	># input_value_two  object		Input to the calculation function #2.

	#> output_value object		The result of the calculation.

	<= done						Indicates completion of the calculation.

	'''


	def __init__(self, calculation):
		'''
		Constructor
		'''
		super(CalculationState, self).__init__(outcomes=['done'],
											   input_keys=['input_value_one', 'input_value_two'],
											   output_keys=['output_value'])
		
		self._calculation = calculation
		self._calculation_result = None
		
		
	def execute(self, userdata):
		'''Execute this state'''

		userdata.output_value = self._calculation_result

		# nothing to check
		return 'done'


	def on_enter(self, userdata):
		
		if self._calculation is not None:
			try:
				self._calculation_result = self._calculation(userdata.input_value_one, userdata.input_value_two)
			except Exception as e:
				Logger.logwarn('Failed to execute calculation function!\n%s' % str(e))
		else:
			Logger.logwarn('Passed no calculation!')
