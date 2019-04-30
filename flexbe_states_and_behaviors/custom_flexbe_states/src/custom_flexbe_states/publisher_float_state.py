#!/usr/bin/env python


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Float32

'''
Created on 25-Apr-2019

@author: Davis
'''


class PublisherFloatState(EventState):
    '''
	Publishes a float (std_msgs/Float32) message on a given topic name.

	-- topic	string		The topic on which should be published.

	># value 					Value of float.

	<= done 					Done publishing.

	'''

    def __init__(self, topic):
        '''
		Constructor
		'''
        super(PublisherFloatState, self).__init__(outcomes=['done'], input_keys=['value'])

        self._topic = topic
        self._pub = ProxyPublisher({self._topic: Float32})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        val = Float32()
        print(type(userdata.value))
        val.data = userdata.value
        self._pub.publish(self._topic, val)
