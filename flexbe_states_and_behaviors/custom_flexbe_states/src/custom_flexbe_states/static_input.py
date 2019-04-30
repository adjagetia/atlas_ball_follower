#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from std_msgs.msg import Float32

'''
Created on 25-Apr-2019

@author: Davis
'''

class StaticInput(EventState):
    '''
    State to apply planning scene using ApplyPlanningSceneAction.

    -- value        float          data_value

    #> data         float         output the data on this
    <= done                        successful


    '''

    def __init__(self, value=0.0):
        '''
        Constructor
        '''
        super(StaticInput, self).__init__(
            output_keys=['data'],
            outcomes=['done'])

        self.value  = value

        self.success = None
        self.return_code = None

    def execute(self, userdata):
        '''
        Execute this state
        '''

        userdata.data = self.value
        return 'done'

    def on_enter(self, userdata):
        pass

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        pass
