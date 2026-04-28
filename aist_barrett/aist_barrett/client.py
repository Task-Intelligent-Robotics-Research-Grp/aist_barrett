# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Clients of gripper action controller of control_msg/GripperCommandAction type.
@file   __init__.py
@author t.ueshiba@aist.go.jp
"""
import rclpy, threading
from rclpy.duration           import Duration
from rclpy.parameter_client   import AsyncParameterClient
from rclpy.callback_groups    import MutuallyExclusiveCallbackGroup
from action_msgs.msg          import GoalStatus
from std_srvs.srv             import SetBool, Trigger
from aist_barrett_msgs.action import GripperCommand
from aist_barrett_msgs.srv    import SetVelocity
from aist_barrett_msgs.msg    import TactileStates
from .simple_action_client    import SimpleActionClient

######################################################################
#  class BarrettHand                                                 #
######################################################################
class BarrettHand(SimpleActionClient):
    def __init__(self, node, prefix='bhand_'):
        # Create action client for
        ns = prefix + 'controller'
        self._callback_group = MutuallyExclusiveCallbackGroup()
        super().__init__(node, GripperCommand, ns + '/gripper_cmd',
                         self._callback_group)
        self.wait_for_server()

        # Create service client for setting torque mode.
        self._set_torque_mode \
            = node.create_client(SetBool, ns + '/set_torque_mode',
                                 callback_group=self._callback_group)

        # Create service client for setting velocity.
        self._set_velocity \
            = node.create_client(SetVelocity, ns + '/set_velocity',
                                 callback_group=self._callback_group)

        self._parameters = {'release_gap': 0.1,
                            'spread':      0.0,
                            'max_effort':  10.0,
                            'mode':        GripperCommand.Goal.PINCH}

    @property
    def parameters(self):
        """
        Return a dictionary of grippaer parameters
        @return a dictionary of grippaer parameters with string keys
        """
        return self._parameters

    def set_torque_mode(self, enable):
        self._set_velocity.call(SetBool.Request(data=enable)).success

    def set_velocity(self, velocity, spread=False):
        self._set_velocity.call(SetVelocity.Request(velocity=velocity,
                                                    spread=spread)).success

    def grasp(self, timeout=Duration()):
        return self.move(self.parameters['grasp_gap'],
                         None, None, None, None, timeout)

    def release(self, timeout=Duration()):
        return self.move(self.parameters['release_gap'],
                         None, 0.0, None, None, timeout)

    def move(self, gap, spread=None, max_effort=None, mode=None,
             timeout=Duration()):
        if not spread:
            spread = self.parameters['spread']
        if not max_effort:
            max_effort = self.parameters['max_effort']
        if not mode:
            mode = self.parameters['mode']
        return self.send_goal(GripperCommand.Goal(gap=gap, spread=spread,
                                                  max_effort=max_effort,
                                                  mode=mode),
                              timeout=timeout)
