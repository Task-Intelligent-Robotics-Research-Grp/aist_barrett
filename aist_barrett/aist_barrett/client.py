#  BSD 3-Clause License
#
#  Copyright (c) 2026, National Institute of Advanced Industrial Science
#  and Technology(AIST)
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
#  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
#  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import rclpy, threading
from rclpy.duration               import Duration
from rclpy.parameter_client       import AsyncParameterClient
from rclpy.callback_groups        import MutuallyExclusiveCallbackGroup
from action_msgs.msg              import GoalStatus
from std_srvs.srv                 import SetBool, Trigger
from aist_barrett_msgs.action     import GripperCommand
from aist_barrett_msgs.srv        import SetVelocity
from aist_barrett_msgs.msg        import TactileStates
from task_wrappers.service_client import ServiceClient
from task_wrappers.action_client  import SimpleActionClient

#*********************************************************************
#  class BarrettHand                                                 *
#*********************************************************************
class BarrettHand(SimpleActionClient):
    def __init__(self, node, name='bhand'):
        self._name = name
        self._cbg  = MutuallyExclusiveCallbackGroup()

        # Create action client for gripper command.
        controller_ns = name + '_controller'
        super().__init__(node, GripperCommand, controller_ns + '/gripper_cmd',
                         callback_group=self._cbg)

        # Create service client for setting torque mode.
        self._set_torque_mode \
            = ServiceClient(node, SetBool, controller_ns + '/set_torque_mode',
                            callback_group=self._cbg)

        # Create service client for setting velocity.
        self._set_velocity \
            = ServiceClient(node, SetVelocity, controller_ns + '/set_velocity',
                            callback_group=self._cbg)

        self._parameters = {'release_gap': 0.32,
                            'spread':      0.0,
                            'max_effort':  10.0,
                            'mode':        GripperCommand.Goal.PINCH}

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return 'three_finger'

    @property
    def base_link(self):
        return self._name + '_base_link'

    @property
    def tip_link(self):
        return self._name + '_tip_link'

    @property
    def parameters(self):
        """ Return a dictionary of gripper parameters.
        :return: Dictionary of gripper parameters with string keys.
        """
        return self._parameters

    def set_torque_mode(self, enable):
        return self._set_velocity.call(SetBool.Request(data=enable))

    def set_velocity(self, velocity, *, spread=False):
        return self._set_velocity.call(SetVelocity.Request(velocity=velocity,
                                                           spread=spread))

    def pregrasp(self):
        self.release(timeout_sec=0.0)

    def grasp(self, *, timeout_sec=None):
        return self.move(0.0, spread=None, max_effort=None, mode=None,
                         timeout_sec=timeout_sec)

    def postgrasp(self):
        self.grasp(timeout_sec=0.0)

    def release(self, *, timeout_sec=None):
        return self.move(self.parameters['release_gap'],
                         spread=None, max_effort=0.0,
                         mode=None, timeout_sec=timeout_sec)

    def move(self, gap, *,
             spread=None, max_effort=None, mode=None, timeout_sec=None):
        if not spread:
            spread = self.parameters['spread']
        if not max_effort:
            max_effort = self.parameters['max_effort']
        if not mode:
            mode = self.parameters['mode']
        return self.send_goal(GripperCommand.Goal(gap=gap, spread=spread,
                                                  max_effort=max_effort,
                                                  mode=mode),
                              timeout_sec=timeout_sec)
