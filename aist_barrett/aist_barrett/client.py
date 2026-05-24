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
    def __init__(self, node: Node, name: str='bhand'):
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
    def name(self) -> str:
        """ Name of the gripper.
        """
        return self._name

    @property
    def type(self) -> str:
        """ Name of the gripper's type.
        """
        return 'three_finger'

    @property
    def base_link(self) -> str:
        """ Name of the gripper's base link.
        """
        return self._name + '_base_link'

    @property
    def tip_link(self) -> str:
        """ Name of the gripper's tip link.
        """
        return self._name + '_tip_link'

    @property
    def parameters(self) -> dict:
        """ Dictionary of gripper parameters.
        """
        return self._parameters

    def set_torque_mode(self, enable: bool):
        """ Set finger velocity value to the gripper.

        Args:
          enable: `True` if torque mode turned on. `False` otherwise.
        """
        return self._set_velocity.call(SetBool.Request(data=enable))

    def set_velocity(self, velocity: float, *, spread: bool=False):
        """ Set velocity value to the gripper.

        Args:
          velocity: Velocity of the gripper.
        """
        return self._set_velocity.call(SetVelocity.Request(velocity=velocity,
                                                           spread=spread))

    def pregrasp(self) -> None:
        """ Move to release position and return immediatelty.
        """
        self.release(timeout_sec=0.0)

    def grasp(self, *, timeout_sec: Optional[float]=None):
        """ Grasp an object with the gripper.
        Desired spread, applied effort and mode are specified by parameters
        with 'spread', 'max_effort' and 'mode' keys, respectively.

        Args:
          timeout_sec: Timeout time waiting for the gripper to complete
            grasping. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the movement result of
          `GripperCommand.Result` type.
        """
        return self.move(0.0, spread=None, max_effort=None, mode=None,
                         timeout_sec=timeout_sec)

    def postgrasp(self) -> None:
        """ Move to grasp position and return immediatelty.
        """
        self.grasp(timeout_sec=0.0)

    def release(self, *, timeout_sec: Optional[float]=None):
        """ Release an object grasped by the gripper.
        No effort is applied. Current spread and mode are kept unchanged.

        Args:
          timeout_sec: Timeout time waiting for the gripper to complete
            releasing. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the movement result of
          `GripperCommand.Result` type.
        """
        return self.move(self.parameters['release_gap'],
                         spread=None, max_effort=0.0,
                         mode=None, timeout_sec=timeout_sec)

    def move(self, gap: float, *,
             spread: Optional[float]=None, max_effort: Optional[float]=None,
             mode: Optional[int]=None, timeout_sec: Optional[float]=None):
        """ Move gripper to the desired position.

        Args:
          gap: Desired gap between the fingers.
          spread: Desired spread. The value of parameter 'spread' is used,
            if `None`.
          max_effort: Desired maximum effort to be applied. The value of
            parameter 'max_effort' is used, if `None`.
          mode: Desired grasping mode. Possible values are
            'GripperCommand.Goal.[PINCH|ENCOMPASS|SCISSOR|GRIP]'.
            The value of parameter 'mode' is used, if `None`.
          timeout_sec: Timeout time waiting for the gripper to complete
            movement. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
            A tuple of the goal status and the movement result of
            `control_msgs.action.GripperCommand.Result` type
        """
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
