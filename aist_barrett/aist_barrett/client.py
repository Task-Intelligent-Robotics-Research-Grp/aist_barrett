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
from rclpy.callback_groups        import MutuallyExclusiveCallbackGroup
from action_msgs.msg              import GoalStatus
from control_msgs.action          import GripperCommand
from control_msgs.msg             import GripperCommand as GripperCommandMsg
from aist_barrett_msgs.msg        import TactileStates
from task_wrappers.service_client import ServiceClient
from task_wrappers.action_client  import SimpleActionClient
from ddynamic_reconfigure2.client import ParameterClient

from typing                       import Optional
from rclpy.node                   import Node

#*********************************************************************
#  class BarrettHand                                                 *
#*********************************************************************
class BarrettHand(SimpleActionClient):
    _RemoteParams = ('velocity', 'spread', 'grasp_mode', 'torque_mode')

    def __init__(self, node: Node, name: str='bhand'):
        self._name = name
        self._cbg  = MutuallyExclusiveCallbackGroup()

        # Create action client for gripper command.
        controller_ns = name + '_controller'
        super().__init__(node, GripperCommand, controller_ns + '/command',
                         callback_group=self._cbg)

        # Create parameter client for setting/getting controller parameters.
        self._param_clnt   = ParameterClient(node, controller_ns)
        self._local_params = {'release_position': 0.32,
                              'max_effort':       10.0}

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
        timeout_sec = 10.0
        values = self._param_clnt \
                     .get_parameters_sync(BarrettHand._RemoteParams,
                                          timeout_sec=timeout_sec)
        remote_params = dict(zip(BarrettHand._RemoteParams, values))
        return self._local_params | remote_params

    def set_parameters(self, params: dict):
        """ Set gripper parameters.

        Args:
          params: Dictionary of gripper parameters. Effective keys are
          - 'max_effort': Maximum effort in Newton applied when grasping.
          - 'release_position': Gap between fingers in meters when releasing.
          - 'velocity': Finger velocity
          - 'mode': Grasping mode(0: PINCH, 1: ENCOMPASS, 2: SCISSOR, 3: GRIP).
        """
        self._local_params |= dict(filter(lambda item: item[0]
                                          not in BarrettHand._RemoteParams,
                                          params.items()))

        remote_params = dict(filter(lambda item: item[0]
                                    in BarrettHand._RemoteParams,
                                    params.items()))
        timeout_sec = 1.0
        self._param_clnt.set_parameters_sync(remote_params,
                                             timeout_sec=timeout_sec)

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
        return self.move(0.0, max_effort=None, timeout_sec=timeout_sec)

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
        return self.move(self.parameters['release_position'],
                         max_effort=0.0, timeout_sec=timeout_sec)

    def move(self, gap: float, *, max_effort: Optional[float]=None,
             timeout_sec: Optional[float]=None):
        """ Move gripper to the desired position.

        Args:
          gap: Desired gap between the fingers.
          max_effort: Desired maximum effort to be applied. The value of
            parameter 'max_effort' is used, if `None`.
          timeout_sec: Timeout time waiting for the gripper to complete
            movement. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the movement result of
          `GripperCommand.Result` type
        """
        if not max_effort:
            max_effort = self._local_params['max_effort']
        return self.send_goal(GripperCommand.Goal(
                                  command=GripperCommandMsg(
                                      position=gap, max_effort=max_effort)),
                              timeout_sec=timeout_sec)

    def grasped(self, *, timeout_sec: Optional[float]=None):
        _, result = self.wait(timeout_sec=timeout_sec)
        return result.stalled
