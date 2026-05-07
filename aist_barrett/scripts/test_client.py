#!/usr/bin/env python3
#
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
import rclpy, sys, threading
from math                import radians
from rclpy.node          import Node
from rclpy.duration      import Duration
from aist_barrett.client import BarrettHand

#########################################################################
#  class BarrettHandClient                                              #
#########################################################################
class BarrettHandClient(Node):
    def __init__(self, name):
        super().__init__(name)

        gripper_name = self.declare_parameter('gripper_name', 'bhand').value
        self._torque_mode = False
        self._gripper     = BarrettHand(self, gripper_name)
        self._gripper.wait_for_server()

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        def is_float(s):
            try:
                float(s)
            except ValueError:
                return False
            else:
                return True

        while rclpy.ok():
            print('==== Available commands ====')
            print('  g:         Grasp')
            print('  r:         Release')
            print('  <numeric>: Open gripper with a specified gap value')
            print('  c:         Cancel motion')
            print('  w:         Wait until goal completed')
            print('  s:         Set spread')
            print('  e:         Set max effort')
            print('  m:         Set mode')
            print('  v:         Set velocity')
            print('  q:         Quit\n')

            key = input('>> ')
            if key == 'g':
                self._gripper.grasp(timeout=None)
            elif key == 'r':
                self._gripper.release(timeout=None)
            elif is_float(key):
                self._gripper.move(float(key),
                                   self._gripper.paremeters['spread'],
                                   timeout=None)
            elif key == 'c':
                self._gripper.cancel()
            elif key == 'w':
                status, result = self._gripper \
                                     .wait(timeout=Duration(seconds=10))
                print(result)
            elif key == 's':
                spread = min(max(0.0, float(input('  spread: '))), 180.0)
                self._gripper.parameters['spread'] = radians(spread)
                print('spread set to %f' % spread)
            elif key == 'e':
                effort = min(max(0.0, float(input('  effort: '))), 180.0)
                self._gripper.parameters['max_effort'] = effort
                print('max_effort set to %f' % effort)
            elif key == 'm':
                mode = int(input('  mode(0: PINCH, 1: ENCOMPASS, 2: SCISSOR, 3: GRIP): '))
                self._gripper.parameters['mode'] = mode
                print('mode set to %f' % mode)
            elif key == 'v':
                velocity = float(input('  velocity: '))
                success  = self._gripper.set_velocity(velocity)
                if success:
                    print('velocity successfully set to %f' % velocity)
                else:
                    print('failed to set velocity')
            elif key=='q':
                break
            else:
                print('unknown command: %s' % key)

        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init(args=sys.argv)

    test = BarrettHandClient('test_client')
    rclpy.spin(test)

if __name__ == '__main__':
    main()
