# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import robomaster
from robomaster import robot

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")

    ep_arm = ep_robot.robotic_arm

    # ep_arm.sub_position(freq=5, callback=sub_data_handler)
    ep_arm.moveto(x=140, y = 90).wait_for_completed()

    ep_arm.moveto(x=50, y = 150).wait_for_completed()
    
    ep_arm.moveto(x=140, y = 90).wait_for_completed()

    # ep_arm.move(y=30).wait_for_completed()


    # time.sleep(2)
    # ep_arm.move(x=30).wait_for_completed()

    # ep_arm.move(y=-30).wait_for_completed()
    # ep_arm.move(y=-50).wait_for_completed()
    # ep_arm.unsub_position()

    ep_robot.close()
