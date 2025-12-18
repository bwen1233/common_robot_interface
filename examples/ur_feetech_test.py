import time

import numpy as np
import ipdb
from cri.robot import SyncRobotGripperWrapper, SyncRobot, SyncGripper
from cri.controller import RTDEController
from cri.controller import FeetechServoController


def main():

    robot = SyncRobot(RTDEController(ip="192.168.1.141"))
    gripper = SyncGripper(FeetechServoController(port="/dev/ttyUSB0", servo_id=1))
    robot = SyncRobotGripperWrapper(
        robot,
        gripper,

    )


    base_frame = (0, 0, 0, 0, 0, 0)
    work_frame = (0, -640.0, 350, 180, 0, 90)

    robot.robot.tcp = (0, 0, -80, 0, 0, 0)
    robot.robot.linear_speed = 20
    robot.robot.angular_speed = 10
    robot.robot.coord_frame = work_frame


    robot.robot.move_linear((0, 0, -30, 0, 0, 0))
    robot.gripper.move_to(1200)
    ipdb.set_trace()
if __name__ == '__main__':
    main()