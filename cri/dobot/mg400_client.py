"""Python client interface for Dobot MG400 using sockets
Firmware version 1.5.7.0
Uses https://github.com/Dobot-Arm/TCP-IP-4Axis-Python-CMD (now Dobot-Arm/TCP-IP-4Axis-Python).
That 4-axis API has no ServoJ, so servo_joints streams JointMovJ (+ SpeedJ/AccJ/CP) instead.
"""

import threading
import numpy as np
from time import sleep, monotonic

from cri.dobot.mg400.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from cri.transforms import euler2quat, quat2euler

feedback = None  # latest real-time feedback packet, set by MG400Client.get_feed


class MG400Client:
    """Python client interface for Dobot MG400
    """

    class CommandFailed(RuntimeError):
        pass

    class InvalidZone(ValueError):
        pass

    def __init__(self, ip="192.168.1.6", feed_port=30004,
                 servo_speed=100, servo_acc=100, servo_cp=100,
                 servo_period=0.05, servo_max_lag=10.0):
        """Settings for servo_joints (JointMovJ streaming):
        servo_speed / servo_acc / servo_cp: SpeedJ / AccJ / CP (1-100 %) of each segment;
            these are scaled by the global SpeedFactor.
        servo_period: minimum time between segments (s); short segments sent at a steady
            rate overlap in the queue, so CP can blend them.
        servo_max_lag: stop adding segments while the arm is more than this many degrees
            behind the last target, so the queue cannot grow.
        """
        self.set_units('millimeters', 'degrees')
        self._servo_mode = False
        self._servo_delay = 0.0
        self._feed_port = feed_port
        self._servo_params = (f"SpeedJ={int(servo_speed)}", f"AccJ={int(servo_acc)}", f"CP={int(servo_cp)}")
        self._servo_period = servo_period
        self._servo_max_lag = servo_max_lag
        self._last_servo_target = None
        self._last_servo_time = None
        self.connect(ip)

    def __repr__(self):
        return "{} ({})".format(self.__class__.__name__, self.get_info())

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()        
        
    def set_units(self, linear, angular):
        """Sets linear and angular units.
        """
        units_l = {'millimeters' : 1.0,
               'meters' : 1000.0,
               'inches' : 25.4,
               }
        units_a = {'degrees' : 1.0,
               'radians' : 57.2957795,
               }
        self._scale_linear = units_l[linear]
        self._scale_angle  = units_a[angular]

    def connect(self, ip):
        """Connects to Dobot MG400.
        """
        self._dashboard = DobotApiDashboard(ip, 29999)
        self._move = DobotApiMove(ip, 30003)
        # feedback port: 30004 pushes every 8 ms (real-time), 30005 every 200 ms
        self._feed = DobotApi(ip, self._feed_port)

        self._dashboard.ClearError()
        self._dashboard.EnableRobot()
        self._dashboard.User(0)
        self._dashboard.Tool(0)

        feed_thread = threading.Thread(target=self.get_feed, args=(self._feed,))
        feed_thread.setDaemon(True)
        feed_thread.start()

    def get_feed(self, feed: DobotApi):
        global feedback
        hasRead = 0
        while True:
            data = bytes()
            while hasRead < 1440:
                temp = feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0

            a = np.frombuffer(data, dtype=MyType)
            if hex((a['test_value'][0])) == '0x123456789abcdef':

                # Refresh Properties
                feedback = a[0]
            # no sleep: recv() blocks until the next packet, and sleeping would let
            # packets queue up so the readings fall behind the robot

    def get_info(self):
        """Returns a unique robot identifier string.
        """
        pass

    def set_servo_mode(self, servo_mode):
        """Sets whether move_joints uses ServoJ (streaming) instead of JointMovJ.
        """
        if type(servo_mode) is not bool:
            raise Exception("Servo mode must be [True, False]")
        self._servo_mode = servo_mode

    def get_servo_mode(self):
        """Returns whether move_joints uses ServoJ.
        """
        return self._servo_mode

    def set_servo_delay(self, servo_delay):
        """Sets the time to sleep after each ServoJ command (s).
        """
        self._servo_delay = servo_delay

    def get_servo_delay(self):
        """Gets the time to sleep after each ServoJ command (s).
        """
        return self._servo_delay

    def move_joints(self, joint_angles):
        """Executes an immediate move to the specified joint angles.

        joint_angles = (j0, j1, j2, rz)
        j0, j1, j2, rz are numbered from base to end effector and are
        measured in degrees (default)
        """
        if self._servo_mode:
            self.servo_joints(joint_angles)
            return
        joint_angles = np.array(joint_angles, dtype=np.float32).ravel()
        joint_angles *= self._scale_angle
        self._move.JointMovJ(*joint_angles)
        self._move.Sync()

    def servo_joints(self, joint_angles):
        """Streams a joint target; returns without waiting for the motion.

        joint_angles = (j0, j1, j2, rz), degrees (default). Send targets close to
        the current pose at a steady rate (e.g. teleoperation).

        The MG400 TCP/IP API has no ServoJ (the controller answers -1), so this sends
        JointMovJ(..., SpeedJ, AccJ, CP) without Sync(). Calls closer together than
        servo_period are dropped (the next one carries the newer target), and so are calls
        while the arm lags the last target by more than servo_max_lag.
        """
        joint_angles = np.array(joint_angles, dtype=np.float32).ravel()[:4]
        joint_angles *= self._scale_angle
        now = monotonic()
        if self._last_servo_time is not None and now - self._last_servo_time < self._servo_period:
            return
        last = self._last_servo_target
        if last is not None and feedback is not None:
            if np.max(np.abs(feedback["q_actual"][:4] - last)) > self._servo_max_lag:
                return
        reply = self._move.JointMovJ(*joint_angles, *self._servo_params)
        if not reply.startswith("0,"):
            raise self.CommandFailed(f"JointMovJ rejected by controller: {reply.strip()}")
        self._last_servo_target = joint_angles.copy()
        self._last_servo_time = now
        if self._servo_delay > 0:
            sleep(self._servo_delay)

    def move_linear(self, pose):
        """Executes a linear/cartesian move from the current base frame pose to
        the specified pose.

        pose = (x, y, z, 0, 0, rz)
        x, y, z specify a Euclidean position (default mm)
        """
        pose = np.array(pose, dtype=np.float32).ravel()
        pose = quat2euler(pose, 'sxyz')[[0, 1, 2, 5]]
        pose[:3] *= self._scale_linear
        pose[3] *= self._scale_angle
        self._move.MovL(*pose)
        self._move.Sync()
 
    def set_linear_speed(self, linear_speed):
        """Sets the linear speed (% of maximum)
        """
        if linear_speed < 1 or linear_speed > 100: 
            raise Exception("Linear speed value outside range of 1-100%")
        self._dashboard.SpeedL(round(linear_speed))
        self._move.Sync()

    def set_angular_speed(self, angular_speed):
        """Sets the angular speed (% of maximum)
        """
        if angular_speed < 1 or angular_speed > 100: 
            raise Exception("Angular speed value outside range of 1-100%")
        self._dashboard.SpeedJ(round(angular_speed))
        self._move.Sync()
   
    def set_speed(self, speed):
        """Sets the angular speed (% of maximum)
        """
        if speed < 1 or speed > 100: 
            raise Exception("Speed value outside range of 1-100%")
        self._dashboard.SpeedFactor(round(speed))
        self._move.Sync()

    def get_speed(self):
        """Gets the speed (% of maximum)
        """
        return feedback["speed_scaling"] 

    def get_joint_angles(self):
        """returns the robot joint angles.

        joint_angles = (j0, j1, j2, rz, 0, 0)
        j0, j1, j2, rz are numbered from base to end effector and are
        measured in degrees (default)
        """
        return feedback["q_actual"][:4] / self._scale_angle

    def get_pose(self):
        """retvalsurns the TCP pose in the reference coordinate frame.

        pose = (x, y, z, rz, 0, 0)
        x, y, z specify a Euclidean position (default mm)
        rz rotation of end effector
        """
        pose_actual = feedback["tool_vector_actual"]
        pose = np.array([*pose_actual[:3], 0, 0, pose_actual[3]], dtype=np.float32) 
        pose = euler2quat(pose, 'sxyz')
        pose[:3] /= self._scale_linear        
        pose[3] /= self._scale_angle        
        return pose

    def move_circular(self, via_pose, end_pose):
        """Executes a movement in a circular path from the current base frame
        pose, through via_pose, to end_pose.
        
        via_pose, end_pose = (x, y, z, qw, qx, qy, qz)
        x, y, z specify a Cartesian position (default mm)
        qw, qx, qy, qz specify a quaternion rotation
        """     
        via_pose = np.array(via_pose, dtype=np.float32).ravel()
        via_pose = quat2euler(via_pose, 'sxyz')[[0, 1, 2, 5]]
        via_pose[:3] *= self._scale_linear
        via_pose[3] *= self._scale_angle
        end_pose = np.array(end_pose, dtype=np.float32).ravel()
        end_pose = quat2euler(end_pose, 'sxyz')[[0, 1, 2, 5]]
        end_pose[:3] *= self._scale_linear   
        end_pose[3] *= self._scale_angle   
        pose = self.get_pose()
        pose = quat2euler(pose, 'sxyz')[[0, 1, 2, 5]]
        pose[:3] *= self._scale_linear   
        pose[3] *= self._scale_angle   
        self._move.MovL(*pose)
        self._move.Circle(1, *via_pose, 0, 0, *end_pose, 0, 0) # doesn't work 
        self._move.Sync()

    def close(self):
        """Releases any resources held by the controller (e.g., sockets). And disconnects from Dobot magician
        """
        self._move.Sync()
        self._dashboard.ClearError()
        self._dashboard.DisableRobot()
        self._dashboard.close()
        print("Disconnecting Dobot...")
