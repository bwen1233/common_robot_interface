"""Hiwonder LSC series PWM servo controller (e.g. LSC-24) over USB HID.

Protocol (Hiwonder "LSC Series Controller Communication Protocol"):
    frame = 0x55 0x55 | length | cmd | params..., length = len(params) + 2, little-endian, no checksum.
    CMD_SERVO_MOVE          0x03  [count, time_lo, time_hi, (id, pos_lo, pos_hi) * count]   no reply
    CMD_ACTION_GROUP_RUN    0x06  [group, times_lo, times_hi]
    CMD_ACTION_GROUP_STOP   0x07  []
    CMD_ACTION_GROUP_SPEED  0x0B  [group, speed_lo, speed_hi]
    CMD_GET_BATTERY_VOLTAGE 0x0F  []  -> reply [mv_lo, mv_hi]
The LSC boards drive PWM servos, which have no feedback: there is no command to read a
servo's position. read_position() returns the last commanded position instead.
Over USB the board is a HID device (VID 0x0483, PID 0x5750); each HID report is the frame
prefixed with report id 0.
"""

import time

CMD_SERVO_MOVE = 0x03
CMD_ACTION_GROUP_RUN = 0x06
CMD_ACTION_GROUP_STOP = 0x07
CMD_ACTION_GROUP_SPEED = 0x0B
CMD_GET_BATTERY_VOLTAGE = 0x0F

PULSE_MIN, PULSE_MAX = 500, 2500   # PWM pulse width (us)
TIME_MAX = 30000                    # ms


class HiwonderLSCController:
    def __init__(self, servo_id=1, default_time=500, vid=0x0483, pid=0x5750, serial=None):
        """servo_id: controller channel the servo is plugged into.
        default_time: move duration (ms) when move_to() is not given one.
        serial: HID serial string to pick one board when several are connected.
        """
        import hid  # imported here so cri.controller still loads without hidapi installed

        self.id = servo_id
        self.time = default_time
        self._last_position = None
        self._device = hid.device()
        try:
            if serial is None:
                self._device.open(vid, pid)
            else:
                self._device.open(vid, pid, serial)
        except OSError as e:
            raise Exception(f"Failed to open Hiwonder LSC controller (HID {vid:04x}:{pid:04x}): {e}")
        print(f"Opened {self._device.get_product_string()} controller "
              f"({self._device.get_serial_number_string()})")

    # ------------------------------------------------------------------ low level
    def _send(self, cmd, params=()):
        frame = [0x55, 0x55, len(params) + 2, cmd, *params]
        self._device.write([0x00, *frame])

    def _recv(self, cmd, timeout_ms=200):
        t_end = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < t_end:
            data = self._device.read(64, max(1, int((t_end - time.monotonic()) * 1000)))
            if len(data) >= 4 and data[0] == 0x55 and data[1] == 0x55 and data[3] == cmd:
                return bytes(data[4:4 + data[2] - 2])
        return None

    # ------------------------------------------------------------------ interface
    def set_speed(self, move_time):
        """Sets the default move duration (ms); PWM servos are commanded by time, not speed."""
        self.time = move_time
        print(f"Default move time set to {move_time} ms")

    def move_to(self, position, time_ms=None, wait=True):
        """Moves the servo to a pulse width (500-2500 us) over time_ms (default: default_time).
        wait=True sleeps for the move duration (there is no feedback to poll).
        """
        position = int(min(max(position, PULSE_MIN), PULSE_MAX))
        t = int(min(max(self.time if time_ms is None else time_ms, 0), TIME_MAX))
        self._send(CMD_SERVO_MOVE, (1, t & 0xFF, t >> 8, self.id, position & 0xFF, position >> 8))
        self._last_position = position
        if wait:
            time.sleep(t / 1000.0)

    def read_position(self):
        """PWM servos cannot report their position; returns the last commanded one (or None)."""
        return self._last_position

    def read_voltage(self):
        """Controller supply voltage (V), or None if the board does not answer."""
        self._send(CMD_GET_BATTERY_VOLTAGE)
        r = self._recv(CMD_GET_BATTERY_VOLTAGE)
        if r is None or len(r) < 2:
            print("No voltage reply from controller")
            return None
        return int.from_bytes(r[:2], "little") / 1000.0

    def run_action_group(self, group, times=1):
        self._send(CMD_ACTION_GROUP_RUN, (group, times & 0xFF, times >> 8))

    def stop_action_group(self):
        self._send(CMD_ACTION_GROUP_STOP)

    def close(self):
        self._device.close()
        print("Controller closed")


if __name__ == "__main__":
    # read-only check: opens the board and reads its supply voltage; nothing moves
    servo = HiwonderLSCController(servo_id=1)
    print("Supply voltage:", servo.read_voltage(), "V")
    print("Last commanded position:", servo.read_position())


    servo.close()
