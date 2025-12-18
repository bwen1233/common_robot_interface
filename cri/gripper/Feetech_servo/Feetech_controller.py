from cri.gripper.Feetech_servo.scservo_sdk import *



class FeetechServoController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=1000000, servo_id=1, default_speed=200):
        self.port = port
        self.baudrate = baudrate
        self.id = servo_id
        self.speed = default_speed
        self.acc = 50
        self.torque = 500


        self.portHandler = PortHandler(self.port)
        self.packetHandler = hls(self.portHandler)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            raise Exception("Failed to open port")

        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            raise Exception("Failed to set baudrate")


    def set_speed(self, speed):
        self.speed = speed
        print(f"Default speed set to {speed}")


    def move_to(self, position, speed=None, acc=None, torque=None, wait=True):
        s = speed if speed is not None else self.speed
        a = acc if acc is not None else self.acc
        t = torque if torque is not None else self.torque

        scs_comm_result, scs_error = self.packetHandler.WritePosEx(self.id, position, s, a, t)

        if scs_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print(self.packetHandler.getRxPacketError(scs_error))

        if wait:
            while True:
                moving, comm_result, error = self.packetHandler.ReadMoving(self.id)

                if comm_result != COMM_SUCCESS:
                    print(self.packetHandler.getTxRxResult(comm_result))

                if moving == 0:
                    break

                time.sleep(0.002)




    def read_position(self):
        pos, spd, comm_result, error = self.packetHandler.ReadPosSpeed(self.id)

        if comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(comm_result))
            return None

        if error != 0:
            print(self.packetHandler.getRxPacketError(error))
            return None

        return pos

    def close(self):
        self.portHandler.closePort()
        print("Port closed")


if __name__ == "__main__":

        # Tips make contact at 700,
    # Open limit 1700,
    # Run this every time to give the permission sudo chmod 666 / dev / ttyUSB0

    servo = FeetechServoController(port="/dev/ttyUSB0", servo_id=1)



    pos = servo.read_position()
    print("Current pos:", pos)

    servo.move_to(1500)
    # time.sleep(1)

    print("Current pos:", servo.read_position())

    servo.close()