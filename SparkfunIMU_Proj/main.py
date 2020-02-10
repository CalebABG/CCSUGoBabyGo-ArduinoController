import string
import sys
import time
import serial
import struct

from serial.tools import list_ports


def prog_shutdown_func(exception=None):
    global script_running, imu_serialdev, motor_controller_serdev

    script_running = False
    
    print(exception)

    s_msg = "Try Send Stop Command :- "
    
    if motor_controller_serdev is not None:
        print("Stopping Motors...")
        s_msg += u'\u2611'
        motor_controller_serdev.write(struct.pack('B', 0x0))
        motor_controller_serdev.close()
        print("Closed MotorController Serial")
    else:
        s_msg += u'\u24CD'

    if imu_serialdev is not None:
        imu_serialdev.close()
        print("Closed IMU Serial")

    print(s_msg)

    sys.exit(-1)


def map_val(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def clamp_val(n, min_n, max_n):
    return min(max(n, min_n), max_n)


def m_drive(v):
    return (-.376 * v) + 48


def m_angular_offset(v):
    return (v * .117) - 15


# up and down - left stick
last_ud = 128

# left and right - right stick
last_lr = 128

m1_stop = 64
m2_stop = 192


imu_port = 'COM9'
motor_controller_port = 'COM11'


print("Listing Ports on Computer...")
for device in list_ports.comports():
    print(device)

print("\n")


# Check your COM port and baud rate
print("Opening %s\n" % imu_port)

try:
    # old baud: 57600 115200
    imu_serialdev = serial.Serial(port=imu_port, baudrate=115200, timeout=1)
    motor_controller_serdev = serial.Serial(port=motor_controller_port, baudrate=9600)
except serial.serialutil.SerialException:
    print("IMU not found at port " + imu_port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(-1)

imu_init_time = 2  # in seconds

print("Giving the razor IMU board %s seconds to boot..." % imu_init_time)
time.sleep(imu_init_time)  # Sleep for ... seconds to wait for the board to boot

# configure board #
# stop data-stream
imu_serialdev.write(b'#o0\n')

# discard old input
discard = imu_serialdev.readlines()

# set output mode: #ox
print("Setting Output Mode")
imu_serialdev.write(b'#oscb\n')

# start data-stream
imu_serialdev.write(b'#o1\n')

print("Printing IMU data...")

script_running = True

if __name__ == '__main__':
    while script_running:
        try:
            try:
                data = struct.unpack('<9f', imu_serialdev.read(36))

                MAX_AXIS_VAL = 512.0 / 2
                MIN_AXIS_VAL = -MAX_AXIS_VAL

                last_ud = map_val(data[0], MIN_AXIS_VAL, MAX_AXIS_VAL, 1.0, 127.0)
                last_lr = map_val(data[1], MIN_AXIS_VAL, MAX_AXIS_VAL, 128.0, 255.0)

                print("ABS_X: {} - ABS_A: {}".format(last_ud, last_lr))

                m_ud1 = round(m_drive(last_ud) - 18) + m1_stop
                m_ud2 = round(m_drive(last_ud) - 32) + m2_stop

                offset = round(m_angular_offset(last_lr))

                m_vel1 = int(clamp_val(m_ud1 - offset, 1, 255))
                m_vel2 = int(clamp_val(m_ud2 + offset, 1, 255))

                # m_vel1 = int(clamp_val(last_ud, 1, 255))
                # m_vel2 = int(clamp_val(last_ud, 1, 255))

                # print(m_vel1, m_vel2)

                print("Writing Serial Data(M1, M2): ({}, {})".format(m_vel1, m_vel2))
                motor_controller_serdev.write(struct.pack('BB', m_vel1, m_vel2))
                print("Wrote Data Successfully \n")

                # print(data)
            except IOError as err:
                prog_shutdown_func(err)

        except KeyboardInterrupt as ex:
            prog_shutdown_func(ex)
