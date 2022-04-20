#!/usr/bin/env python3.7

import sys, struct, serial, os
import numpy as np
import argparse
import matplotlib.pyplot as plt
import time
import bluetooth
import threading
import subprocess
import rospy
from diagnostic_msgs.msg import KeyValue
from pub_classes import diag_class, threeIMUs_class
import tkinter


os.chdir(os.path.expanduser("~/catkin_ws/src/multimodal_human_robot_collaboration/"))

IMU_MSGS = ['ERROR', 'Ready', 'Unknown', 'Shutdown', 'Starting', 'Connecting', 'Initialising']
IMU_SYS_MSGS = ['ERROR', 'Ready', 'Setting Up']

# Argument parsing
parser = argparse.ArgumentParser(
    description='Base structure for connecting and streaming data from Shimmer 3 IMU sensor')
parser.add_argument('--user_name', '-N',
                    help='Set name of user, default: unknown',
                    default='unknown',
                    action="store_true")
parser.add_argument('--user_id', '-I',
                    help='Set id of user, default: None',
                    default=1,
                    action="store_true")

args = parser.parse_known_args()[0]
frame_id = f'shimmerBase {args.user_name} {args.user_id} node'

# Shimmer sensor connection params
serialports = ['/dev/rfcomm3', '/dev/rfcomm4', '/dev/rfcomm5']
POSITIONS = ['Hand', 'Wrist', 'Arm']
SHIM_IDs = ['F2:AF:44', 'F2:B6:ED', 'F2:C7:80']
numsensors = len(serialports)

gyro_offset = [[0], [0], [0]]
gyro_sens = [[65.5, 0, 0], [0, 65.5, 0], [0, 0, 65.5]]
gyro_align = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
accel_offset = [[2253], [2253], [2253]]
accel_sens = [[92, 0, 0], [0, 92, 0], [0, 0, 92]]
accel_align = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
shimmers = {}
shim_threads = {}
quit_IMU = False

dir_path = os.path.dirname(os.path.realpath(__file__))

def shutdown_imu():
    global quit_IMU
    quit_IMU = True


class shimmer():
    def __init__(self, q):
        self._ready = False
        self._connect_error = True
        #self._connection = None
        self._connected = False
        self._ID = SHIM_IDs[q]
        self._location = POSITIONS[q]
        self._port = serialports[q]
        self._serial = None
        self._accel = np.zeros((3))
        self._gyro = np.zeros((3))
        self._batt = None
        self._batt_perc = None
        self._num = q
        self._ddata = b''
        self._batt_time = time.time()
        self._numbytes = 0
        self._batt_arr = np.empty((1, 0))
        self._shutdown = True
        self._status = 4 # starting
        rospy.on_shutdown(self.shutdown)

    def wait_for_ack(self):
        ddata = b''
        ack = struct.pack('B', 0xff)
        count = 0
        timer = time.time()
        while ddata != ack:
            if time.time() - timer < 1:
                try:
                    ddata = self._serial.read(1)
                except Exception as e:
                    print(f"Error reading acknowledgment from {self._location} sensor: {e}")
                    self._connected = False
                    count = count + 1
                if count > 3:
                    print(f"###---{self._location} acknowledgement error---###")
                    return False
            #print("0x%02x" % ord(ddata))
            else:
                print(f"###---{self._location} acknowledgement timeout---###")
                return False
        return True

    def initiate(self): 
        while (not self._connected) & (not self._connect_error) & (not quit_IMU):
            time.sleep(5)
            try:
                self._serial = serial.Serial(self._port, 115200, timeout=5)
                self._serial.flushInput()

                devices = subprocess.Popen("rfcomm", stdout=subprocess.PIPE)
                devices = devices.communicate()[0].decode("utf-8")
                devices = devices.split("\n")
                correct_device = False
                for device in devices:
                    if self._port[-7:] in device:
                        if self._ID in device:
                            print(f"Correct device for {self._location}")
                            correct_device = True
                            break

                if correct_device:
                    print(f"---{self._location} port opening, done.")
                    # send the set sensors command
                    self._serial.write(struct.pack('BBBB', 0x08, 0xC0, 0x20, 0x00))  # analogaccel, mpu gyro, batt volt
                    if not self.wait_for_ack():
                        return False
                    self._connected = True
                    self._status = 6 # initialising
                    print(f"---{self._location} sensor setting, done.")
                    # send the set sampling rate command
                    self._serial.write(
                        struct.pack('BBB', 0x05, 0x80,
                                    0x02))  # 51.2Hz (6400 (0x1900)). Has to be done like this for alignment reasons
                    if not self.wait_for_ack():
                        return False
                    print(f"---{self._location} sampling rate setting, done.")
                    # send start streaming command
                    self._serial.write(struct.pack('B', 0x07))
                    if not self.wait_for_ack():
                        return False
                    print(f"---{self._location} start command sending, done.")
                    #self.inquiry_response()  # Use this if you want to find the structure of data that will be streamed back

                    return True

                else:
                    print(f"Incorrect device for {self._location}")
                    return False

            except Exception as e:
                print(f'exception in {self._location} initiate: {e}')
                self._connected = False

    def inquiry_response(self):
        # Outputs data structure of data to be streamed back
        self._serial.flushInput()
        # send inquiry command
        self._serial.write(struct.pack('B', 0x01))
        self.wait_for_ack()

        # read incoming data
        ddata = b''
        numbytes = 0
        framesize = 9

        print("Inquiry response:")
        while (numbytes < framesize) and (not quit_IMU):
            ddata = ddata + self._serial.read(framesize)
            print(ddata)
            numbytes = len(ddata)

        data = ddata[0:framesize]
        ddata = ddata[framesize:]
        #numbytes = len(ddata)
        (packettype) = struct.unpack('B', bytes(data[0].to_bytes(1, sys.byteorder)))
        (samplingrate, configByte0, configByte1, configByte2, configByte3, numchans, bufsize) = struct.unpack('HBBBBBB',
                                                                                                              data[1:9])
        print("          Packet type: 0x%02x" % packettype)
        print("        Sampling rate: 0x%04x" % samplingrate)
        print("  Config Setup Byte 0: 0x%02x" % configByte0)
        print("  Config Setup Byte 1: 0x%02x" % configByte1)
        print("  Config Setup Byte 2: 0x%02x" % configByte2)
        print("  Config Setup Byte 3: 0x%02x" % configByte3)
        print("   Number of channels: 0x%02x" % numchans)
        print("          Buffer size: 0x%02x" % bufsize)

        for i in range(numchans):
            data = self._serial.read(1)
            print("           Channel %2d:" % i)
            print("                        0x%02x" % (struct.unpack('B', data[0].to_bytes(1, sys.byteorder))))

    def calibrate_data(self, data, sensor):
        # Calibrate readings to m/s2, deg/s. More info: http://www.shimmersensing.com/images/uploads/docs/Shimmer_9DOF_Calibration_User_Manual_rev2.10a.pdf
        if sensor == 'a':
            Ri = np.linalg.inv(accel_align)
            Ki = np.linalg.inv(accel_sens)
            offset = accel_offset
        elif sensor == 'g':
            Ri = np.linalg.inv(gyro_align)
            Ki = np.linalg.inv(gyro_sens)
            offset = gyro_offset
        else:
            print('Calibration sensor invalid, must be ''a'' or ''g'' ')
            raise ValueError
        calib_data = np.transpose(Ri @ Ki @ (np.transpose(data) - offset))
        return np.reshape(np.nan_to_num(calib_data), (-1))

    def checkbattery(self):
        batt_last = self._batt_perc
        if self._batt < 3.2:
            self._batt_perc = 0.0
        elif self._batt < 3.627:
            self._batt_perc = 5.9
        elif self._batt < 3.645:
            self._batt_perc = 9.8
        elif self._batt < 3.663:
            self._batt_perc = 13.8
        elif self._batt < 3.681:
            self._batt_perc = 17.7
        elif self._batt < 3.699:
            self._batt_perc = 21.6
        elif self._batt < 3.717:
            self._batt_perc = 25.6
        elif self._batt < 3.7314:
            self._batt_perc = 29.5
        elif self._batt < 3.735:
            self._batt_perc = 33.4
        elif self._batt < 3.7386:
            self._batt_perc = 37.4
        elif self._batt < 3.7566:
            self._batt_perc = 41.3
        elif self._batt < 3.771:
            self._batt_perc = 45.2
        elif self._batt < 3.789:
            self._batt_perc = 49.2
        elif self._batt < 3.8034:
            self._batt_perc = 53.1
        elif self._batt < 3.8106:
            self._batt_perc = 57.0
        elif self._batt < 3.8394:
            self._batt_perc = 61.0
        elif self._batt < 3.861:
            self._batt_perc = 64.9
        elif self._batt < 3.8826:
            self._batt_perc = 68.9
        elif self._batt < 3.9078:
            self._batt_perc = 72.8
        elif self._batt < 3.933:
            self._batt_perc = 76.7
        elif self._batt < 3.969:
            self._batt_perc = 80.7
        elif self._batt < 4.0086:
            self._batt_perc = 84.6
        elif self._batt < 4.041:
            self._batt_perc = 88.5
        elif self._batt < 4.0734:
            self._batt_perc = 92.5
        elif self._batt < 4.113:
            self._batt_perc = 96.4
        else:
            self._batt_perc = 100

        if batt_last is not None:
            if (self._batt_perc < batt_last):
                print(f"{self._location} sensor battery at {self._batt_perc}%")

    def start(self):
        count = 0
        while count <= 3:
            self._connect_error = False
            if self.initiate():  # Send set up commands, etc to shimmer
                print(f'---Initiated {self._location} Sensor---')
                return True
            else:
                count += 1
                print(f"Failed to initialise {self._location} sensor, attempt {count}/3")
                return False

    def getdata(self):
        framesize = 18  # 1byte packet type + 3byte timestamp + 3x2byte Analog Accel + 2byte Battery + 3x2byte Gyro
        while self._numbytes < framesize and (not quit_IMU):
            try:
                self._ddata = self._ddata + self._serial.read(framesize)
            except Exception as e:
                print(f"Unable to read {self._location} IMU data: {e}")
                return False
            self._numbytes = len(self._ddata)

        data = self._ddata[0:framesize]
        self._ddata = self._ddata[framesize:]
        self._numbytes = len(self._ddata)

        self._accel = self.calibrate_data(np.array(struct.unpack('HHH', data[4:10]), ndmin=2), 'a')

        batt_now = struct.unpack('H', data[10:12])[0] * 6 / 4095
        batt_now = np.nan_to_num(batt_now)
        self._batt_arr = np.append(self._batt_arr, batt_now)
        self._batt = np.mean(self._batt_arr)
        while len(self._batt_arr) > 50:
            self._batt_arr = np.delete(self._batt_arr, 0)

        if (time.time() - self._batt_time) > 1:
            self.checkbattery()
            self._batt_time = time.time()

        self._gyro = self.calibrate_data(np.array(struct.unpack('>hhh', data[12:framesize]), ndmin=2), 'g')
        self._status = 1 # ready
        return True

    def shutdown(self):
        # Reset all flags/parameters
        self._connected = False
        self._ready = False
        self._connect_error = True

        # Shut sensor down and kill serial connection
        try:
            count = 1
            sd = False
            while (not sd) and (count <= 3):
                # send stop streaming command
                self._serial.write(struct.pack('B', 0x20))
                print(f"{self._location} stop command sent, waiting for ACK_COMMAND")
                if self.wait_for_ack():
                    print(f"---{self._location} stop ACK_COMMAND received.")
                    # close serial port
                    self._serial.close()
                    sd = True
                else:
                    print(f"---{self._location} stop ACK_COMMAND *NOT* received. Attempt {count}/3")
                    count += 1

        except Exception as e:
            print(f"{self._location} close down sensor error: {e}")
            pass

        return True

def shimmer_thread(num):
    print(f"Setting up Sensor {num + 1}/{numsensors}")
    shimmers[num] = shimmer(num)  # Create instance of shimmer class for each device

    # read incoming data
    first_try = True
    while not quit_IMU:
        if shimmers[num]._ready and (not quit_IMU):
            if shimmers[num]._connected:
                success = shimmers[num].getdata()
                if not success:
                    shimmers[num]._ready = False
                    shimmers[num]._connected = False
            else:
                print(f"{shimmers[num]._location} not connected?")

        elif not quit_IMU:
            if first_try:
                # Try starting connection
                print(f"Starting {POSITIONS[num]} connection and initialisation")
                first_try = False
            else:
                # Try restarting connection
                print(f"Lost {POSITIONS[num]} connection, restarting")

            shimmers[num]._shutdown = False
            shimmers[num]._ready = shimmers[num].start()

            if shimmers[num]._ready:
                print(f"{shimmers[num]._location} sensor connected, starting data stream {num + 1}")
            else:
                print(f"{shimmers[num]._location} sensor failed to start successfully, retrying")
                shimmers[num]._shutdown = shimmers[num].shutdown()

    # quit_IMU condition
    #shutdown = shimmers[num].shutdown()


def IMUsensorsMain():
    print("-----Here we go-----")
    rospy.init_node(f'shimmerBase_{args.user_name}_{args.user_id}', anonymous=True)
    rate = rospy.Rate(50)  # Message publication rate, Hz => should be 50
    
    keyvalues = [KeyValue(key = f'Shimmer {POSITIONS[0]} {SHIM_IDs[0]}', value = IMU_MSGS[2]),
                 KeyValue(key = f'Shimmer {POSITIONS[1]} {SHIM_IDs[1]}', value = IMU_MSGS[2]),
                 KeyValue(key = f'Shimmer {POSITIONS[2]} {SHIM_IDs[2]}', value = IMU_MSGS[2]),
                 KeyValue(key = f'Overall', value = IMU_SYS_MSGS[2])] # [unknown, unknown, unknown, setting up]
    diag_obj = diag_class(frame_id=frame_id, user_id=args.user_id, user_name=args.user_name, queue=1, keyvalues=keyvalues)
    IMU_pub_obj = threeIMUs_class(frame_id, user_id=args.user_id, user_name=args.user_name, queue=1)

    # Start separate thread for collecting data from each Shimmer
    for shimthread in range(0, numsensors):
        shim_threads[shimthread] = threading.Thread(target=shimmer_thread, args=(shimthread,))
        shim_threads[shimthread].start()

    ready = np.zeros((len(shim_threads)))  # Bool array for if shimmers are setup and streaming
    alive = np.zeros((len(shim_threads)))  # Bool array for if shimmer thread are active
    conn = np.zeros((len(shim_threads)))  # Bool array for if connections are successful
    s_down = np.zeros((len(shim_threads)))  # Bool array for if sensors are shutdown
    battery_levels = np.zeros((len(shim_threads)))
    time.sleep(1)

    print("Starting main loop")
    status = [2, 2, 2, 2] # [unknown, unknown, unknown, setting up]
    diag_level = 1 # 0:ok, 1:warning, 2:error, 3:stale
    feedback_timer = time.time()

    while not quit_IMU:
        new_data = np.empty((0), dtype=np.float64)
        for p in shimmers:
            new_data = np.hstack((new_data, shimmers[p]._accel, shimmers[p]._gyro))
        new_data = np.nan_to_num(new_data)
        IMU_pub_obj.publish(new_data)

        if time.time()-feedback_timer > 1:
            status[3] = 2 # setting up
            for s in shimmers:
                ready[s] = shimmers[s]._ready
                alive[s] = shim_threads[s].is_alive()
                conn[s] = shimmers[s]._connected
                s_down[s] = shimmers[s]._shutdown
                status[s] = shimmers[s]._status
                battery_levels[s] = shimmers[s]._batt_perc
                keyvalues = [KeyValue(key = f'Shimmer {POSITIONS[s]} {SHIM_IDs[s]}', value = IMU_MSGS[status[s]])]

            if all(ready) & all(conn) & all(alive):
                status[3] = 1 # Ready
                diag_level = 0 # ok
            else:
                diag_level = 1 # warning

            out_str = f"Sensors Ready:{ready} Threads:{alive} Connections:{conn} Shutdowns:{s_down} " \
                       f"Total Threads:{threading.active_count()} Quit:{quit_IMU} Battery: {battery_levels}"
            print(out_str)

            diag_msg = "Some helpful message"
            keyvalues = [KeyValue(key = f'Shimmer {POSITIONS[0]} {SHIM_IDs[0]}', value = IMU_MSGS[status[0]]),
                         KeyValue(key = f'Shimmer {POSITIONS[1]} {SHIM_IDs[1]}', value = IMU_MSGS[status[1]]),
                         KeyValue(key = f'Shimmer {POSITIONS[2]} {SHIM_IDs[2]}', value = IMU_MSGS[status[2]]),
                         KeyValue(key = f'Overall', value = IMU_SYS_MSGS[status[3]])]
            diag_obj.publish(diag_level, diag_msg, keyvalues)
            feedback_timer = time.time()

        rate.sleep()


if __name__ == "__main__":
    rospy.on_shutdown(shutdown_imu)
    try:
        IMUsensorsMain()
    except rospy.ROSInterruptException:
        quit_IMU = True
        print("Keyboard Interrupt")
        plt.close('all')
    finally:
        threads_alive = True
        while threads_alive:
            threads_alive = False
            for s in shim_threads:
                threads_alive = threads_alive | shim_threads[s].is_alive()
            time.sleep(0.01)

        if args.disp:
            plt.show()
        quit_IMU = True
        ready = np.zeros((len(shim_threads)))  # Bool array for if shimmers are setup and streaming
        alive = np.zeros((len(shim_threads)))  # Bool array for if shimmer thread are active
        conn = np.zeros((len(shim_threads)))  # Bool array for if connections are successful
        s_down = np.zeros((len(shim_threads)))  # Bool array for if sensors are shutdown

        for s in shimmers:
            ready[s] = shimmers[s]._ready
            alive[s] = shim_threads[s].is_alive()
            conn[s] = shimmers[s]._connected
            s_down[s] = shimmers[s]._shutdown

        print("###---End Status---###")
        print(f"Sensors Ready:{ready} Threads:{alive} Connections:{conn} Shutdowns:{s_down} "
              f"Threads:{threading.active_count()} Quit:{quit_IMU}")

        print("All done")
