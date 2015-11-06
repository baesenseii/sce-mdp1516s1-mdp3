from constants import *
from simulator import ArenaMap, get_filled_list
import time


class PhysicalRobot:
    def __init__(self):
        self.command_now = CM_UNKNOWN
        self.command_next = CM_UNKNOWN
        self.command_id = 0
        self.command_done = False
        self.sensor_reading_ideal = [315, 320, 303, 325, 515, 490]  # reading from init_align
        self.sensor_reading_now = get_filled_list(0, NUM_SENSORS)  # current reading of sensors
        self.sensor_dev_now = get_filled_list(0, NUM_SENSORS)  # current deviation in reading
        self.reading_time = time.time()

    def register_packet(self, means, devs, command_now, command_next, command_id, command_done):
        self.command_now = command_now
        self.command_next = command_next
        self.command_id = command_id
        self.command_done = command_done
        self.sensor_reading_now = means
        self.sensor_dev_now = devs
        self.reading_time = time.time()

    @staticmethod
    def is_clean(reading, ideal):
        # test first cell using SR sensor
        return ideal - reading > 60  # 60 is the tolerance

    @staticmethod
    def get_R0R_identity(reading_sr, ideal_sr, reading_lr, ideal_lr):
        dist_sr = ideal_sr - reading_sr
        dist_lr = ideal_lr - reading_lr
        print reading_sr, ideal_sr, reading_lr, ideal_lr
        print dist_sr, dist_lr
        if dist_sr > 160:
            return C
        if dist_sr <= 60:
            return U
        if dist_sr < 100:  # our right-lr only starts working at distance > 1.5
            return O

        if dist_lr < 65:
            return O
        return C

    def get_L0L_value(self):
        reading_sr = self.sensor_reading_now[3]  # short range sensor
        dist_sr = self.sensor_reading_ideal[3] - reading_sr
        reading_lr = self.sensor_reading_now[4]  # long range sensor
        dist_lr = self.sensor_reading_ideal[4] - reading_lr
        if dist_sr > 160:  # definitely clear
            return C
        if dist_sr <= 60:  # L0 is O
            return U

        # now, 60< dist_sr <= 150, we use long range to decide
        if dist_lr < 160:
            return O
        return C

    def get_L0LL_value(self):
        reading_lr = self.sensor_reading_now[4]  # long range sensor
        dist_lr = self.sensor_reading_ideal[4] - reading_lr

        if 160 <= dist_lr < 200:
            return O
        elif dist_lr > 260:
            return C
        return U

    def get_ahead_vals(self):
        first_4_reading = self.sensor_reading_now[:4]
        first_4_ideal   = self.sensor_reading_ideal[:4]
        first_4 = [C if PhysicalRobot.is_clean(reading, ideal) else O
                   for reading, ideal in zip(first_4_reading, first_4_ideal)]
        l0l  = self.get_L0L_value()
        l0ll = self.get_L0LL_value()
        true_values = first_4 + [l0l, l0ll]
        if true_values[3] == O:
            true_values[4] = U
            true_values[5] = U
        if true_values[4] == O:
            true_values[5] = U
        return true_values


class GlobalState(object):

    def __init__(self, bypass_connections=False):

        if not bypass_connections:
            # arduino serial
            self.serial = None  # serial object
            self.serial_okay = False  # serial status
            self.serial_sync = 'none'  # synchronization status
            self.serial_remain = 0  # number of bytes remaining in a packet.
            self.serial_normal_count = 0

            # wifi sockets
            self.socket = None  # socket used for WiFi monitor
            self.socket_cons = []  # list of socket connections

        # command system
        self.command_monitor = None  # command from monitor about to be sent
        self.command_algo = None  # command from algo about to be sent
        self.command_id = 0
        self.command_sequence = None

        # physical robot
        self.physical_robot = PhysicalRobot()
        """:type : PhysicalRobot"""

        # algo states
        self.arena = ArenaMap(self.physical_robot, self)
        """:type : ArenaMap"""

        # autonomous mode
        self.autonomous = False

        # miscellaneous
        self.error_list = []  # list of errors, created via register_error
        self.loop_counter = 0


g = GlobalState()
