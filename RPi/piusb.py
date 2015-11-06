import serial
import select
import socket
from time import sleep
# noinspection PyUnresolvedReferences
import time
import traceback
# noinspection PyUnresolvedReferences
import os  # , commands
import json
# noinspection PyUnresolvedReferences
from globalVars import *
from algorithm import step_algorithm
from constants import *


def register_error(name, obj):
    global g
    g = g
    """:type : GlobalState"""
    print name
    print obj


def serial_init():
    global g
    g = g
    """:type : GlobalState"""
    if g.serial is not None:
        g.serial.close()
    g.serial = serial.Serial('/dev/ttyACM0', 115200)
    g.serial_okay = True


def socket_init():
    global g
    g = g
    """:type : GlobalState"""

    if g.socket is not None:
        g.socket.close()
        g.socket = None

    g.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    g.socket.setblocking(0)
    addr = "0.0.0.0"
    port = PORT
    g.socket.bind((addr, port))
    g.socket.listen(10)
    print "socket listening at {}:{}".format(addr, port)


def process_socket():
    global g
    g = g
    """:type : GlobalState"""
    if g.socket is None:
        return
    readables, writables, exceptionals = select.select(
        g.socket_cons + [g.socket],
        [], [], 0)
    for r in readables:
        if r is g.socket:  # establish new connection
            con, addr = g.socket.accept()
            print "Connection accepted at " + str(addr)
            con.setblocking(0)
            g.socket_cons.append(con)
        else:  # existing connection
            try:
                data = r.recv(1024)
                if data:  # readable socket, receive commands
                    process_monitor_command(data, r)
                else:  # unreadable socket, close it
                    g.socket_cons.remove(r)
                    r.close()
                    print "one connection closed"
            except Exception, e:
                print "Lost a connection"
                g.socket_cons.remove(r)


def get_robot_status():
    global g
    g = g
    """:type : GlobalState"""
    return {
        "map": g.arena.get_encoded_map(),
        "robot": {
            "x": g.arena.get_robot_coord()[0],
            "y": g.arena.get_robot_coord()[1],
            "degree": g.arena.get_robot_degree()
        },
        "state": g.arena.robot_state,
        "timing": {
            "explore": g.arena.get_time_explore(),
            "shortest": g.arena.get_time_shortest()
        }
    }


def process_monitor_command(data, con):
    global g
    g = g
    """:type : GlobalState"""
    data = data[0]
    # start signals
    if data == chr(MCM_INIT_BALANCE):  # start initial balancing
        g.arena.permission_balance = True
    elif data == chr(MCM_INIT_ALIGN):  # start initial aligning
        g.arena.permission_align = True
    elif data == chr(MCM_START_EXPLORE):  # start explore
        g.arena.permission_explore = True
    elif data == chr(MCM_START_SHORTEST):  # start shortest path
        g.arena.permission_shortest = True

    # direct robot commands
    elif data == chr(MCM_FORWARD):  # forward
        g.command_monitor = CM_FORWARD
    elif data == chr(MCM_BACK):  # back
        g.command_monitor = CM_BACK
    elif data == chr(MCM_LEFT):  # left
        g.command_monitor = CM_LEFT
    elif data == chr(MCM_RIGHT):  # right
        g.command_monitor = CM_RIGHT
    elif data == chr(MCM_CM_INIT_ALIGN):  # init align
        g.command_monitor = CM_INIT_ALIGN
    elif data == chr(MCM_CM_ALIGN):  # align
        g.command_monitor = CM_ALIGN
    elif data == chr(MCM_CM_INIT_BALANCE):  # init balance
        g.command_monitor = CM_INIT_BALANCE

    elif MCM_CM_MULTI_FORWARD <= ord(data) < MCM_CM_MULTI_FORWARD + 31:
        g.command_monitor = ord(data)

    # others
    elif data == chr(MCM_ABORT):  # abort
        g.command_monitor = CM_ABORT
    elif data == chr(MCM_REQUEST):  # request data
        con.send(json.dumps(get_robot_status()))
    elif data == chr(MCM_CLOSE_ALL):
        for con in g.socket_cons:
            con.close()
        g.socket.close()
        exit()


def process_monitors():
    global g
    g = g
    """:type : GlobalState"""
    process_socket()
    pass


def register_packet_normal(packet_buf):
    global g
    g = g
    """:type : GlobalState"""

    g.serial_normal_count += 1

    code = map(ord, packet_buf)
    command_id = code[0]
    command_now = code[1]
    command_next = code[2]
    command_done = code[3]

    offset = 4
    means = []
    devs = []
    for i in range(NUM_SENSORS):
        means.append(code[offset + 2 * i] + code[offset + 2 * i + 1] * 256)
    offset += NUM_SENSORS * 2
    for i in range(NUM_SENSORS):
        devs.append (code[offset + 2 * i] + code[offset + 2 * i + 1] * 256)

    if g.serial_normal_count % 30 == 0:
        print code
        print means
        print devs

    g.physical_robot.register_packet(means, devs, command_now, command_next, command_id, command_done)


def register_packet_debug(packet_buf):
    print ('DEBUG packet received:')
    print (''.join(packet_buf))


serial_buf = []
sync_count = 0  # number of 254 received


def read_serial():
    global g, serial_buf, sync_count
    g = g
    """:type : GlobalState"""
    s = g.serial
    if g.serial_okay:
        try:
            num_avail = s.inWaiting()
            for i in range(num_avail):
                char = s.read()
                byte = ord(char)
                # 8 consecutive 254 is a synchronization sequence
                if byte == 254:
                    sync_count += 1
                    if sync_count == 8:
                        serial_buf = []
                        g.serial_sync = 'end'
                        continue
                else:
                    sync_count = 0
                # not yet synchronized
                if g.serial_sync == 'none':
                    continue
                # synchronized, expecting new packet header
                elif g.serial_sync == 'end':
                    if byte == 0x40:  # normal packet
                        g.serial_sync = 'normal_header'
                        continue
                    elif byte == 0x80:  # debug packet
                        g.serial_remain = -1
                        g.serial_sync = 'middle'
                        continue
                    elif byte == 254:
                        continue
                    else:
                        register_error('out of sync', (byte, serial_buf))
                        serial_buf = []
                        g.serial_sync = 'none'
                        continue
                # normal header received, expecting a byte indicating packet length
                elif g.serial_sync == 'normal_header':
                    g.serial_remain = byte
                    g.serial_sync = 'middle'
                # in the middle of a packet.
                elif g.serial_sync == 'middle':
                    serial_buf.append(char)
                    # If serial_remain is -1, it is a 0-terminated debug packet
                    if g.serial_remain == -1:
                        if byte == 0:
                            register_packet_debug(serial_buf)
                            serial_buf = []
                            g.serial_sync = 'end'
                            return
                    # otherwise, this is a normal packet
                    else:
                        g.serial_remain -= 1
                        if g.serial_remain == 0:
                            register_packet_normal(serial_buf)
                            serial_buf = []
                            g.serial_sync = 'end'
                            return

            return
        except Exception, e:
            register_error('Serial broke half-way', str(e))
            print traceback.format_exc()
            g.serial_okay = False
    try:
        serial_init()
    except Exception, e:
        register_error('Serial initialization failed', str(e))
        g.serial_okay = False


def run_algo():
    """
    This is the main algorithm driver that
    1. wait for physical_robot.command_now to be CM_WAIT
    2. run next command
    """
    global g
    g = g
    """:type : GlobalState"""
    if ( g.physical_robot.command_now is not CM_WAIT or     # if robot is not in waiting
            g.physical_robot.command_id != g.command_id ):  # if previously given command not executed
        return
    step_algorithm(g.arena)
    # after step_algorithm, g.command_algo may be set
    if g.command_algo is not None:
        g.command_id += 1
        g.command_id %= 256
    """
    if (previous_command_counter > 0 or
            g.physical_robot.command_now is not CM_WAIT or
            time.time() - g.physical_robot.reading_time > 0.2 ):
        return
    step_algorithm(g.arena)
    # after step_algorithm, g.command_algo may be set
    if g.command_algo is not None:
        previous_command_counter = 40  # wait for 1/5 second before we proceed to next step
    """


def send_command():
    global g
    g = g
    """:type " GlobalState"""

    if not g.serial_okay:
        return

    command_to_give = None

    if g.command_algo is not None:
        command_to_give = g.command_algo
        g.command_algo = None
    elif g.command_monitor is not None:
        command_to_give = g.command_monitor
        g.command_monitor = None

    if g.command_sequence is not None:
        try:
            g.serial.write(chr(0x64))
            for c in g.command_sequence:
                g.serial.write(chr(c))
            g.serial.write(chr(0))
            g.command_sequence = None

        except Exception, e:
            register_error('serial write failed', str(e))
            g.serial_okay = False

    if command_to_give is not None:
        try:
            g.serial.write(chr(command_to_give))
            g.serial.write(chr(g.command_id))
            g.serial.write(chr(0))
        except Exception, e:
            register_error('serial write failed', str(e))
            g.serial_okay = False


def loop():
    global g
    g = g
    """:type : GlobalState"""
    while True:
        process_monitors()
        read_serial()

        g.autonomous = (time.time() - g.physical_robot.reading_time) > 60

        if g.serial_okay:
            run_algo()
            send_command()
        g.loop_counter += 1
        sleep(0.005)  # 200Hz


socket_init()
serial_init()

sleep(2)  # wait for arduino to initialize

loop()
