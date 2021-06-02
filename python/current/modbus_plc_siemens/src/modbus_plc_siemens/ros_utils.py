import rospy

from threading import Thread
from std_msgs.msg import Int32MultiArray as HoldingRegister


######################
# ROS and FT methods #
######################

in_ports = []
out_ports = [0 for i in range(106)]


def r_error(msg):
    """
        Print ROS error message
        :param msg: Message to print
        :type msg: String
    """
    rospy.logerr(msg)


def r_get(port):
    """
        Get input port value
        :param port: Input port number
        :type port: int in range (0, 112)
    """
    # r_print(in_ports)
    return in_ports[0][port]


def r_print(msg):
    """
        Print ROS message
        :param msg: Message to print
        :type msg: String
    """
    rospy.loginfo(msg)


def r_post(command, args=None):
    """
        Run command in a new thread
        :param command: Command to run
        :type command: String
        :param args:
    """
    th = None

    if args is not None:
        th = Thread(target=command, args=args)
    else:
        th = Thread(target=command)

    th.daemon = True
    th.start()


def r_send(value=None):
    """
        Set output ports
        :param value: Output ports' value list
        :type value: list in range (0, 112)
    """
    global output

    if value is None:
        output.data = out_ports
    else:
        output.data = value

    pub.publish(output)


def r_set(port, value=1, sensor=None, time=None):
    """
        Set specified output port
        :param port: Register number
        :type port: int in range (0, 106)
        :param value: Value to set
        :type value: int (0 or 1 only)
        :param sensor: Sensor port number
                       Resets port value after sensor value changed
        :type port: int in range (0, 112)
        :param time: Timer to revert value
        :type value: int
    """
    global out_ports

    out_ports[port] = value
    r_send(out_ports)

    if sensor is not None:
        value = r_get(sensor)

        while r_get(sensor) == value:
            r_sleep(0.001)

        out_ports[port] = 1 if out_ports[port] == 0 else 0
        r_send(out_ports)

    if time is not None:
        r_sleep(time)

        out_ports[port] = 1 if out_ports[port] == 0 else 0
        r_send(out_ports)


def r_sleep(time):
    """
        Sleep timer
        :param time: Seconds to sleep
        :type time: float
    """
    rospy.sleep(time)


def r_wait():
    """
        Wait until Ctrl-C is pressed
    """
    rospy.spin()


##################
# ROS Subscriber #
##################

def __in_ports_update(msg):
    """
        Service func to read input ports
        and write them to in_ports list
        to make them available in app
    """
    global in_ports

    del in_ports[0: len(in_ports)]
    in_ports.append(msg.data)

    # r_print(str(ports))


sub = rospy.Subscriber("modbus_wrapper/input",
                       HoldingRegister,
                       __in_ports_update,
                       queue_size=500)


#################
# ROS Publisher #
#################

pub = rospy.Publisher("modbus_wrapper/output",
                      HoldingRegister,
                      queue_size=500)
output = HoldingRegister()










def pickup_block(way):
    # move to necessary column (column №3)
    r_set(3, 1, 6)  # 115
    # move to necessary cell (line №4)
    r_set(5, 1, 23)  # 117
    # pickup the block
    r_set(8, 1, 25)  # 120
    r_set(5, 1, 24)  # 117
    r_set(7, 1, 26)  # 119
    # deliver the block
    if way == 1:
        r_set(2, 1, 99)  # 114
    elif way == 2:
        r_set(3, 1, 100)  # 115
    elif way == 3:
        r_set(3, 1, 101)  # 115
    elif way == 4:
        r_set(3, 1, 102)  # 115

    r_set(7, 1, 27)  # 119
    r_set(6, 1, 17)  # 118
    r_set(8, 1, 26)  # 120

    # return to starting position
    r_post(r_set, (2, 1, 4))  # 114


# ---------------------------------------

def put_block():
    # pickup the block
    r_set(16, 1, 96)  # 128
    r_set(13, 1, 89)  # 125
    r_set(15, 1, 97)  # 127
    # move to necessary column (column №9)
    r_set(10, 1, 83)  # 122
    # move to necessary cell (line №4)
    r_set(13, 1, 95)  # 125
    # put the block
    r_set(15, 1, 98)  # 127
    r_set(14, 1, 94)  # 126
    r_set(16, 1, 97)  # 128
    # return to starting position
    r_set(14, 1, 88)  # 126
    r_set(11, 1, 86)  # 123


# ---------------------------------------

def run_a():
    pickup_block(1)
    # ----------------------------
    r_post(r_set, (19, 1, 33))  # 131
    r_set(21, 1, 33)  # 133

    # handler work
    r_set(25, 1, 36)  # 137
    r_set(26, 1, time=1)  # 138
    r_set(24, 1, 37)  # 136

    r_set(31, 1, 39)  # 143

    r_post(r_set, (21, 1, 38))
    r_set(33, 1, 38)

    r_set(30, 1, 40)
    # ----------------------------

    r_post(r_set, (33, 1, 41))
    r_set(35, 1, 41)

    r_post(r_set, (35, 1, 48))
    r_set(52, 1, 48)

    r_post(r_set, (52, 1, 51))
    r_set(54, 1, 51)

    r_post(r_set, (54, 1, 58))
    r_set(71, 1, 58)

    r_post(r_set, (71, 1, 61))
    r_set(73, 1, 61)

    if r_get(70) == 1:
        r_set(87, 1, 70)

    r_post(r_set, (73, 1, 68))
    r_set(90, 1, 68)

    # ----------------------------
    r_set(88, 1, 69)

    r_post(r_set, (90, 1, 72))
    r_set(93, 1, 72)

    r_post(r_set, (87, 1, 70))
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))
    r_set(95, 1, 73)
    # ----------------------------
    put_block()


# ---------------------------------------

def run_b():
    pickup_block(2)
    # ----------------------------
    r_post(r_set, (38, 1, 43))  # 150
    r_set(40, 1, 43)  # 152

    # handler work
    r_set(43, 1, 46)  # 155
    r_set(45, 1, time=1)  # 157
    r_set(44, 1, 47)  # 156

    r_set(49, 1, 49)  # 161

    r_post(r_set, (40, 1, 48))  # 152
    r_set(52, 1, 48)  # 164

    r_set(50, 1, 50)  # 162
    # ----------------------------

    r_post(r_set, (52, 1, 51))  # 164
    r_set(54, 1, 51)  # 166

    r_post(r_set, (54, 1, 58))  # 166
    r_set(71, 1, 58)  # 183

    r_post(r_set, (71, 1, 61))  # 183
    r_set(73, 1, 61)  # 185

    r_post(r_set, (73, 1, 68))  # 185
    r_set(90, 1, 68)  # 202

    # ----------------------------
    r_set(88, 1, 69)  # 200

    r_post(r_set, (90, 1, 72))  # 202
    r_set(93, 1, 72)  # 205

    r_post(r_set, (87, 1, 70))  # 199
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))  # 205
    r_set(95, 1, 73)  # 207
    # ----------------------------
    put_block()


# ---------------------------------------

def run_c():
    pickup_block(3)
    # ----------------------------
    r_post(r_set, (57, 1, 53))  # 169
    r_set(59, 1, 53)  # 171

    # handler work
    r_set(62, 1, 56)  # 174
    r_set(64, 1, time=1)  # 176
    r_set(63, 1, 57)  # 175

    r_set(69, 1, 59)  # 181

    r_post(r_set, (59, 1, 58))  # 171
    r_set(71, 1, 58)  # 183

    r_set(68, 1, 60)  # 180
    # ----------------------------

    r_post(r_set, (71, 1, 61))  # 183
    r_set(73, 1, 61)  # 185

    r_post(r_set, (73, 1, 68))  # 185
    r_set(90, 1, 68)  # 202

    # ----------------------------
    r_set(88, 1, 69)  # 200

    r_post(r_set, (90, 1, 72))  # 202
    r_set(93, 1, 72)  # 205

    r_post(r_set, (87, 1, 70))  # 199
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))  # 205
    r_set(95, 1, 73)  # 207
    # ----------------------------
    put_block()


# ---------------------------------------

def run_d():
    pickup_block(4)
    # ----------------------------
    r_post(r_set, (76, 1, 63))  # 188
    r_set(78, 1, 63)  # 190

    # handler work
    r_set(81, 1, 66)  # 193
    r_set(83, 1, 103)  # 195
    r_set(96, 1, time=1)  # 208
    r_set(82, 1, 67)  # 194

    r_set(88, 1, 69)  # 200

    r_post(r_set, (78, 1, 68))  # 190
    r_set(90, 1, 68)  # 202

    r_post(r_set, (90, 1, 72))  # 202
    r_set(93, 1, 72)  # 205

    r_set(30, 1, 70)  # 199
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))  # 205
    r_set(95, 1, 73)  # 207
    # ----------------------------
    put_block()
