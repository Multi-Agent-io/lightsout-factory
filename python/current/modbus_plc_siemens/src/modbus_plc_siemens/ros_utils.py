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


#########################################

#########################################

def pickup_block(way):
    # move to necessary column (column №3)
    r_set(3, 1, 6)
    # move to necessary cell (line №4)
    r_set(5, 1, 23)
    # pickup the block
    r_set(8, 1, 25)
    r_set(5, 1, 24)
    r_set(7, 1, 26)
    # deliver the block
    if way == 1:
        r_set(2, 1, 99)
    elif way == 2:
        r_set(3, 1, 100)
    elif way == 3:
        r_set(3, 1, 101)
    elif way == 4:
        r_set(3, 1, 102)

    r_set(7, 1, 27)
    r_set(6, 1, 17)
    r_set(8, 1, 26)

    # return to starting position
    r_post(r_set, (2, 1, 4))


# ---------------------------------------

def put_block():
    # pickup the block
    r_set(16, 1, 96)
    r_set(13, 1, 89)
    r_set(15, 1, 97)
    # move to necessary column (column №9)
    r_set(10, 1, 83)
    # move to necessary cell (line №4)
    r_set(13, 1, 95)
    # put the block
    r_set(15, 1, 98)
    r_set(14, 1, 94)
    r_set(16, 1, 97)
    # return to starting position
    r_set(14, 1, 88)
    r_set(11, 1, 86)


# ---------------------------------------

def run_a():
    pickup_block(1)
    # ----------------------------
    r_post(r_set, (19, 1, 33))
    r_set(21, 1, 33)

    # handler work
    r_set(22, 1, 34)
    r_set(25, 1, 36)
    r_set(26, 1, time=1)
    r_set(24, 1, 37)
    r_set(23, 1, 35)

    if r_get(39) == 1:
        r_set(31, 1, 39)

    r_post(r_set, (21, 1, 38))
    r_set(33, 1, 38)

    r_set(30, 1, 40)
    # ----------------------------

    r_post(r_set, (33, 1, 41))
    r_set(35, 1, 41)

    if r_get(50) == 1:
        r_set(50, 1, 50)

    r_post(r_set, (35, 1, 48))
    r_set(52, 1, 48)

    r_post(r_set, (52, 1, 51))
    r_set(54, 1, 51)

    if r_get(60) == 1:
        r_set(68, 1, 60)

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
    r_post(r_set, (38, 1, 43))
    r_set(40, 1, 43)

    # handler work
    r_set(41, 1, 44)
    r_set(43, 1, 46)
    r_set(45, 1, time=1)
    r_set(44, 1, 47)
    r_set(42, 1, 45)

    if r_get(49) == 1:
        r_set(49, 1, 49)

    r_post(r_set, (40, 1, 48))
    r_set(52, 1, 48)

    r_set(50, 1, 50)
    # ----------------------------

    r_post(r_set, (52, 1, 51))
    r_set(54, 1, 51)

    if r_get(60) == 1:
        r_set(68, 1, 60)

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

def run_c():
    pickup_block(3)
    # ----------------------------
    r_post(r_set, (57, 1, 53))
    r_set(59, 1, 53)

    # handler work
    r_set(60, 1, 64)
    r_set(62, 1, 56)
    r_set(64, 1, time=1)
    r_set(63, 1, 57)
    r_set(61, 1, 65)

    if r_get(59) == 1:
        r_set(69, 1, 59)

    r_post(r_set, (59, 1, 58))
    r_set(71, 1, 58)

    r_set(68, 1, 60)
    # ----------------------------

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

def run_d():
    pickup_block(4)
    # ----------------------------
    r_post(r_set, (76, 1, 63))
    r_set(78, 1, 63)

    # handler work
    r_set(79, 1, 64)
    r_set(81, 1, 66)
    r_set(83, 1, time=1)
    r_set(96, 1, time=1)
    r_set(82, 1, 67)
    r_set(80, 1, 65)

    if r_get(69) == 1:
        r_set(88, 1, 69)

    r_post(r_set, (78, 1, 68))
    r_set(90, 1, 68)

    r_post(r_set, (90, 1, 72))
    r_set(93, 1, 72)

    r_set(30, 1, 70)
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))
    r_set(95, 1, 73)
    # ----------------------------
    put_block()