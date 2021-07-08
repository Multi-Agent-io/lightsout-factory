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
        :type time: float
    """
    global out_ports

    if sensor is not None:
        out_ports[port] = value
        r_send(out_ports)

        value = r_get(sensor)

        while r_get(sensor) == value:
            r_sleep(0.001)

        out_ports[port] = 1 if out_ports[port] == 0 else 0
        r_send(out_ports)

    if time is not None:
        out_ports[port] = value
        r_send(out_ports)

        r_sleep(time)

        out_ports[port] = 1 if out_ports[port] == 0 else 0
        r_send(out_ports)

    if sensor is None and time is None:
        out_ports[port] = value
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

