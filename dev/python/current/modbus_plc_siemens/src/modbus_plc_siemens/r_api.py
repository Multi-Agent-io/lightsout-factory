import psycopg2 as pg
import rospy

from std_msgs.msg import Int32MultiArray as HoldingRegister
from modbus.process import ProcessWrapper
from modbus.post_threading import Post
from contextlib import closing

##########################################
#           ROS and FT methods           #
##########################################


in_ports = []
out_ports = [0] * 106

action_threads = []


class RApi:
    # noinspection PyUnresolvedReferences
    from modbus_plc_siemens.algorithms import act0, act1, act2, act3, \
                                              show_warehouse, clear_warehouse, add_tasks, run_factory, stop_factory, \
                                              run_loader_0, run_loader_1, run_lights, \
                                              run_line_a, run_line_b, run_line_c, run_line_d

    th = None

    def __init__(self):
        self.post = Post(self)

    ######################################

    def s(self):
        self.post.set(28, 1, time=100)

    # def r_kill(th):
    #     th

    def test(self):
        p = ProcessWrapper()
        p.run(self.set(28, time=100))

    # def r_post(command):
    #     """
    #         Run command in a new thread
    #         :param command: command to run
    #         :type command: str
    #     """
    #     th =

    ######################################
    #          Main FT methods           #
    ######################################

    @staticmethod
    def print(msg):
        """
            Print ROS' message
            :param msg: message to print
            :type msg: str
        """
        rospy.loginfo(msg)

    @staticmethod
    def error(msg):
        """
            Print ROS' error-message
            :param msg: message to print
            :type msg: str
        """
        rospy.logerr(msg)

    ######################################

    @staticmethod
    def sleep(time):
        """
            Sleep timer
            :param time: seconds to sleep
            :type time: float
        """
        rospy.sleep(time)

    @staticmethod
    def wait():
        """
            Wait until 'Ctrl-C' is pressed
        """
        rospy.spin()

    ######################################

    @staticmethod
    def get(port):
        """
            Get input port value
            :param port: input port number
            :type port: int in range (0, 112)
        """
        # r_print(in_ports)
        return in_ports[0][port]

    def send(self, value=None):
        """
            Send output ports values
            :param value: output ports values to send
            :type value: list in range (0, 112)
        """
        output = self.output

        if value is None:
            output.data = out_ports
        else:
            output.data = value

        self.pub.publish(output)

    def set(self, port, sensor=None, time=None, value=1):
        """
            Set specified output port value
            :param port: output port number
            :type port: int in range (0, 106)
            :param sensor: sensor port number
                           resets port value after sensor value changed
            :type sensor: int in range (0, 112)
            :param time: seconds to reset port value
            :type time: float
            :param value: value to set
            :type value: int in range (0, 2)
        """
        global out_ports

        # if (sensor is not None) and (self.get(sensor) != value):
        out_ports[port] = value
        self.send(out_ports)

        if value == 0:
            return
        elif sensor:
            value = self.get(sensor)
            while self.get(sensor) == value:
                self.sleep(0.001)
        elif time:
            self.sleep(time)
        else:
            return

        out_ports[port] = 0
        self.send(out_ports)

    ######################################

    @staticmethod
    def execute(query, result=False):
        """
            Execute SQL-query
            :param query: query to execute
            :type query: str
            :param result: label to return executing result
            :type result: bool
        """
        with closing(pg.connect(user='postgres', password='panda', host='localhost', database='postgres')) as conn:
            with conn.cursor() as cursor:
                cursor.execute(query)
                conn.commit()
                if result:
                    return cursor.fetchall()

    ######################################
    #           ROS Subscriber           #
    ######################################

    # noinspection PyMethodParameters
    def __in_ports_update(msg):
        """
            Service callback-function:
            Read input ports and write them to list
            Make input ports available in app
        """
        global in_ports

        del in_ports[0: len(in_ports)]
        # noinspection PyUnresolvedReferences
        in_ports.append(msg.data)

    sub = rospy.Subscriber("modbus_wrapper/input",
                           HoldingRegister,
                           __in_ports_update,
                           queue_size=500)

    ######################################
    #           ROS Publisher            #
    ######################################

    pub = rospy.Publisher("modbus_wrapper/output",
                          HoldingRegister,
                          queue_size=500)

    output = HoldingRegister()
