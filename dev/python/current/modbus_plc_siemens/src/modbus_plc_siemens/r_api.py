import psycopg2 as pg
import rospy

from std_msgs.msg import Int32MultiArray as HoldingRegister
from modbus.process import ProcessWrapper
from modbus.post_threading import Post
from contextlib import closing

######################################
#         ROS and FT methods         #
######################################


in_ports = []
out_ports = [0] * 106

action_threads = []


class RApi:

    # noinspection PyUnresolvedReferences
    from modbus_plc_siemens.algorithms import show_warehouse, run_lights, run_a, run_b, run_c, run_d
    # noinspection PyUnresolvedReferences
    from modbus_plc_siemens.algorithms import act0, act1, act2, act3

    th = None


    def __init__(self):
        self.post = Post(self)

    def s(self):
        self.post.set(28, 1, time=100)
        
    @staticmethod
    def execute(query, result=False):
        with closing(pg.connect(user='postgres', password='panda', host='localhost', database='postgres')) as conn:
            with conn.cursor() as cursor:
                cursor.execute(query)
                conn.commit()
                if result:
                    table = []
                    for line in cursor:
                        table.append(line)
                    return table

    @staticmethod
    def error(msg):
        """
            Print ROS error message
            :param msg: Message to print
            :type msg: String
        """
        rospy.logerr(msg)

    @staticmethod
    def get(port):
        """
            Get input port value
            :param port: Input port number
            :type port: int in range (0, 112)
        """
        # r_print(in_ports)
        return in_ports[0][port]

    # def r_kill(th):
    #     th

    def test(self):
        p = ProcessWrapper()
        p.run(self.set(28, time=100))

    @staticmethod
    def print(msg):
        """
            Print ROS message
            :param msg: Message to print
            :type msg: String
        """
        rospy.loginfo(msg)

    # def r_post(command):
    #     """
    #         Run command in a new thread
    #         :param command: Command to run
    #         :type command: String
    #     """
    #     th =

    def send(self, value=None):
        """
            Set output ports
            :param value: Output ports' value list
            :type value: list in range (0, 112)
        """
        output = self.output

        if value is None:
            output.data = out_ports
        else:
            output.data = value

        self.pub.publish(output)

    def set(self, port, sensor=None, value=1, time=None):
        """
            Set specified output port
            :param port: Register number
            :type port: int in range (0, 106)
            :param sensor: Sensor port number
                            Resets port value after sensor value changed
            :type sensor: int in range (0, 112)
            :param value: Value to set
            :type value: int (0 or 1 only)
            :param time: Timer to revert value
            :type time: float
        """
        global out_ports

        # if (sensor is not None) and (self.get(sensor) != value):
        if sensor is not None:
            out_ports[port] = value
            self.send(out_ports)

            value = self.get(sensor)

            while self.get(sensor) == value:
                self.sleep(0.001)

            out_ports[port] = 1 if out_ports[port] == 0 else 0
            self.send(out_ports)

        if time is not None:
            out_ports[port] = value
            self.send(out_ports)

            self.sleep(time)

            out_ports[port] = 1 if out_ports[port] == 0 else 0
            self.send(out_ports)

        if sensor is None and time is None:
            out_ports[port] = value
            self.send(out_ports)

    @staticmethod
    def sleep(time):
        """
            Sleep timer
            :param time: Seconds to sleep
            :type time: float
        """
        rospy.sleep(time)

    @staticmethod
    def wait():
        """
            Wait until Ctrl-C is pressed
        """
        rospy.spin()

    ######################################
    #           ROS Subscriber           #
    ######################################

    # noinspection PyMethodParameters
    def __in_ports_update(msg):
        """
            Service func to read input ports
            and write them to in_ports list
            to make them available in app
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
