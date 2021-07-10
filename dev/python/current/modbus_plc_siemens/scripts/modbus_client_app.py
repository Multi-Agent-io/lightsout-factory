#!/usr/bin/env python
from modbus_plc_siemens.r_api import *
from modbus_plc_siemens.algorithms import *
from modbus_plc_siemens.client_init import ModbusClient


#########################################

if __name__ == "__main__":

    #####################
    #       Init        #
    #####################

    rospy.init_node("modbus_client_app")

    modbus_host = "192.168.0.199"
    modbus_port = 502

    modclient = None

    try:
        modclient = ModbusClient(modbus_host, modbus_port)
        r_print("Modbus client session successfully started")
    except:
        r_error("Modbus client session failed to start")
        exit()

    r_sleep(1)

    # in_ports = modclient.readRegisters(0, 112), (1, 112)
    r_print("Press any button on FT to proceed")

    while not in_ports:
        r_sleep(0.2)

    # r_print(out_ports)

    #####################
    #   Application     #
    #####################

    r_print("Available commands: 1, 2, 3, 4, stop")

    command = None

    while command != "stop":
        command = input("Command: ")

        try:
            if command == "1":
                r_post(run_a)
            elif command == "2":
                r_post(run_b)
            elif command == "3":
                r_post(run_c)
            elif command == "4":
                r_post(run_d)
            else:
                exec(command)
        except:
            pass

    #####################
    #       Exit        #
    #####################

    r_print("Shutting down modbus client session...")

    try:
        rospy.signal_shutdown("Server shutting down")
    except:
        pass

    modclient.stopListening()
