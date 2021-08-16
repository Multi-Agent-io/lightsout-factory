#!/usr/bin/env python
import sys
from contextlib import suppress

import psycopg2
from modbus_plc_siemens.r_api import *
from modbus_plc_siemens.client_init import ModbusClient


##################################################################################

if __name__ == "__main__":

    ######################################
    #           Initialisation           #
    ######################################

    rospy.init_node("modbus_client_app")

    modbus_host = "192.168.0.199"
    modbus_port = 502

    modclient = None
    conn = None
    R = RApi()

    try:
        modclient = ModbusClient(modbus_host, modbus_port)
        R.print("Modbus client session successfully started")
        conn = psycopg2.connect(user='postgres',
                                password='panda',
                                host='localhost',
                                database='postgres')
        R.query = conn.cursor()
    except Exception as e:
        R.error("Modbus client session failed to start")
        R.error("[REASON] " + str(e))
        exit()

    R.sleep(1)

    # in_ports = modclient.readRegisters(0, 112), (1, 112)
    R.print("Press any button on FT to proceed")

    while not in_ports:
        R.sleep(0.2)

    # r_print(out_ports)


    ######################################
    #            Application             #
    ######################################

    R.print("Available commands: 1, 2, 3, 4, stop")

    command = None

    while command != "stop":
        command = input("Command: ")

        try:
            # if command == "1":
            #     R.post(run_a)
            # elif command == "2":
            #     R.post(run_b)
            # elif command == "3":
            #     R.post(run_c)
            # elif command == "4":
            #     R.post(run_d)
            # else:
            #     exec(command)
            if command == "stop":
                sys.tracebacklimit = 0

                with suppress(Exception):
                    R.query.close()
                    conn.close()
                    
                    R.print("Shutting down modbus client session...")
                    rospy.signal_shutdown("Server shutting down")
                    exit(0)

            else:
                exec(command)

        except Exception as e:
            R.error("[REASON] " + str(e))

    #
    # #####################
    # #       Exit        #
    # #####################
    #
    # R.print("Shutting down modbus client session...")
    #
    # try:
    #     rospy.signal_shutdown("Server shutting down")
    # except:
    #     pass
    #
    # modclient.stopListening()
