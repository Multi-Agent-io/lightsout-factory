#!/usr/bin/env python
import sys

from contextlib import suppress
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
    R = RApi()

    try:
        modclient = ModbusClient(modbus_host, modbus_port)
        R.print("Modbus client session successfully started")

    except Exception as e:
        R.error("Modbus client session failed to start")
        R.error("[REASON] " + str(e))
        exit(1)

    R.sleep(1)

    # in_ports = modclient.readRegisters(0, 112), (1, 112)
    R.print("Press any button on FT to proceed")

    while not in_ports:
        R.sleep(0.2)

    # r_print(out_ports)

    ######################################
    #            Application             #
    ######################################

    R.print("Available commands: show, clear, act*(...), add(...), run, stop, exit")

    command = None

    while command != "exit":
        command = input("Command: ")

        try:
            if command == "show":
                R.show_warehouse()
            elif command == "clear":
                R.clear_warehouse()
            elif command == "run":
                R.run_factory()
            elif command == "stop":
                R.stop_factory()
            # elif command.startswith(("act", "add", "set")):
                # exec('R.' + command)
            elif command == "exit":
                # sys.tracebacklimit = 0
                
                # application closing
                with suppress(Exception):
                    R.print("Shutting down modbus client session...")
                    rospy.signal_shutdown("Server shutting down")
                    # modclient.stopListening()

            else:
                exec(command)
                # if command.startswith(('R.', 'exec')):
                    # print('- Warning: this is unecceptable command!')
                    # continue
                # exec(command)

        except Exception as e:
            R.error("[REASON] " + str(e))
