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
    R.print("(Press any button on FT to proceed)")

    while not in_ports:
        R.sleep(0.2)

    # r_print(out_ports)

    ######################################
    #            Application             #
    ######################################

    R.print("Available commands: show, clear, add*(.), run, stop, exit (act*(.), set(), color)")

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
            elif command == "color":
                print(f'- Color: {R.get(0)}')
            elif command.startswith(("add", "act", "set")):
                exec('R.' + command)
            elif command == "exit":
                
                # application closing
                sys.tracebacklimit = 0
                with suppress(Exception):
                    R.print("Shutting down modbus client session...")
                    rospy.signal_shutdown("Server shutting down")
                    # modclient.stopListening()

            else:
                # print('- This command is not available!')
                exec(command)

        except Exception as e:
            R.error("[REASON] " + str(e))
