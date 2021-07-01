import sys

from modbus.modbus_wrapper import ModbusWrapperClient


class ModbusClient(ModbusWrapperClient):
    def __init__(self, host, port=502, rate=50, reset_registers=True):
        """
            :param host: Contains the IP address of the modbus server
            :type host: string
            :param rate: How often the registers on the modbus server should be read per second
            :type rate: float
            :param reset_registers: Defines if the holding registers should be reset to 0 after they have been read. Only possible if they are writeable
            :type reset_registers: bool
        """

        ModbusWrapperClient.__init__(self, host, port, rate, reset_registers)
        self.setReadingRegisters(0, 112)
        self.setWritingRegisters(112, 106)
        self.startListening()

        self.readRegisters(0, 1)
