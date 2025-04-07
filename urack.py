import minimalmodbus
import serial

class BMSReader:
    def __init__(self, port, slave_address):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.instrument.serial.baudrate = 19200
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_NONE
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 5
        self.instrument.mode = minimalmodbus.MODE_RTU

    def read_registers(self, starting_register, number_of_registers):
        try:
            values = self.instrument.read_registers(starting_register, number_of_registers)
            for i, value in enumerate(values):
                print(f"Register {starting_register + i}: {value}")
            return values
        except IOError as e:
            print(f"Failed to read registers from {starting_register} to {starting_register + number_of_registers - 1}: {e}")
            return None

if __name__ == "__main__":
    port = '/dev/ttyUSB0'  # Change this to your BMS port
    slave_address = 181      # Change this to your BMS slave address

    bms_reader = BMSReader(port, slave_address)
    
    # Example usage
    starting_register = 0       # Change this to your starting register address
    number_of_registers = 32    # Change this to the number of registers you want to read

    bms_reader.read_registers(starting_register, number_of_registers)
