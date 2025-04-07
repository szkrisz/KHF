#!/usr/bin/env python3
import serial
import time
import struct
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("carel_evd_communication.log"),
        logging.StreamHandler()
    ]
)

class CarelEVDCommunicator:
    """
    A class to handle communication with Carel EVD devices via RS485/Modbus RTU
    """
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, slave_address=1):
        """
        Initialize the Carel EVD communicator
        
        Args:
            port (str): Serial port to connect to
            baudrate (int): Communication speed in bits per second
            slave_address (int): Modbus slave address of the EVD device
        """
        self.port = port
        self.baudrate = baudrate
        self.slave_address = slave_address
        self.serial = None
        self.connected = False
        
    def connect(self):
        """Establish connection to the EVD device"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.connected = True
            logging.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            logging.error(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close the connection to the EVD device"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            logging.info("Disconnected from EVD device")
            
    def calculate_crc16(self, data):
        """
        Calculate CRC-16 (Modbus) for the given data
        
        Args:
            data (bytes): Data to calculate CRC for
            
        Returns:
            bytes: CRC as 2 bytes in little-endian order
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)  # Little-endian format
    
    def read_holding_registers(self, start_address, register_count):
        """
        Read multiple holding registers from the EVD device
        
        Args:
            start_address (int): Starting register address
            register_count (int): Number of registers to read
            
        Returns:
            list: Register values or None if failed
        """
        if not self.connected:
            logging.error("Not connected to device")
            return None
            
        # Build Modbus RTU read holding registers (function code 0x03) message
        message = struct.pack(
            '>BBHH',
            self.slave_address,  # Slave address
            0x03,                # Function code (Read Holding Registers)
            start_address,       # Starting address
            register_count       # Quantity of registers
        )
        
        # Add CRC
        message += self.calculate_crc16(message)
        
        try:
            # Clear any pending data
            self.serial.reset_input_buffer()
            
            # Send message
            self.serial.write(message)
            logging.debug(f"Sent: {message.hex()}")
            
            # Wait for response - minimum response is 5 bytes
            # (slave address, function code, byte count, at least 2 bytes of CRC)
            time.sleep(0.1)
            
            # Calculate expected response length: 
            # 1 byte slave address + 1 byte function code + 1 byte count + 2 bytes per register + 2 bytes CRC
            expected_length = 5 + (register_count * 2)
            
            # Read response
            response = self.serial.read(expected_length)
            logging.debug(f"Received: {response.hex()}")
            
            if len(response) != expected_length:
                logging.error(f"Incomplete response. Expected {expected_length} bytes, got {len(response)}")
                return None
                
            # Verify response
            if response[0] != self.slave_address:
                logging.error(f"Wrong slave address in response: {response[0]}")
                return None
                
            if response[1] != 0x03:
                # Check if it's an exception response
                if response[1] == 0x83:
                    logging.error(f"Modbus exception received: {response[2]}")
                else:
                    logging.error(f"Unexpected function code in response: {response[1]}")
                return None
                
            # Extract data
            byte_count = response[2]
            if byte_count != register_count * 2:
                logging.error(f"Unexpected byte count: {byte_count}")
                return None
                
            # Extract register values
            registers = []
            for i in range(register_count):
                reg_value = struct.unpack('>H', response[3 + i*2:5 + i*2])[0]
                registers.append(reg_value)
                
            return registers
            
        except Exception as e:
            logging.error(f"Communication error: {e}")
            return None
            
    def get_device_status(self):
        """
        Get the status information from the EVD device
        
        Returns:
            dict: Status information or None if failed
        """
        # Define register addresses for common EVD status parameters
        # (These addresses may vary based on your specific EVD model and firmware)
        registers = {
            "valve_opening_percentage": 0x0016,
            "superheat": 0x0009,
            "suction_temperature": 0x0002,
            "evaporation_temperature": 0x0003,
            "evaporation_pressure": 0x0004,
            "valve_control_status": 0x0016,
            "alarm_status": 0x0006
        }
        
        try:
            # Read all registers at once (more efficient)
            values = self.read_holding_registers(0x0000, 7)
            
            if values is None:
                return None
                
            # Process raw values based on EVD specifications
            # Note: You may need to adjust these calculations based on your specific model
            status = {
                "timestamp": datetime.now().isoformat(),
                "valve_opening_percentage": values[0] / 10.0,  # Assuming scale factor of 0.1
                "superheat": values[1] / 10.0,  # Assuming scale factor of 0.1 K
                "suction_temperature": values[2] / 10.0 - 273.15,  # Convert to Celsius
                "evaporation_temperature": values[3] / 10.0 - 273.15,  # Convert to Celsius
                "evaporation_pressure": values[4] / 100.0,  # Assuming scale factor of 0.01 bar
                "valve_control_status": self._decode_valve_status(values[5]),
                "alarm_status": self._decode_alarms(values[6])
            }
            
            return status
            
        except Exception as e:
            logging.error(f"Error getting device status: {e}")
            return None
    
    def _decode_valve_status(self, status_code):
        """
        Decode the valve control status code
        
        Args:
            status_code (int): Status code from the device
            
        Returns:
            str: Human readable status description
        """
        # This is an example and should be adjusted based on actual EVD documentation
        status_map = {
            0: "Closed",
            1: "Opening",
            2: "Open",
            3: "Closing",
            4: "Positioning",
            5: "Error",
            6: "Manual control"
        }
        return status_map.get(status_code, f"Unknown status ({status_code})")
    
    def _decode_alarms(self, alarm_code):
        """
        Decode the alarm status bits
        
        Args:
            alarm_code (int): Alarm code from the device
            
        Returns:
            list: List of active alarms
        """
        # This is an example and should be adjusted based on actual EVD documentation
        alarm_bits = {
            0: "Motor error",
            1: "Probe error",
            2: "Superheat control error",
            3: "Valve not closing properly",
            4: "Low SH",
            5: "Low suction temperature",
            6: "Low evaporation pressure",
            7: "High evaporation pressure",
            8: "Emergency closing",
            9: "Communication error",
            10: "EEPROM error",
            11: "Battery error"
        }
        
        active_alarms = []
        for bit, description in alarm_bits.items():
            if alarm_code & (1 << bit):
                active_alarms.append(description)
                
        return active_alarms if active_alarms else ["No alarms"]


def main():
    """Main function to run the EVD communication program"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Carel EVD RS485 Communication Tool")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port to use")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baudrate for communication")
    parser.add_argument("--address", type=int, default=1, help="Modbus slave address")
    parser.add_argument("--interval", type=float, default=5.0, help="Polling interval in seconds")
    parser.add_argument("--continuous", action="store_true", help="Run in continuous polling mode")
    
    args = parser.parse_args()
    
    # Create the communicator
    evd = CarelEVDCommunicator(port=args.port, baudrate=args.baudrate, slave_address=args.address)
    
    try:
        # Connect to the device
        if not evd.connect():
            logging.error("Failed to connect. Exiting.")
            return 1
        
        # Poll for status
        if args.continuous:
            logging.info(f"Starting continuous polling with {args.interval}s interval. Press Ctrl+C to stop.")
            while True:
                status = evd.get_device_status()
                if status:
                    print("\n--- EVD Status at", status["timestamp"], "---")
                    for key, value in status.items():
                        if key != "timestamp":
                            print(f"{key.replace('_', ' ').title()}: {value}")
                time.sleep(args.interval)
        else:
            # Single status read
            status = evd.get_device_status()
            if status:
                print("\n--- EVD Status ---")
                for key, value in status.items():
                    print(f"{key.replace('_', ' ').title()}: {value}")
                return 0
            else:
                logging.error("Failed to read device status")
                return 1
                
    except KeyboardInterrupt:
        logging.info("Program stopped by user")
    finally:
        # Always disconnect properly
        evd.disconnect()
    
    return 0

if __name__ == "__main__":
    exit(main())
