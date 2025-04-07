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
            
            # Wait for response
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
            
    def read_single_register(self, register_address):
        """
        Read a single register from the EVD device
        
        Args:
            register_address (int): Register address to read
            
        Returns:
            int: Register value or None if failed
        """
        result = self.read_holding_registers(register_address, 1)
        if result and len(result) == 1:
            return result[0]
        return None
    
    def get_device_status(self):
        """
        Get the status information from the EVD device
        
        Returns:
            dict: Status information or None if failed
        """
        # Define register addresses for EVD status parameters with actual addresses
        registers = {
            "valve_opening_percentage": 0x10,  # Register 16
            "superheat": 0x09,                 # Register 9
            "evaporation_pressure": 0x06,      # Register 6
            "temp_probe": 0x01,      # Register 1
            # Add other registers as needed
        }
        
        try:
            status = {
                "timestamp": datetime.now().isoformat()
            }
            
            # Read each register individually
            for param, reg_addr in registers.items():
                logging.info(f"Reading {param} from register 0x{reg_addr:04X}")
                raw_value = self.read_single_register(reg_addr)
                
                if raw_value is None:
                    logging.error(f"Failed to read {param} from register 0x{reg_addr:04X}")
                    continue
                
                # Process each parameter according to its specific scaling/conversion
                if param == "valve_opening_percentage":
                    status[param] = raw_value / 10.0  # Assuming scale factor of 0.1
                elif param == "superheat":
                    status[param] = raw_value / 10.0  # Assuming scale factor of 0.1 K
                elif param == "evaporation_pressure":
                    status[param] = raw_value / 100.0  # Assuming scale factor of 0.01 bar
                elif param == "temp_probe":
                    status[param] = raw_value / 10.0  # Assuming scale factor of 0.01 C
                else:
                    # Default handling for other parameters
                    status[param] = raw_value
            
            return status
            
        except Exception as e:
            logging.error(f"Error getting device status: {e}")
            return None
    
    def get_device_status_optimized(self):
        """
        Get the status information from the EVD device using grouped register reads
        where possible to optimize communication
        
        Returns:
            dict: Status information or None if failed
        """
        # Define register addresses
        registers = {
            "valve_opening_percentage": 0x10,  # Register 16
            "superheat": 0x09,                 # Register 9
            "evaporation_pressure": 0x06,      # Register 6
            "temp_probe": 0x01,      # Register 1
            # Add other registers as needed
        }
        
        try:
            status = {
                "timestamp": datetime.now().isoformat()
            }
            
            # Group consecutive registers for more efficient reading
            reg_groups = self._group_consecutive_registers(registers)
            
            # Read each group of registers
            for start_reg, count, reg_map in reg_groups:
                values = self.read_holding_registers(start_reg, count)
                if values is None:
                    logging.error(f"Failed to read register group starting at 0x{start_reg:04X}")
                    continue
                
                # Process each parameter in this group
                for param, (reg_addr, idx) in reg_map.items():
                    raw_value = values[idx]
                    
                    # Process each parameter according to its specific scaling/conversion
                    if param == "valve_opening_percentage":
                        status[param] = raw_value / 10.0  # Assuming scale factor of 0.1
                    elif param == "superheat":
                        status[param] = raw_value / 10.0  # Assuming scale factor of 0.1 K
                    elif param == "evaporation_pressure":
                        status[param] = raw_value / 100.0  # Assuming scale factor of 0.01 bar
                    elif param == "temp_probe":
                        status[param] = raw_value / 10.0  # Assuming scale factor of 0.01 C
                    else:
                        # Default handling for other parameters
                        status[param] = raw_value
            
            return status
            
        except Exception as e:
            logging.error(f"Error getting device status: {e}")
            return None
    
    def _group_consecutive_registers(self, registers, max_gap=5):
        """
        Group consecutive registers to minimize the number of reads
        
        Args:
            registers (dict): Dictionary mapping parameter names to register addresses
            max_gap (int): Maximum gap between registers to consider them part of the same group
            
        Returns:
            list: List of tuples (start_address, count, register_mapping)
                  where register_mapping maps parameter names to (reg_addr, index) pairs
        """
        # Sort registers by address
        sorted_regs = sorted([(param, addr) for param, addr in registers.items()], 
                             key=lambda x: x[1])
        
        if not sorted_regs:
            return []
            
        groups = []
        current_group = []
        
        for param, addr in sorted_regs:
            if not current_group or addr <= current_group[-1][1] + max_gap:
                current_group.append((param, addr))
            else:
                groups.append(current_group)
                current_group = [(param, addr)]
                
        if current_group:
            groups.append(current_group)
            
        # Convert groups to (start_address, count, mapping) format
        result = []
        for group in groups:
            start_addr = group[0][1]
            end_addr = group[-1][1]
            count = end_addr - start_addr + 1
            
            # Create mapping from parameter name to (reg_addr, index in response)
            mapping = {param: (addr, addr - start_addr) for param, addr in group}
            
            result.append((start_addr, count, mapping))
            
        return result


def main():
    """Main function to run the EVD communication program"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Carel EVD RS485 Communication Tool")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port to use")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baudrate for communication")
    parser.add_argument("--address", type=int, default=1, help="Modbus slave address")
    parser.add_argument("--interval", type=float, default=5.0, help="Polling interval in seconds")
    parser.add_argument("--continuous", action="store_true", help="Run in continuous polling mode")
    parser.add_argument("--optimize", action="store_true", help="Use optimized reading method")
    
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
                if args.optimize:
                    status = evd.get_device_status_optimized()
                else:
                    status = evd.get_device_status()
                    
                if status:
                    print("\n--- EVD Status at", status["timestamp"], "---")
                    for key, value in status.items():
                        if key != "timestamp":
                            print(f"{key.replace('_', ' ').title()}: {value}")
                time.sleep(args.interval)
        else:
            # Single status read
            if args.optimize:
                status = evd.get_device_status_optimized()
            else:
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
