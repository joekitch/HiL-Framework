#!/usr/bin/env python3
"""
CAN and Hardware Interface Utilities for HiL Testing Framework

This module provides utilities for CAN bus communication with Arduino (MCP2515), 
Arduino serial communication,
DAC control (MCP4725) for analog output,
ADC reading (ADS1115) for analog input,
Toyota Corolla 2017 CAN message handling for OpenPilot
"""

import time
import os
import glob
from typing import Optional, List, Tuple, Dict, Any
import cantools.database
import serial
import can
import cantools
import board
import busio
import adafruit_mcp4725
import adafruit_ads1x15.ads1115 as ADC
from adafruit_ads1x15.analog_in import AnalogIn

class HiLInterface:
    """
    A general class for hardware interfacing across the entire HiL
    Contains various methods for communicating with the device under test, and the DAC/ADC
    """
    def __init__(self,
                dbc_path: str = '../opendbc/toyota_corolla_2017_pt_generated.dbc',
                can_channel: str = 'can0',
                can_bitrate: int = 500000,
                serial_baudrate: int = 115200):
        """
        Initialize the Hardware-in-Loop interface
        
        Args:
            dbc_path: Path to the DBC file
            can_channel: CAN interface name
            can_bitrate: CAN bitrate in bits/second
            serial_baudrate: Serial baudrate for Arduino communication
        """
        self.dbc_path = dbc_path
        self.can_channel = can_channel
        self.can_bitrate = can_bitrate
        self.serial_baudrate = serial_baudrate
        
        # Initialize I2C for DAC and ADC
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialize attributes that will be set up later
        self.can_bus = None
        self.db = None
        self.serial_port = None
        self.dac = None
        self.adc = None
        
        self._load_dbc()
        
    def _load_dbc(self) -> None:
        """Load the DBC file for CAN message parsing/creation"""
        try:
            self.db = cantools.database.load_file(self.dbc_path)
            print(f"Successfully loaded DBC file: {self.dbc_path}")
        except FileNotFoundError:
            print(f"DBC file not found: {self.dbc_path}")
        except PermissionError:
            print(f"Permission denied when accessing DBC file: {self.dbc_path}")
        except (ValueError, SyntaxError) as e:
            print(f"Invalid DBC file format: {e}")
        except Exception as e:
            # fallback
            print(f"Unexpected error loading DBC file: {e}")
            
    def setup_can(self) -> bool:
        """
        Set up the CAN interface
        
        Returns:
            bool: True if setup was successful, False otherwise
        """
        try:
            # Configure the CAN interface if not already done
            os.system(f"sudo ip link set {self.can_channel} type can bitrate {self.can_bitrate}")
            os.system(f"sudo ifconfig {self.can_channel} up")
            
            # Create a CAN bus interface
            self.can_bus = can.interface.Bus(channel=self.can_channel, bustype='socketcan')
            print(f"Successfully set up CAN interface on {self.can_channel}")
            return True
        except OSError as e:
            print(f"OS error when setting up CAN interface: {e}")
            return False
        except can.CanError as e:  # python-can library specific exception
            print(f"CAN bus error: {e}")
            return False
        except Exception as e:
            # fallback
            print(f"Unexpected error setting up CAN interface: {e}")
            return False
            
    def setup_dac(self, address: int = 0x60) -> bool:
        """
        Set up the MCP4725 DAC
        
        Args:
            address: I2C address of the DAC
            
        Returns:
            bool: True if setup was successful, False otherwise
        """
        try:
            self.dac = adafruit_mcp4725.MCP4725(
                self.i2c,
                address=address
            )
            print(f"Successfully set up MCP4725 DAC at address 0x{address:02X}")
            return True
        except ValueError as e:
            print(f"Invalid parameter for DAC setup: {e}")
            return False
        except (OSError, IOError) as e:
            print(f"I2C communication error with DAC: {e}")
            return False
        except RuntimeError as e:
            print(f"Runtime error with DAC hardware: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error setting up DAC: {e}")
            return False
            
    def setup_adc(self, address: int = 0x48) -> bool:
        """
        Set up the ADS1115 ADC
        
        Args:
            address: I2C address of the ADC
            
        Returns:
            bool: True if setup was successful, False otherwise
        """
        try:
            self.adc = ADC.ADS1115(self.i2c, address=address)
            print(f"Successfully set up ADS1115 ADC at address 0x{address:02X}")
            return True
        except ValueError as e:
            print(f"Invalid parameter for DAC setup: {e}")
            return False
        except (OSError, IOError) as e:
            print(f"I2C communication error with DAC: {e}")
            return False
        except RuntimeError as e:
            print(f"Runtime error with DAC hardware: {e}")
            return False
        except Exception as e:
            print(f"Error setting up ADC: {e}")
            return False
    
    def find_arduino_serial_port(self) -> Optional[str]:
        """
        Find the Arduino serial port by checking available ports
        
        Returns:
            str: Serial port name if found, None otherwise
        """
        # Common patterns for Arduino serial ports
        patterns = [
            '/dev/ttyACM*',  # Linux Arduino Uno/Mega
            '/dev/ttyUSB*',  # Linux with USB-to-Serial converter
            '/dev/tty.usbmodem*',  # macOS Arduino Uno/Mega
            '/dev/tty.usbserial*',  # macOS with USB-to-Serial converter
            'COM*'  # Windows
        ]
        
        ports = []
        for pattern in patterns:
            ports.extend(glob.glob(pattern))
            
        if not ports:
            print("No Arduino serial ports found")
            return None
            
        # If multiple ports are found, we'll need to check each one
        for port in ports:
            try:
                # Try to open the port and send a test command
                with serial.Serial(port, self.serial_baudrate, timeout=1) as ser:
                    # Wait for Arduino bootloader
                    time.sleep(2)
                    # Send a simple command and check for response
                    ser.write(b'PING\n')
                    response = ser.readline().strip()
                    if response:
                        print(f"Found Arduino on port {port}")
                        return port
            except Exception as e:
                print(f"Failed to communicate with port {port}: {e}")
                
        print("No responding Arduino found on any port")
        return None
        
    def setup_serial(self, port: Optional[str] = None) -> bool:
        """
        Set up serial communication with the Arduino
        
        Args:
            port: Serial port to use. If None, will attempt to find it.
            
        Returns:
            bool: True if setup was successful, False otherwise
        """
        if port is None:
            port = self.find_arduino_serial_port()
            if port is None:
                return False
        
        try:
            self.serial_port = serial.Serial(port, self.serial_baudrate, timeout=1)
            print(f"Successfully connected to Arduino on {port}")
            return True
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            return False
        except ValueError as e:
            print(f"Invalid serial parameter: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error setting up serial: {e}")
            return False
            
    def is_arduino_alive(self) -> bool:
        """
        Check if the Arduino is responsive
        
        Returns:
            bool: True if Arduino is responsive, False otherwise
        """
        # First try serial communication if available
        if self.serial_port:
            try:
                self.serial_port.write(b'PING\n')
                response = self.serial_port.readline().strip()
                if response:
                    print("Arduino is responsive via serial")
                    return True
            except Exception as e:
                print(f"Serial communication error: {e}")
                
        # Then try CAN communication if available
        if self.can_bus and self.db:
            try:
                # Send a simple CAN message and check for a response
                # Using the GAS_SENSOR message ID to check for a response
                message = can.Message(
                    arbitration_id=513,  # GAS_SENSOR ID
                    data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )
                self.can_bus.send(message)
                
                # Wait for a response for up to 1 second
                response = self.can_bus.recv(timeout=1.0)
                if response:
                    print("Arduino is responsive via CAN")
                    return True
            #Cantools doesn't have a more specific exception for this
            except Exception as e:
                print(f"CAN communication error: {e}")
                
        print("Arduino does not appear to be responsive")
        return False
        
    def set_voltage(self, voltage: float) -> bool:
        """
        Set the DAC output voltage
        
        Args:
            voltage: Desired voltage (0-5V)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.dac:
            print("DAC not set up")
            return False
            
        if not 0 <= voltage <= 5:
            print(f"Voltage must be between 0 and 5V, got {voltage}V")
            return False
            
        try:
            # Convert voltage to a 12-bit value (0-4095)
            # MCP4725 is 12-bit, so 4095 = 5V
            value = int((voltage / 5.0) * 4095)
            self.dac.raw_value = value
            print(f"Set DAC to {voltage}V (raw value: {value})")
            return True
        except Exception as e:
            print(f"Error setting DAC voltage: {e}")
            return False
            
    def read_voltage(self, adc_channel: int = 0) -> Optional[float]:
        """
        Read voltage from the ADC
        
        Args:
            adc_channel: ADC channel to read from (0-3)
            
        Returns:
            float: Voltage (0-5V) if successful, None otherwise
        """
        if not self.adc:
            print("ADC not set up")
            return None
            
        if not 0 <= adc_channel <= 3:
            print(f"ADC channel must be between 0 and 3, got {adc_channel}")
            return None
            
        try:
            # Set up the channel to read from
            channels = [
                AnalogIn(self.adc, ADC.P0),
                AnalogIn(self.adc, ADC.P1),
                AnalogIn(self.adc, ADC.P2),
                AnalogIn(self.adc, ADC.P3)
            ]
            chan = channels[adc_channel]
            
            # Read the voltage
            voltage = chan.voltage
            print(f"Read {voltage}V from ADC channel {adc_channel}")
            return voltage
        except Exception as e:
            print(f"Error reading ADC voltage: {e}")
            return None
            
    def send_throttle_command(self, throttle_percent: float) -> bool:
        """
        Send a throttle command via CAN
        
        Args:
            throttle_percent: Throttle percentage (0-100%)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.can_bus or not self.db:
            print("CAN bus or DBC database not set up")
            return False
            
        if not 0 <= throttle_percent <= 100:
            print(f"Throttle percentage must be between 0 and 100, got {throttle_percent}")
            return False
            
        try:
            # Scale throttle_percent to the range expected by the CAN message
            # GAS_COMMAND signal uses a scale factor of 0.159375 with offset -75.555
            # So to calculate the raw value: raw = (desired_value + 75.555) / 0.159375
            gas_command_raw = int((throttle_percent + 75.555) / 0.159375)
            
            # Create the CAN message data using the DBC file
            message_data = self.db.encode_message('GAS_COMMAND', {
                'GAS_COMMAND': gas_command_raw,
                'ENABLE': 1,  # Enable throttle control
                'COUNTER_PEDAL': 0,  # Counter should increment, but starting at 0
                'CHECKSUM_PEDAL': 0  # Checksum will be calculated by vehicle
            })
            
            # Send the CAN message
            message = can.Message(
                arbitration_id=512,  # GAS_COMMAND ID
                data=message_data,
                is_extended_id=False
            )
            self.can_bus.send(message)
            print(f"Sent throttle command: {throttle_percent}%")
            return True
        except cantools.database.EncodeError as e:
            print(f"Encoder error when sending throttle command: {e}")
            return False
        except Exception as e:
            print(f"Error sending throttle command: {e}")
            return False
            
    def close(self) -> None:
        """Close all open connections and interfaces"""
        # Close serial port if open
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Closed serial port")
            
        # Close CAN bus if open
        if self.can_bus:
            self.can_bus.shutdown()
            print("Shut down CAN bus")
            
        # Turn off CAN interface
        os.system(f"sudo ifconfig {self.can_channel} down")
        print(f"Turned off CAN interface {self.can_channel}")


# Example usage
if __name__ == "__main__":
    # Create the interface
    hil = HiLInterface()
    
    # Set up all components
    can_ok = hil.setup_can()
    dac_ok = hil.setup_dac()
    adc_ok = hil.setup_adc()
    serial_ok = hil.setup_serial()
    
    if can_ok and dac_ok and adc_ok and serial_ok:
        print("All systems initialized successfully")
        
        # Check if Arduino is responsive
        if hil.is_arduino_alive():
            print("Arduino is alive and responding")
            
            # Example: Set voltage and read it back
            hil.set_voltage(2.5)  # Set to 2.5V
            time.sleep(0.5)  # Wait for voltage to stabilize
            voltage = hil.read_voltage(0)  # Read from channel 0
            print(f"Voltage read back: {voltage}V")
            
            # Example: Send throttle command
            hil.send_throttle_command(50)  # Set to 50%
    
    # Clean up
    hil.close()