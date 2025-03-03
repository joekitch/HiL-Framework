#!/usr/bin/env python3
"""
Hardware-in-the-Loop (HiL) test suite for Arduino throttle interface.

Runs various tests on the DUT (arduino throttle interceptor unit)
"""

import time
from pathlib import Path
import sys
import pytest
from utils.setup_raspi_arduino import HiLInterface  # Assuming the filename is can_interface.py


# Add the parent directory to sys.path to import the utils module
sys.path.append(str(Path(__file__).parent.parent))

# Disable redefined-outer-name warning (pytest fixtures)
# pylint: disable=redefined-outer-name

# Constants
VOLTAGE_TOLERANCE = 0.1  # 100mV tolerance
SETUP_TIMEOUT = 5  # seconds timeout
VOLTAGE_STABILIZE_TIME = 0.5  # seconds to wait for voltage to stabilize

@pytest.fixture(scope="session")
def hil_setup():
    """
    Fixture that sets up the Hardware-in-Loop test environment.
    This fixture runs once for the entire test session.
    
    If any of these steps fail, the entire test suite will be skipped. (smoke test)
    """
    print("\nSetting up HiL...")
    
    # Initialize the HiL interface
    hil = HiLInterface()

    # Check if Arduino is responsive
    arduino_alive = hil.is_arduino_alive()
    if not arduino_alive:
        pytest.skip("Arduino is not responsive")

    # serial communication
    serial_ok = hil.setup_serial()
    if not serial_ok:
        pytest.skip("Failed to set up serial communication with Arduino")
    
    # canbus
    can_ok = hil.setup_can()
    if not can_ok:
        pytest.skip("Failed to set up CAN bus interface")
    
    # DAC and ADC
    dac_ok = hil.setup_dac()
    adc_ok = hil.setup_adc()
    if not (dac_ok and adc_ok):
        pytest.skip("Failed to set up DAC or ADC interfaces")

    print("HiL test environment successfully set up")
    
    # yield interface to tests
    yield hil
    
    # teardown
    print("\nTearing down HiL...")
    hil.close()
    print("HiL cleaned up")

def test_dac_adc_reference(hil_setup):
    """
    Test that the DAC can set a reference voltage and the ADC can read a response from DUT.
    
    """
    # Reference voltage to set and expect
    ref_voltage = 2.5
    
    print(f"\nSetting DAC output to {ref_voltage}V...")
    set_success = hil_setup.set_voltage(ref_voltage)
    assert set_success, "Failed to set DAC voltage"
    
    print(f"Waiting {VOLTAGE_STABILIZE_TIME}s for voltage to stabilize...")
    time.sleep(VOLTAGE_STABILIZE_TIME)
    
    print("Reading voltage from ADC...")
    measured_voltage = hil_setup.read_voltage(0)  # Read from channel 0
    assert measured_voltage is not None, "Failed to read ADC voltage"
    
    print(f"Expected: {ref_voltage}V, Measured: {measured_voltage}V")
    # Verify the measured voltage is within tolerance
    assert abs(measured_voltage - ref_voltage) < VOLTAGE_TOLERANCE, \
        f"Measured voltage ({measured_voltage}V) is outside tolerance range " \
        f"({ref_voltage - VOLTAGE_TOLERANCE}V to {ref_voltage + VOLTAGE_TOLERANCE}V)"

def test_throttle_can_command_half(hil_setup):
    """
    Test that a CAN throttle command affects the Arduino output voltage.

    Args:
        hil_setup: The initialized HiLInterface fixture
    """
    # Throttle percentage and expected voltage
    throttle_percent = 50.0
    expected_voltage = 2.5
    
    print(f"\nSending CAN throttle command for {throttle_percent}% throttle...")
    send_success = hil_setup.send_throttle_command(throttle_percent)
    assert send_success, "Failed to send CAN throttle command"
    
    print(f"Waiting {VOLTAGE_STABILIZE_TIME}s for Arduino to process command...")
    time.sleep(VOLTAGE_STABILIZE_TIME)
    
    print("Reading voltage from ADC to verify Arduino response...")
    measured_voltage = hil_setup.read_voltage(0)  # Read from channel 0
    assert measured_voltage is not None, "Failed to read ADC voltage"
    
    print(f"Expected: {expected_voltage}V, Measured: {measured_voltage}V")
    assert abs(measured_voltage - expected_voltage) < VOLTAGE_TOLERANCE, \
        f"Measured voltage ({measured_voltage}V) doesn't match expected voltage " \
        f"for {throttle_percent}% throttle ({expected_voltage}V)"

if __name__ == "__main__":
    print("invoke with 'pytest tests/'")