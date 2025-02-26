"""
zipline take home assignment

reating test cases for an ADXL345 accelerometer chip


ASSUMPTIONS MADE:
- That the i2c_cmd uses the device address each time, and the register to use is part of the data 
    variable so to set the bw rate register's D3-D0 bits, data would be [0x2C, 0x0F]
- That the adxl is already calibrated and the three offset value registers are already set correctly , 
    for instance if X was off by 27 I'd write -7 to register 0x1E
- That the frequency setting already converts to Rate Code (400hz is 1100 rate code in the doc)
- That there's not a feasible way to measure elapsed time without importing the time library
- The i2c address is 0x53, which is the default for the adxl45
- Interpreting the "maximum data rate possible" line to mean maximum SPI rate of 5mhz during i2c setup

POSSIBLE IMPROVEMENTS:
- with pytest, the configuration step could be a test fixture, 
    set to run at the module level, setting the board up only once per test run
- I'm using a wrapper to process execution times but it gets clunky, 
    with the custom error message handling and needing the output print statement to be very specific
- Adding register checks after each i2c command, to make sure the target register 
    was actually set correctly, using something like i2c_cmd(0x53,0x2C,resp_len=1)
"""

from time import perf_counter
from time import time
from functools import wraps
from zip_test_fwk import ZipTestBoard as Zip

# store all registers in a dictionary for readability
ADDRESSES = {

    "DATA_FORMAT":0x31,
    "BW_RATE":0x2C,
    "DATAX0":0x32,  
    "DATAX1":0x33,   
    "DATAY0":0x34,   
    "DATAY1":0x35,   
    "DATAZ0":0x36,  
    "DATAZ1":0x37,  

}
# store device address
ADXL345 = 0x53


class TestFailure(Exception):
    """Exception raised for custom error scenarios.

    Attributes:
        message -- explanation of the error
    """
    def __init__(self, message) -> None:
        super().__init__(message)
        self.message = message
    def __str__(self):
        return f'{self.message}'

def handle_failure(error):
    """
    Custom exception handling
    Args:
        error, the exception being thrown
    """
    print(f" due to {error}")
    Zip.turn_off_ps("2V5")

def convert_to_gs(data, scale):
    """
    convert raw accelerometer data from bytes into gs, 
    combine LSB and MSB into a 16 bit signed integer,
    then adjust by scale factor
    """
    data = (data[1] << 8) | data[0]
    data = data / scale
    return data

def capture_accelerometer_data(duration: float,
                               scalex: int,
                               scaley: int,
                               scalez: int,
                               single: bool) -> dict:
    """
    Gather data from the acceleromter for {time} seconds
    then convert into g forces, and return a dict
    Args:
        time (float): number of seconds to gather data
        scale (int): the scales for each axis to convert into g forces
        single (bool): flag to gather a single data point instead of over time
    Returns:
        data: dictionary of x/y/z data in gs
    """
    data={"DATAX":0,
          "DATAY":0,
          "DATAZ":0,}

    if single:
        try:
            data["DATAX"] = convert_to_gs([Zip.i2c_cmd(ADXL345, ADDRESSES["DATAX0"], resp_len=2),
                                           Zip.i2c_cmd(ADXL345, ADDRESSES["DATAX1"], resp_len=2)],
                                           scalex)
            data["DATAY"] = convert_to_gs([Zip.i2c_cmd(ADXL345, ADDRESSES["DATAY0"], resp_len=2),
                                           Zip.i2c_cmd(ADXL345, ADDRESSES["DATAY1"], resp_len=2)],
                                           scaley)
            data["DATAZ"] = convert_to_gs([Zip.i2c_cmd(ADXL345, ADDRESSES["DATAZ0"], resp_len=2),
                                           Zip.i2c_cmd(ADXL345, ADDRESSES["DATAZ1"], resp_len=2)],
                                           scalez)
        except I2CError as e:
            print(f"TEST FAILED ")
            handle_failure(e)
    else:
        timer_start = time()
        duration = 0.1
        x,y,z = [],[],[]
        while time() < timer_start + duration:
            try:
                x.append(convert_to_gs([Zip.i2c_cmd(ADXL345, ADDRESSES["DATAX0"], resp_len=2),
                                        Zip.i2c_cmd(ADXL345, ADDRESSES["DATAX1"], resp_len=2)],
                                        scalex))
                y.append(convert_to_gs([Zip.i2c_cmd(ADXL345, ADDRESSES["DATAY0"], resp_len=2),
                                        Zip.i2c_cmd(ADXL345, ADDRESSES["DATAY1"], resp_len=2)],
                                        scaley))
                z.append(convert_to_gs([Zip.i2c_cmd(ADXL345, ADDRESSES["DATAZ0"], resp_len=2),
                                        Zip.i2c_cmd(ADXL345, ADDRESSES["DATAZ1"], resp_len=2)],
                                        scalez))
            except I2CError as e:
                print(f"TEST FAILED ")
                handle_failure(e)
        data["DATAX"] = sum(x) // len(x)
        data["DATAY"] = sum(y) // len(y)
        data["DATAZ"] = sum(z) // len(z)

    return data



#using a decorator to measure elapsed time, works equally if it passes or fails
def elapsed(func):
    """
    Decorator to record duration of test cases
    """
    @wraps(func)
    def elapsed_wrapper(*args, **kwargs):
        start = perf_counter()
        result = func(*args, **kwargs)
        end = perf_counter()
        total = end-start
        print(f'in {total:.4f} sec')
        return result
    yield elapsed_wrapper

@elapsed
def configure():
    """
    Configures the accelerometer to output valid x,y,z measurements 
    at the maximum data rate possible given communications and sensor constraints 
    """
    try:
        Zip.__init__()
    except ConnectionError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    # 2.5v is specified as normal operational voltage
    Zip.turn_on_ps("2V5")

    # 5mhz is the maximum specified i2c data rate
    Zip.i2c_setup(MCU_DIO2,MCU_DIO1,5000000)

    # normal power operation, set D4 to 0 using 00001010 
    try:
        Zip.i2c_cmd(ADXL345, [ADDRESSES["BW_RATE"], 0x0A], resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    # set data rate to 3200 with rate code 1111, setting D3-D0 to high
    try:
        Zip.i2c_cmd(ADXL345, [ADDRESSES["BW_RATE"], 0x0F], resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    # initial sensitivity settings, use DATA_FORMAT register of 0x31, 
    # set D3 bit and write 0x03 to range bits D1 and D0, 3.9mg/LSB scale factor
    try:
        Zip.i2c_cmd(ADXL345, [ADDRESSES["DATA_FORMAT"], 0x0F], resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)




@elapsed
def test_selftest():
    """
    Run accelerometer self test and check results are within datasheet range
    data rate and power settings are aleady set to 3200 and normal power by configure()
    """

    # set to 16g mode, set D3-D0 to 1 in data_format
    try:
        Zip.i2c_cmd(ADXL345, [ADDRESSES["DATA_FORMAT"], 0x07], resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    # gather data with selftest off
    # for +-16gs, the scale factor is 32 LSB/g
    scale = 32
    st_off = capture_accelerometer_data(0.1, scale,scale,scale,False)
    #set d7 in data_format register to start self test
    try:
        Zip.i2c_cmd(ADXL345, [ADDRESSES["DATA_FORMAT"], 0x80], resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    #wait for 4 samples before actually logging data
    for _ in range (4):
        capture_accelerometer_data(0,scale,scale,scale,True)
                                                                  
    # gather data with selftest on
    scale = 32
    st_on = capture_accelerometer_data(0.1, scale,scale,scale,False)

    #calculate self test values
    x_st = st_on.get("DATAX") - st_off.get("DATAX")
    y_st = st_on.get("DATAY") - st_off.get("DATAY")
    z_st = st_on.get("DATAZ") - st_off.get("DATAZ")

    if ((6 <= x_st <= 67) and (-67 <= y_st <= -6) and (19 <= z_st <= 219)):
        return f"TEST PASSED "
    else:
        raise TestFailure("TEST FAILED ")

@elapsed
def test_slow_climb():
    """
    Move actuator through “slow_climb” config 
    and check throughout the motion that the y axis is between -1g and 1g, 
    and that the z axis is between 6g and 8g 
    """

    #set to +-8g resolution, bits D1 and D0 to 10 in data format register
    try:
        Zip.i2c_cmd(ADXL345, [ADDRESSES["DATA_FORMAT"], 0x02], resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    # for +-1gs, the scale factor is 256 LSB/g
    scalexy = 256
    # for +-8gs, the scale factor is 64 LSB/g
    scalez = 64
    try:
        Zip.actuator_move('slow_climb')
    except ActuatorError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    data = capture_accelerometer_data(10.0,scalexy,scalexy,scalez,False)

    if ((-1 <= data.get("DATAX") <= 1) 
        and (-1 <= data.get("DATAY") <= 1) 
        and (6 <= data.get("DATAZ") <= 8)):
        passed = True
    else:
        raise TestFailure("TEST FAILED ")

    return f"TEST PASSED " if passed else ("TEST FAILED ")

@elapsed
def test_sharp_turn():
    """
    Move actuator through “sharp_turn” config 
    and check throughout the motion that the x axis and y axis are both greater than 5g 
    """

    #set to +-8g resolution, bits D1 and D0 to 10 in data format register
    try:
        Zip.i2c_cmd(ADXL345, [ADDRESSES["DATA_FORMAT"], 0x02], resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    # for +-8gs, the scale factor is 64 LSB/g
    scale = 64
    try:
        Zip.actuator_move('sharp_turn')
    except ActuatorError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    data = capture_accelerometer_data(10.0,scale,scale,scale,False)

    if ((5 < data.get("DATAX")) and (5 < data.get("dataY"))):
        passed = True
    else:
        raise TestFailure("TEST FAILED ")

    return f"TEST PASSED " if passed else ("TEST FAILED ")


@elapsed
def test_quick_drop():
    """
    Move actuator through “quick_drop” config and check throughout the motion 
    that the z axis is less than -8g
    """

    #set to +-16g resolution, bits D1 and D0 to 11 in data format register
    try:
        Zip.i2c_cmd(ADXL345, ADDRESSES["DATA_FORMAT"], 0xB, resp_len=0)
    except I2CError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    # for +-16gs, the scale factor is 32 LSB/g
    scale = 32
    try:
        Zip.actuator_move('sharp_turn')
    except ActuatorError as e:
        print(f"TEST FAILED ")
        handle_failure(e)

    data = capture_accelerometer_data(10.0,scale,scale,scale,False)

    if -8 > data.get("DATAZ"):
        passed = True
    else:
        raise TestFailure("TEST FAILED ")

    return f"TEST PASSED " if passed else ("TEST FAILED ")



if __name__ == "__main__":
    print(configure())
    print(test_selftest())
    print(test_slow_climb())
    print(test_sharp_turn())
    print(test_quick_drop())
