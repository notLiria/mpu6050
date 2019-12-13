"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer

CHANGES 2019/11/16
- Added documentation
- Added read_i2c_byte(self, register)
- Added read_i2c_bit(self, register, bitNum)
- Added read_i2c_bits(self, register, bitStart, length)
- Added get_accel_x(self)
- Added get_accel_y(self)
- Added get_accel_z(self)
- Added get_gyro_x(self)
- Added get_gyro_y(self)
- Added get_gyro_z(self)
- Added reset(self)
"""



"""
Method List
read_i2c_byte(self, register): Reads a byte from register
read_i2c_bit(self, register, bitNum): Reads bitNum from register
read_i2c_bits(self, register, bitStart, length): Reads length bits from register, starting at bitNum
read_i2c_word(self, register): Reads a byte from register
write_i2c_bit(self, register, bitNum, value): Writes value to bitNum on register
write_i2c_bits(self, register, bitStart, length): Writes length bits to bitNum on register
get_temp(self): Reads the temperature from onboard temperature sensor
set_accel_range(self, accel_range): Sets the accelerometer to range
read_accel_range(self, raw): Reads the range the accelerometer is set to
get_accel_data(self): Returns the x, y, z values from the accelerometer, scaled
get_accel_x(self): Returns the raw x value from the accelerometer 
get_accel_y(self): Returns the raw y value from the accelerometer 
get_accel_z(self): Returns the raw z value from the accelerometer 
set_gyro_range(self, gyro_range): Sets gyroscope range to gyro_range
read_gyro_range(self, raw): Reads the range the gyroscope is set to
get_gyro_data(self): Gets and returns x, y, and z values from the gyroscope
get_gyro_x(self): Returns the raw x value from the gyro
get_gyro_y(self): Returns the raw y value from the gyro
get_gyro_z(self): Returns the raw z value from the gyro
get_all_data(self): Reads and returns all available data
reset(self): Resets the device
get_sleep_enabled(self): Checks if sleep is enabled
set_sleep_enabled(self, enabled): Set sleep mode status
get_wake_cycle_enabled(self): Get wake cycle enabled status
set_wake_cycle_enabled(self, enabled): set wake cycle enabled status
get_temp_sensor_enabled(self): Get temp sensor enabled status
set_temp_sensor_enabled(self, enabled): Set temp senosr enabled status
get_clock_source(self): Gets the clock source
"""



import smbus

class mpu6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    #Added 2019/11/16
    PWR1_DEVICE_RESET_BIT = 7
    PWR1_DEVICE_SLEEP_BIT = 6
    PWR1_CYCLE_BIT = 5
    PWR1_TEMP_DIS_BIT = 3
    PWR1_CLKSEL_BIT = 2
    PWR1_CLKSEL_LENGTH = 3

    PWR2_LP_WAKE_CTRL_BIT = 7
    PWR2_LP_WAKE_CTRL_LENGTH = 2
    PWR2_STBY_XA_BIT = 5
    PWR2_STBY_YA_BIT = 4
    PWR2_STBY_XG_BIT = 3
    PWR2_STBY_YG_BIT = 1
    PWR2_STBY_ZG_BIT = 0
    

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    # I2C communication methods


    def read_i2c_byte(self, register):
        """
        Read one byte from the specified register
        Inputs: register -- the register to read
        Outputs: The read result
        """
        return self.bus.read_byte_data(self.address, register)
   
    def read_i2c_bit(self, register, bitNum):
        """
        Reads the bitNum-th bit from register
        Inputs: register - the register to read
                bitNum - the bit to read
        OutputS: The bit at bitNum 
        """
        data = self.read_i2c_byte(register)
        return data &  (1 << bitNum)

    def read_i2c_bits(self, register, bitStart, length):
        """
        Read multiple bits from an 8-bit device register.
        Inputs: register - the register to read
                bitStart - First bit position to read (0-7)
                length - number of bits to read (no more than 8)
        """
        data = self.read_i2c_byte(register)
        if (data != 0):
            mask = ((1 << length) - 1) << (bitStart - length + 1)
            data &= mask
            data >>= (bitStart - length + 1)
            return data
        
        
    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def write_i2c_bit(self, register, bitNum, value):
        data = self.read_i2c_byte(register)
        if (data != 0):
            data = (data | (1 << bitNum))
        else:
            data = (data & ~(1 << bitNum))
        self.bus.write_byte_data(self.address, register, data)
        

    def write_i2c_bits(self, register, bitStart, length, data):
        b = self.read_i2c_byte(self.address)
        if (data != 0):
            mask = ((1 << length) - 1) << (bitStart - length + 1)
            data <<= (bitStart - length + 1)
            data &= mask
            b &= ~(mask)
            b |= data
            return self.bus.write_byte_data(self.address, register, b)
            

    # MPU-6050 Methods

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.

        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer., scaled

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}
        
    def get_accel_x(self):
        """ 
        Gets and returns the raw X value from the acceloremetr
        Returns x-axis acceleration measurement in 16 bit
        """
        return self.read_i2c_word(self.ACCEL_XOUT0)

    def get_accel_y(self):
        """ 
        Gets and returns the raw y value from the acceloremetr
        Returns y-axis acceleration measurement in 16 bit 
        """
        return self.read_i2c_word(self.ACCEL_YOUT0)

    def get_accel_z(self):
        """ 
        Gets and returns the raw z value from the acceloremetr
        Returns z-axis acceleration measurement in 16 bit 
        """
        return self.read_i2c_word(self.ACCEL_ZOUT0)

        
    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.

        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.

        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
o        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': x, 'y': y, 'z': z}

    def get_gyro_x(self):
        """
        Gets and returns the raw x value from the gyroscope
        Returns x-axis acceleration measurement in 16 bit 
        """
        return self.read_i2c_word(self.GYRO_XOUT0)

    def get_gyro_y(self):
        """
        Gets and returns the raw y value from the gyroscope
        Returns y-axis acceleration measurement in 16 bit 
        """
        return self.read_i2c_word(self.GYRO_YOUT0)
    
    def get_gyro_z(self):
        """
        Gets and returns the raw z value from the gyroscope
        Returns z-axis acceleration measurement in 16 bit 
        """
        return self.read_i2c_word(self.GYRO_ZOUT0)

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro, temp]

    #PWR_MGMT_1 Register
    def reset(self):
        """
        Triggers a device reset. A small delay of ~50ms may be desirable after triggering a reset.
        """
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x01)

    def get_sleep_enabled(self):
        """
        Gets sleep mode status 
        """
        return self.read_i2c_bit(self.PWR_MGMT_1, self.PWR_1_SLEEP_BIT)

    def set_sleep_enabled(self, enabled):
        """
        Sets the sleep mode status
        """
        self.write_i2c_bit(self.address, self.PWR_MGMT_1, self.PWR1_SLEEP_BIT, enabled)

    def get_wake_cycle_enabled(self):
        """
        Gets the wake cycle enabled status
        """
        return self.read_i2c_bit(self.PWR_MGMT_1, self.PWR_1_CYCLE_BIT)

    def set_wake_cycle_enabled(self, enabled):
        """
        Sets the wake cycle enabled status
        """
        self.write_i2c_bit(self.address, self.PWR_MGMT_1, self.PWR_1_CYCLE_BIT, enabled)


    def get_temp_sensor_enabled(self):
        """
        Get temperature sensor enabled status.
        Note: This register stores the disabled value, but for consistency 
        with the rest of the code, the function is named and used with 
        standard true/false values to indicate whether the sensor is 
        enabled or disabled, respectively.
        """
        return (self.read_i2c_bit(self.PWR_MGMT_1, self.PWR1_TEMP_DIS_BIT)) == 0
        

    def set_temp_sensor_enabled(self, enabled):
        """
        Set temperature sensor enabled status.
        Note: This register stores the disabled value, but for consistency 
        with the rest of the code, the function is named and used with 
        standard true/false values to indicate whether the sensor is 
        enabled or disabled, respectively.
        """
        self.write_i2c_bit(self.address, self.PWR_MGMT_1, self.PWR_1_TEMP_DIS_BIT, ~enabled)

    def get_clock_source(self):
        """
        Get clock source setting
        """
        return self.read_i2c_bits(self.address, self.PWR_MGMT_1, self.PWR1_CLKSEL_BIT, self.PWR1_CLKSEL_LENGTH)

    
        


if __name__ == "__main__":
    mpu = mpu6050(0x68)
    print(mpu.get_temp())
    accel_data = mpu.get_accel_data()
    print(accel_data['x'])
    print(accel_data['y'])
    print(accel_data['z'])
    gyro_data = mpu.get_gyro_data()
    print(gyro_data['x'])
    print(gyro_data['y'])
    print(gyro_data['z'])
