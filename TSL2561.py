import smbus
import time

# Define TSL2561 register addresses
TSL2561_ADDR = 0x39
TSL2561_CMD = 0x80
TSL2561_REG_CONTROL = 0x00
TSL2561_REG_TIMING = 0x01
TSL2561_REG_THRESHLOWLOW = 0x02
TSL2561_REG_THRESHLOWHIGH = 0x03
TSL2561_REG_THRESHHIGHLOW = 0x04
TSL2561_REG_THRESHHIGHHIGH = 0x05
TSL2561_REG_INTERRUPT = 0x06
TSL2561_REG_ID = 0x0A
TSL2561_REG_DATA0LOW = 0x0C
TSL2561_REG_DATA0HIGH = 0x0D
TSL2561_REG_DATA1LOW = 0x0E
TSL2561_REG_DATA1HIGH = 0x0F

# Initialize the I2C bus
bus = smbus.SMBus(1)

# Write to TSL2561 register
def write_byte(reg, value):
    bus.write_byte_data(TSL2561_ADDR, reg | TSL2561_CMD, value)

# Read from TSL2561 register
def read_byte(reg):
    return bus.read_byte_data(TSL2561_ADDR, reg | TSL2561_CMD)

# Read 16-bit value from two registers, LSB first
def read_word(reg):
    low = read_byte(reg)
    high = read_byte(reg+1)
    return (high << 8) | low

# Set the integration time and gain
def set_integration_time_gain(int_time, gain):
    # Set the integration time
    write_byte(TSL2561_REG_TIMING, int_time)

    # Set the gain
    write_byte(TSL2561_REG_TIMING, gain)

# Configure TSL2561 for continuous measurement
write_byte(TSL2561_REG_CONTROL, 0x03)

# Set integration time to 402ms and gain to 1x
set_integration_time_gain(0x02, 0x00)

# Read light sensor values
while True:
    # Read raw sensor values
    data0 = read_word(TSL2561_REG_DATA0LOW)
    data1 = read_word(TSL2561_REG_DATA1LOW)

    # Calculate lux value
    if data0 == 0:
        lux = 0
    else:
        ratio = float(data1) / float(data0)
        if 0 < ratio <= 0.50:
            lux = 0.0304 * data0 - 0.062 * data0 * (ratio**1.4)
        elif 0.50 < ratio <= 0.61:
            lux = 0.0224 * data0 - 0.031 * data1
        elif 0.61 < ratio <= 0.80:
            lux = 0.0128 * data0 - 0.0153 * data1
        elif 0.80 < ratio <= 1.30:
            lux = 0.00146 * data0 - 0.00112 * data1
        else:
            lux = 0

    # Print lux value
    print("Lux: {:.2f}".format(lux))

    # Wait for next measurement
    time.sleep(1)
