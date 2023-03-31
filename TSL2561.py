import RPi.GPIO as GPIO
import time

# Define TSL2561 register addresses
dev_addr = 0x39 # ADD sel_terminalconnected to GND
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

# Define the GPIO pins for SDA and SCL
SDA_PIN = 26
SCL_PIN = 21


# Set up the GPIO pins in BCM mode(uses GPIO numbers)
GPIO.setmode(GPIO.BCM)

#set up the SDA_PIN and SCL_PIN as output pins
GPIO.setup(SDA_PIN, GPIO.OUT)
GPIO.setup(SCL_PIN, GPIO.OUT)


# Define the I2C start signal
def i2c_start():
    GPIO.output(SDA_PIN, GPIO.HIGH)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    GPIO.output(SDA_PIN, GPIO.LOW)
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.LOW)

# Define the I2C stop signal
def i2c_stop():
    GPIO.output(SDA_PIN, GPIO.LOW)
    GPIO.output(SCL_PIN, GPIO.LOW)
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    GPIO.output(SDA_PIN, GPIO.HIGH)
    

# Define the I2C write function
def i2c_write_byte(byte):
    for bit in range(8):
        if byte & (1 << (7 - bit)):
            GPIO.output(SDA_PIN, GPIO.HIGH)
        else:
            GPIO.output(SDA_PIN, GPIO.LOW)
        GPIO.output(SCL_PIN, GPIO.HIGH)
        time.sleep(0.00000005)
        GPIO.output(SCL_PIN, GPIO.LOW)
        
# Define the acknowledge
def ACK():	
    GPIO.output(SDA_PIN, GPIO.HIGH)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    while(SDA_PIN == GPIO.HIGH):
         continue
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.LOW)
    
    
    
# Define the I2C read function
def i2c_read_byte():
    byte = 0
    for bit in range(8):
        GPIO.output(SCL_PIN, GPIO.HIGH)
        time.sleep(0.00000003)
        if GPIO.input(SDA_PIN):
            byte |= 1 << (7 - bit)
        GPIO.output(SCL_PIN, GPIO.LOW)
    return byte

# Define non-acknowledgement function
def NACK():	
    GPIO.output(SDA_PIN, GPIO.HIGH)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.LOW)



# Write to TSL2561 register
def write_byte(reg, value):
      i2c_start() # send the start signal
      i2c_write_byte(dev_addr <<1) # send the device address with 8th bit as zero for writing
      ACK() # sending the acknowledgement
      i2c_write_byte(reg | TSL2561_CMD) # specifies register address
      ACK() # sending the acknowledgement
      i2c_write_byte(value) # write the Data Byte to register reg   
      ACK() # sending the acknowledgement
      i2c_stop() # send the stop signal

# Read from TSL2561 register
def read_byte(reg):
      i2c_start() # send the start signal
      i2c_write_byte(dev_addr <<1) # send the device address with 8th bit as zero for writing
      ACK() # sending the acknowledgement
      i2c_write_byte(reg | TSL2561_CMD) # specifies register address
      ACK() # sending the acknowledgement
      i2c_start() # send the start signal 
      i2c_write_byte(dev_addr <<1 | 0x00) # write the Data Byte to register reg   
      ACK() # sending the acknowledgement
      data = i2c_read_byte() # read the 1st byte of data (MSB)
      ACK() # sending the acknowledgement
      i2c_stop() # send the stop signal
        
    return data

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
  try: 
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

    # Print lux value (lux= lumious flux/ unit area)
    print("Lux: {:.2f}".format(lux))

    #Wait for next measurement
    time.sleep(1)    
  except KeyboardInterrupt:
    print("User interrupted the program.")
    sys.exit()
  finally:
    GPIO.cleanup()
