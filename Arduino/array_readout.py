# This code contains the read/write operations for the 8x8 SPAD array
# This script first configures some settings, and starts a while loop that continuously read out the array
# The code is written in micropython, suitable for operation on the Arduino Nano ESP32 or any Arduino module that works with the ESP32 chip
# For confidentiality reasons, register addressing is standardized

from machine import Pin, I2C
import time 
import sys
#I2C addresses and configuration
OBIOS = 0x00 #Example address  
I2C_freq = 100000 

#Initialization
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq = I2C_freq)


#Writing function
def OBIOS_write(register , data):
  
  if isinstance(data, int):
    data = bytes([data])
  elif isinstance(data, list):
    data = bytes(data)

  buf = data
  nbytes = len(buf)

  #Direct memory writing operation instead of a repeated start
  i2c.writeto_mem(OBIOS , register , buf , addrsize=8)
  
  return None



#Reading function
def OBIOS_read(register , nbytes):

  #Read nbytes from starting register
  val = i2c.readfrom_mem(OBIOS, register , nbytes)
  
  return val

#Trimming oscillator to 25Mhz
trimming = 0x00 #Example value

OBIOS_write(trimming , 63)

#Quenching delay
quench_delay = 0x00 #Example value

OBIOS_write(quench_delay, 4)

#Timebase
timebase = 0x00 #Example value
timebase_value = 3

OBIOS_write(timebase , timebase_value)

clock_period = 0.5 #Example value, in practice this is determined by the oscillator trimming
integration_time = timebase_value * clock_period

#Checksum encryption
def crc16(data, poly = 0x1021, init_val = 0xFFFF):
  crc = init_val
  for byte in data:
    crc ^= (byte<< 8) #Fold each byte into the CRC
    for _ in range(8):
      if crc & 0x8000:
        crc = (crc << 1) ^ poly
      else: 
        crc <<= 1

      crc &= 0xFFFF
  return crc
  
try:
  while True:
    # #Read all the SPADs in one transmission
    readings = OBIOS_read(0x40,192)

    #Compute checksum and send
    crc = crc16(readings)
    
    #Build frame
    frame = bytes([0xFF,0xFF,0xFF,0xFF]) + readings + crc.to_bytes(2, ' big') 

    #Send over through serial
    sys.stdout.buffer.write(frame)
    

    time.sleep(integration_time - 0.01)
except KeyboardInterrupt:
  print("Program stopped manually")
