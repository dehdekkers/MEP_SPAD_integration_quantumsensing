# This script is executed for basic frame-per-frame readout of the SPAD array, once started, it continuously shows frames until a Keyboardinterrupt
# Before running, the piezo stack position is also adjustable, this is not possible when the loop has already started
# This script will only work if the Arduino is also reading out the SPAD array

import serial
import struct
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from mpl_toolkits.axes_grid1 import make_axes_locatable
import time

#SPAD array characteristics for count correction
dcr = np.array([]) #Your 8x8 array of values here


pdp_red = np.array([]) #Your 8x8 array of values here


def array_uniform(pixels):
	imaging_pixels = pixels - dcr
	#imaging_pixels /= pdp_red_24V
	return imaging_pixels

# --- Optional piezo stage settings ---
# Set these BEFORE running the script. They are not changed at runtime.
USE_PIEZO = True  # Set to False if you want to run without moving the stage

PIEZO_SETTINGS = {
    "x0": 1,       # target X position (in piezo units, e.g. mm)
    "y0": 2.81625,       # target Y position
    "z0": -1,      # target Z position
    "velocity": 5.0, # VEL 1 <velocity> for all axes
    # Controller addresses (same style as in previous scripts)
    "pi_x_address": "type=tcp;host=192.168.20.21;port=50000",
    "pi_y_address": "type=tcp;host=192.168.20.22;port=50000",
    "pi_z_address": "type=tcp;host=192.168.20.23;port=50000",
}

# Plot settings
plt.rcParams.update({'font.size': 18,})
plt.rcParams.update({'figure.autolayout': True})

# Configure the serial port
serial_port = '/dev/ttyACM5'  # Change as needed for your system
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1.0)
timeout = 1.0 #seconds. Maximum amount of time the script will keep retrying getting data when all it receives is gibberish.
max_attempts = 50 #Maximum number of times that the get_frame function may attempt to retrieve a frame in one call.

#SPAD settings, change these according to what is in the arduino code
timebase_value = 3
integration_time = timebase_value * 0.16
quenching_value = 4

#Checksum encryption 
def crc16(data, poly = 0x1021, init_val = 0xFFFF):
  crc = init_val
  for byte in data:
    crc ^= (byte << 8) #Fold each byte into the CRC
    for _ in range(8):
      if crc & 0x8000:
        crc = (crc << 1) ^ poly
      else: 
        crc <<= 1

      crc &= 0xFFFF
  return crc

def threshold_test(data, int_time, quenching):
    #Compute max counts:
    holdoff_time = (quenching * 0.5e-6) * 1.6 + 0.5e-6 #Conversion with 25Mhz clock instead of 40 Mhz
    max_counts = int_time / holdoff_time

    if np.max(data) > max_counts:
        return False
    return True

#Find a clean frame and print it
def get_frame():	
    bytes_per_frame = 192
    collecting = False
    current_attempt = 0

    data = np.zeros((8, 8))

    try:
        while current_attempt < max_attempts:
            current_attempt += 1
            try:
                flag_byte_counter = 0
                while collecting == False:
                    data_bytes = ser.read(4)
                    start_flag = int.from_bytes(data_bytes , byteorder = 'big')
                    if(start_flag == 0xFFFFFFFF):
                        collecting = True

                t0 = time.time()
                #Read all 192 data bytes and transform into 24-bit integers
                data_bytes = ser.read(bytes_per_frame)
                counts = np.array([int.from_bytes(data_bytes[i:i+3] , 'little' ) for i in range(0, bytes_per_frame, 3)]) 

                array_counts = counts.reshape(8,8)

                #Perform CRC and threshold checks
                crc_arduino = int.from_bytes(ser.read(2), 'big')
                crc_PC = crc16(data_bytes)
                #print(" CRC checks: " , crc_arduino, crc_PC)
                threshold_test_bool = threshold_test(array_counts, integration_time, quenching_value)
                end_flag = ser.read(4)
                end_flag_number = struct.unpack('>I', end_flag)[0]

                if threshold_test_bool and end_flag_number == 0xFFFFFFFF and crc_arduino == crc_PC:
                    uniform_counts = array_counts - dcr_current_alignment_3
                    cps_uniform = uniform_counts / (integration_time)
                    cps_raw = array_counts / (integration_time)
                    return cps_raw #Change this depending on the type of scan
                print(f'CRC: {crc_arduino == crc_PC}, Threshold test: {threshold_test_bool}')
            except struct.error:
                print("No Signal! Retrying (" + str(current_attempt) + "/" + str(max_attempts) + ")")

        #If the code reaches here, maximum attempts have been reached.
        print("Maximum number of attempts reached!")
        return data
    except KeyboardInterrupt:
        print("Exiting from get_frame().")
        return data

def main():
    # --- Optional piezo setup: move to fixed (x0, y0, z0) once, then just read frames ---
    pi_x = pi_y = pi_z = None
    if USE_PIEZO:
        from rtcs.devices.physikinstrumente.pi_E873_controller import Pistage_controller

        x0 = float(PIEZO_SETTINGS["x0"])
        y0 = float(PIEZO_SETTINGS["y0"])
        z0 = float(PIEZO_SETTINGS["z0"])
        vel = float(PIEZO_SETTINGS["velocity"])

        pi_x = Pistage_controller(PIEZO_SETTINGS["pi_x_address"])
        pi_y = Pistage_controller(PIEZO_SETTINGS["pi_y_address"])
        pi_z = Pistage_controller(PIEZO_SETTINGS["pi_z_address"])

        pi_x.open()
        pi_y.open()
        pi_z.open()

        # Set velocities like in earlier scripts
        pi_x.send_command(f"VEL 1 {vel}")
        pi_y.send_command(f"VEL 1 {vel}")
        pi_z.send_command(f"VEL 1 {vel}")

        print(f"Moving piezo stack to fixed point: x={x0}, y={y0}, z={z0}")
        pi_x.move(x0)
        pi_y.move(y0)
        pi_z.move(z0)

        # Wait until all axes are on target
        pi_x.wait_on_target()
        pi_y.wait_on_target()
        pi_z.wait_on_target()
        print("Piezo stack on target. Starting live OBIOS display from this point.")

    fig = plt.figure()

    ax = fig.add_subplot(111)
    div = make_axes_locatable(ax)
    cax = div.append_axes('right', '5%', '5%')
    data = np.zeros((8, 8))
    im = ax.imshow(data)
    cb = fig.colorbar(im, cax=cax)
    cb.set_label("Counts", rotation=270, fontweight ="bold", labelpad=30)
    ax.set_xlabel("column")
    ax.set_ylabel("row")
    tx = ax.set_title('Frame 0')

    def animate(i):
        cax.cla()
        data = get_frame()
        im = ax.imshow(data)
        cb = fig.colorbar(im, cax=cax)
        cb.set_label("Counts", rotation=270, fontweight ="bold", labelpad=30)
        tx.set_text('Frame {0}'.format(i))

    ani = animation.FuncAnimation(fig, animate)

    try:
        plt.show()
    finally:
        # Always close piezo connections if they were opened
        if USE_PIEZO and pi_x is not None:
            print("Closing piezo connections...")
            pi_x.close()
            pi_y.close()
            pi_z.close()

# Example of how to call the main function with your get_frame() function
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        ser.close()
