# This script executes a scan of the total array counts along either the x or z axis, y is always fixed, along with the other unused coordinate.
# The purpose of this script is to get an estimate of the diamond location, after finding the optimal focus with y_scan_array

from __future__ import print_function
#from psutil import ...
from IPython.display import display, clear_output
#%matplotlib inline
from scipy.signal import find_peaks
from scipy.optimize import curve_fit
import numpy as np
import matplotlib.pyplot as plt
#from zhinst.utils import ...
#import Pyro5.api
#import TimeTagger as TT
# convenience import for all LabOne Q software functionality
from laboneq.simple import *
from laboneq.controller.util import *
from time import sleep
#import TimeTagger
from scipy.optimize import curve_fit
import time
from datetime import date, datetime
import scipy as scipy
#from scipy import optimize
import os
#from pipython import GCSDevice, pitools
#import argparse
import sys
#import h5py
#logging.disable(logging.DEBUG)
#from rtcs.measurements.esr_scan import single_dip_esr_fit as single_fit
#from rtcs.measurements.esr_scan import triple_dip_esr_fit as triple_fit
import json

# Use arry_reader_piezo.py to get the frames
from array_reader_piezo import get_frame

plt.rcParams.update({'font.size': 24,})

def main() -> int:
    settings = {
        "save_folder": "/home/dl-lab-pc3/Documents/Diederik/array_measurements/",
        "n_steps": 50,          # number of scan steps (for X or Z)
        "num_measurements": 1,   # number of 8x8 frames per step

        # Fixed position for y, and base positions for x and z
        "x0": 1,
        "y0": 2.81625,              # Y is fixed at all times
        "z0": -1,
      
        # X scan range (used when scan_x=True)
        "x1": 0,
        "x2": 5,

        # Z scan range (used when scan_x=False)
        "z1": -4,
        "z2": 1,

        # Choose scan axis:
        #   True  -> scan in x (x1→x2), z=z0 fixed
        #   False -> scan in z (z1→z2), x=x0 fixed
        "scan_x": False,

        "laser_power": 4, # mW
        "sample": "Maarten",
        "measurement_type": "xz_scan_array",
    }

    settings_file = None

    if(settings_file != None):
        #Load settings from specified json file
        with open(settings_file, 'r') as f:
            settings = json.load(f)

    save_folder      = settings["save_folder"]
    n_steps          = settings["n_steps"]         
    num_measurements = settings["num_measurements"]
    x0               = settings["x0"]
    y0               = settings["y0"]
    z0               = settings["z0"]
    x1               = settings["x1"]
    x2               = settings["x2"]
    z1               = settings["z1"]
    z2               = settings["z2"]
    scan_x           = settings["scan_x"]

    # Decide which axis to scan over
    if scan_x:
        axis_label = "x"
        start_pos = x1
        end_pos = x2
    else:
        axis_label = "z"
        start_pos = z1
        end_pos = z2

    # Saving the data:
    current_time = datetime.now()
    timestamp_string = str(round(current_time.timestamp()))
    savePath = (
        save_folder
        + f"{axis_label}_scan_array_"
        + timestamp_string
    )  # Without file extension yet
    settings["savePath"] = savePath
    settings["Start time"] = str(current_time)
    settings["scan_axis"] = axis_label

    #Setup piezo-stack
    from rtcs.devices.physikinstrumente.pi_E873_controller import Pistage_controller
    pi_x = Pistage_controller("type=tcp;host=192.168.20.21;port=50000") #x axis
    pi_y = Pistage_controller("type=tcp;host=192.168.20.22;port=50000") #y axis
    pi_z = Pistage_controller("type=tcp;host=192.168.20.23;port=50000") #z axis

    pi_x.open()
    pi_y.open()
    pi_z.open()

    # Move to the starting base point: fixed Y, base X/Z
    pi_x.move(x0)
    pi_y.move(y0)
    pi_z.move(z0)
    time.sleep(1)

    # Set velocity for scan axis
    if scan_x:
        pi_x.send_command("VEL 1 5")
    else:
        pi_z.send_command("VEL 1 5")

    # Generate scan positions along chosen axis
    positions = np.linspace(start_pos, end_pos, n_steps)

    # 8x8 frame is flattened and saved for every step
    PL = np.zeros((n_steps, 64))

    try:
        for i_step in range(n_steps):
            pos = positions[i_step]
            print(f" Step {i_step} of {n_steps}")
            
            if i_step == 0:
                f = get_frame()
            if scan_x:
                # Scan X, keep Y,Z fixed
                pi_x.move(pos)
                pi_x.wait_on_target()
            else:
                # Scan Z, keep X,Y fixed
                pi_z.move(pos)
                pi_z.wait_on_target()

            # Accumulate num_measurements frames and average them
            acc = np.zeros((8, 8), dtype=float)
            for n in range(num_measurements):
                frame = np.asarray(get_frame())
                if frame.shape == (64,):
                    frame = frame.reshape(8, 8)
                elif frame.shape != (8, 8):
                    raise ValueError(
                        f"get_frame() returned unexpected shape {frame.shape}. "
                        "Expected (8,8) or flat (64,)."
                    )
                acc += frame

            mean_img = acc / float(num_measurements)
            PL[i_step, :] = mean_img.flatten()

    finally:
        # Close piezos no matter what
        pi_x.close()
        pi_y.close()
        pi_z.close()

    # Save raw data
    np.save(savePath + ".npy", PL)
    np.savetxt(savePath + ".txt", PL)

    settings["End time"] = str(datetime.now())

    #Save settings in json file
    with open(savePath + ".json", 'w') as f: 
        json.dump(settings, f, indent="")

    # Plotting the scan
    total_counts = PL.sum(axis=1)

    # Create a figure with NORMAL font size
    plt.figure(figsize=(6,4))
    plt.rcParams.update({'font.size': 12})

    plt.plot(positions[1:], total_counts[1:], 'o-', linewidth=2)
    plt.xlabel(f"{axis_label} (mm)", fontsize=12)
    plt.ylabel("Total array count rate (cps)", fontsize=12)
    plt.title(f"Widefield {axis_label.upper()}-scan", fontsize=14)
    plt.grid(True)

    # Save plot
    plot_path = savePath + "_plot.png"
    plt.savefig(plot_path, dpi=300)
    print(f"Plot saved as: {plot_path}")
    # Show plot
    plt.show()
    
    print("Measurement complete!\nFile saved as: " + savePath + ".npy")

    return 0


if __name__ == "__main__":
    exitcode = main()
    if exitcode != 0:
        sys.exit(exitcode)
