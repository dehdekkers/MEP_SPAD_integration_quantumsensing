from __future__ import print_function
from IPython.display import display, clear_output
from scipy.signal import find_peaks
from scipy.optimize import curve_fit
import numpy as np
import matplotlib.pyplot as plt
from laboneq.simple import *
from laboneq.controller.util import *
from time import sleep
import time
#This script counts the total array counts for a range of points in the y-axis. X and z are kept constant. 
#The main purpose of this scan is to find the focus point of the widefield setup

from datetime import date, datetime
import scipy as scipy
import os
import sys
import json

# Use array_reader_piezo for getting frames from the array
from array_reader_piezo import get_frame

plt.rcParams.update({'font.size': 24,})

def main() -> int:
    settings = {
        "save_folder": "/home/dl-lab-pc3/Documents/Diederik/array_measurements/",
        "y_steps": 50,
        "num_measurements": 1,

        "x0": 2,
        
        "z0": 0,
        "y1": 0,
        "y2": 3,

        "laser_power": 4,
        "sample": "Maarten",
        "measurement_type": "focus_y_scan_array",
    }

    save_folder      = settings["save_folder"]
    n_steps          = settings["y_steps"]
    num_meas         = settings["num_measurements"]
    x0               = settings["x0"]
    z0               = settings["z0"]
    y1               = settings["y1"]
    y2               = settings["y2"]

    # Time estimate
    est_frame = 0.01
    total_time = n_steps * (num_meas * est_frame + 0.2)
    print("Estimated time:", total_time/3600, "hours")

    # Saving path
    timestamp = str(round(datetime.now().timestamp()))
    savePath = save_folder + "y_focus_scan_OBIOS_" + timestamp
    settings["savePath"] = savePath

    # Piezo setup
    from rtcs.devices.physikinstrumente.pi_E873_controller import Pistage_controller
    pi_x = Pistage_controller("type=tcp;host=192.168.20.21;port=50000")
    pi_y = Pistage_controller("type=tcp;host=192.168.20.22;port=50000")
    pi_z = Pistage_controller("type=tcp;host=192.168.20.23;port=50000")

    pi_x.open(); pi_y.open(); pi_z.open()

    # Move to base X, Z, and starting Y
    pi_x.move(x0)
    pi_z.move(z0)
    pi_y.move(y1)
    time.sleep(2)
    pi_y.send_command("VEL 1 5")

    # Scan positions in y
    positions = np.linspace(y1, y2, n_steps)

    PL = np.zeros((n_steps, 64))

    try:
        for i in range(n_steps):
            pos = positions[i]
            pi_y.move(pos)
            pi_y.wait_on_target()
            
            if i == 0:
            	frame = np.asarray(get_frame())
            
            print(f"Step {i} of {n_steps}")
            acc = np.zeros((8, 8), float)
            for n in range(num_meas+ 1):
                frame = np.asarray(get_frame())
                if frame.shape == (64,):
                    frame = frame.reshape(8, 8)
                acc += frame

            PL[i, :] = (acc / num_meas).flatten()

    finally:
        pi_x.close(); pi_y.close(); pi_z.close()

    np.save(savePath + ".npy", PL)
    np.savetxt(savePath + ".txt", PL)

    # Save settings
    with open(savePath + ".json", "w") as f:
        json.dump(settings, f, indent=2)

    # Plotting
    total_counts = PL.sum(axis=1)

    plt.figure(figsize=(6,4))
    plt.rcParams.update({'font.size': 12})

    plt.plot(positions, total_counts, 'o-', linewidth=2)
    plt.xlabel("y (mm)", fontsize=12)
    plt.ylabel("Total array count rate (cps)", fontsize=12)
    plt.title("Widefield Y-focus scan", fontsize=14)
    plt.grid(True)

    plot_path = savePath + "_plot.png"
    plt.tight_layout()
    plt.savefig(plot_path, dpi=300)
    print("Plot saved as:", plot_path)

    plt.show()

    return 0

if __name__ == "__main__":
    exitcode = main()
    if exitcode != 0:
        sys.exit(exitcode)
