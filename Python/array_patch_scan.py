# DISCLAIMER: This script was partly generated using the GPT-5 AI model, thorough checks were done by the author before using any AI-generated routines.
# This script performs a patched PL scan for widefield imaging of the diamond sample.
# It reads out num_sweeps per patch, and moves the piezo stage accordingly.

from __future__ import annotations

import numpy as np
from datetime import datetime
import time
import os
import json

from array_reader_piezo import get_frame
from rtcs.devices.physikinstrumente.pi_E873_controller import Pistage_controller

settings = {
    # Saving
    "save_folder": "/home/dl-lab-pc3/Documents/Diederik/array_measurements/",
    "label": "array_patch_scan",

    # Acquisition
    "num_counts": 1,              # frames per sweep per patch
    "num_sweeps": 1,                # sweeps per patch
    "sleep_between_frames_s": 0.1,  # optional pause between frames

    #Time estimation
    "estimated_time_per_frame_s": 3,

    # Patch geometry (X–Z plane)
    "side_patch": 37e-3,              # size of a patch, corresponds to one side of a square FOV, in mm
    "scan_size_x": 1000e-3,            # total scan size in x, centered on x0
    "scan_size_z": 1000e-3,            # total scan size in z, centered on z0

    # Starting point: center of the full scan
    "x0": 1,                      # center of scan in x
    "y0": 2.81625,                      # Y is fixed
    "z0": -1,                    # center of scan in z

    # Piezo controller
    "pi_x_address": "type=tcp;host=192.168.20.21;port=50000",
    "pi_y_address": "type=tcp;host=192.168.20.22;port=50000",
    "pi_z_address": "type=tcp;host=192.168.20.23;port=50000",

    # Piezo behaviour
    "piezo_velocity": 5.0,          # VEL 1 <velocity> for all axes
    "steps_to_autozero": 1000,      # number of patch-moves before running autozero cycle
}


#Computing the patches to be scanned

def compute_patch_grid(center0_1: float, center0_2: float,
                       scan_size_1: float, scan_size_2: float,
                       side_patch: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute centers of patches in two dimensions.

    The scan is centered around (center0_1, center0_2).
    scan_size_1 and scan_size_2 are the total sizes.
    side_patch is the size of each (square) patch.

    Returns the arrays with patches
    """
    n_patches_1 = max(1, int(round(scan_size_1 / side_patch)))
    n_patches_2 = max(1, int(round(scan_size_2 / side_patch)))

    offsets_1 = (np.arange(n_patches_1) - (n_patches_1 - 1) / 2.0) * side_patch
    offsets_2 = (np.arange(n_patches_2) - (n_patches_2 - 1) / 2.0) * side_patch

    centers1 = center0_1 + offsets_1
    centers2 = center0_2 + offsets_2

    return centers1, centers2


#Piezo stack class

class PiezoStack:
    """
    Thin wrapper around the three Pistage_controller axes
    """

    def __init__(self, settings: dict):
        self.settings = settings

        self.pi_x = Pistage_controller(settings["pi_x_address"])
        self.pi_y = Pistage_controller(settings["pi_y_address"])
        self.pi_z = Pistage_controller(settings["pi_z_address"])

        self.pi_x.open()
        self.pi_y.open()
        self.pi_z.open()

        vel = settings["piezo_velocity"]
        # Set velocities of piezo stack (exact commands as in ODMR_2D.py)
        self.pi_x.send_command(f"VEL 1 {vel}")
        self.pi_y.send_command(f"VEL 1 {vel}")
        self.pi_z.send_command(f"VEL 1 {vel}")

    def full_autozero(self):
        """
        Run the full autozero sequence for stability
        """
        pi_x, pi_y, pi_z = self.pi_x, self.pi_y, self.pi_z

        pi_z.move(0)
        pi_y.move(0)
        pi_x.move(0)
        time.sleep(3)
        pi_x.autozero()
        time.sleep(10)
        pi_y.autozero()
        time.sleep(10)
        pi_z.autozero()
        time.sleep(10)

    def _debug_status(self):
        """
        Debug status helper
        """
        pi_x, pi_y, pi_z = self.pi_x, self.pi_y, self.pi_z

        for axis, name in ((pi_x, "x"), (pi_y, "y"), (pi_z, "z")):
            axis.send_command("SVO?")
            response = axis._readline()
            print(f"{name} SVO?: {response}")

            axis.send_command("ERR?")
            response = axis._readline()
            print(f"{name} ERR?: {response}")

            axis.send_command("VEL?")
            response = axis._readline()
            print(f"{name} VEL?: {response}")

            axis.send_command("ONT?")
            response = axis._readline()
            print(f"{name} ONT?: {response}")

            axis._transport.write(b'\x05')
            response = axis._readline()
            print(f"{name} #5 (motion status): {response}")

    def move_and_wait(self, x: float, y: float, z: float,
                      timeout: float | None = 10.0):
        """
        Move all three axes and wait for on-target state.

        Implements the same waiting logic and timeout recovery pattern
        as in ODMR_2D.py, adapted to the patch-based scan.
        """
        pi_x, pi_y, pi_z = self.pi_x, self.pi_y, self.pi_z

        pi_x.move(x)
        pi_y.move(y)
        pi_z.move(z)

        start_time = time.time()
        while (not pi_x.get_on_target_state()
               or not pi_y.get_on_target_state()
               or not pi_z.get_on_target_state()):
            time.sleep(0.005)
            if timeout is not None and time.time() > start_time + timeout:
                print()
                print("Warning: Timeout passed for wait_on_target! Giving up and attempting recovery.")
                print("Real position: ({}, {}, {})".format(
                    pi_x.get_real_position(),
                    pi_y.get_real_position(),
                    pi_z.get_real_position(),
                ))
                print("Target: ({}, {}, {})".format(
                    pi_x.get_target_position(),
                    pi_y.get_target_position(),
                    pi_z.get_target_position(),
                ))

                # Anti-stuck procedure (identical diagnostics as ODMR_2D.py)
                self._debug_status()

                # Autozero and attempt to move back
                self.full_autozero()
                pi_x.move(x)
                pi_y.move(y)
                pi_z.move(z)
                time.sleep(3)
                print("Real position after recovery: ({}, {}, {})".format(
                    pi_x.get_real_position(),
                    pi_y.get_real_position(),
                    pi_z.get_real_position(),
                ))
                break

    def close(self):
        self.pi_x.close()
        self.pi_y.close()
        self.pi_z.close()


#Acquire a single patch

def acquire_patch(piezo: PiezoStack,
                  x_center: float,
                  y_fixed: float,
                  z_center: float,
                  ix: int,
                  iz: int,
                  frames: np.ndarray,
                  timestamps: np.ndarray,
                  positions: np.ndarray,
                  settings: dict):
    """
    Acquire data for a single patch located at (x_center, y_fixed, z_center).

    Parameters
    ----------
    piezo : PiezoStack
        The piezo hardware wrapper.
    x_center : float
        X coordinate (patch center).
    y_fixed : float
        Y coordinate (fixed).
    z_center : float
        Z coordinate (patch center).
    ix, iz : int
        Patch indices in the X and Z directions.
    frames, timestamps, positions : np.ndarray
        Pre-allocated arrays that will be filled in-place.
    settings : dict
        Global settings dict.
    """
    num_sweeps = int(settings["num_sweeps"])
    num_counts = int(settings["num_counts"])
    sleep_between = float(settings["sleep_between_frames_s"])

    use_tilt_plane = bool(settings.get("use_tilt_plane", False))
    x_ref = float(settings["x0"])
    y_ref = float(settings["y0"])
    z_ref = float(settings["z0"])
    ax = float(settings["ax"])
    ay = float(settings["ay"])

    # Move to patch center and wait for stability
    piezo.move_and_wait(x_center, y_fixed, z, timeout=10.0)

    # Store the patch center position
    positions[ix, iz, :] = (x_center, y_fixed, z)

    print(f"Patch ({ix+1}, {iz+1}) -> x={x_center:.6f}, y={y_fixed:.6f}, z={z:.6f}")

    # Acquisition loops
    for s in range(num_sweeps):
        print(f"  Sweep {s+1}/{num_sweeps}")
        if s == 0:
            f = get_frame()
        for c in range(num_counts):
            f = get_frame()
            f = np.asarray(f)
            if f.shape == (64,):
                f = f.reshape(8, 8)
            elif f.shape != (8, 8):
                raise ValueError(
                    f"get_frame() returned unexpected shape {f.shape}. "
                    "Expected (8,8) or flat (64,)."
                )

            frames[ix, iz, s, c] = f
            timestamps[ix, iz, s, c] = np.datetime64(datetime.now(), "ns")

            if (c + 1) % max(1, num_counts // 5) == 0 or c == num_counts - 1:
                print(
                    f"    [count {c+1}/{num_counts}] "
                    f"mean={f.mean():.2f} max={f.max()}        ",
                    end="\r",
                    flush=True,
                )

            if sleep_between > 0:
                time.sleep(sleep_between)
        print()  # newline after sweep


#Format seconds utility 
def format_seconds(sec: float) -> str:
    sec = int(round(sec))
    h = sec // 3600
    m = (sec % 3600) // 60
    s = sec % 60
    if h > 0:
        return f"{h:d}h {m:02d}m {s:02d}s"
    elif m > 0:
        return f"{m:d}m {s:02d}s"
    else:
        return f"{s:d}s"


#Main function that performs the total scan

def main() -> int:
    # Create save folder
    os.makedirs(settings["save_folder"], exist_ok=True)

    # Patch grid
    side_patch = float(settings["side_patch"])
    scan_size_x = float(settings["scan_size_x"])
    scan_size_z = float(settings["scan_size_z"])
    x0 = float(settings["x0"])
    y_fixed = float(settings["y0"])
    z0 = float(settings["z0"])

    x_centers, z_centers = compute_patch_grid(x0, z0, scan_size_x, scan_size_z, side_patch)
    n_patches_x = len(x_centers)
    n_patches_z = len(z_centers)

    settings["n_patches_x"] = n_patches_x
    settings["n_patches_z"] = n_patches_z

    num_counts = int(settings["num_counts"])
    num_sweeps = int(settings["num_sweeps"])
    sleep_between = float(settings["sleep_between_frames_s"])
    est_frame_time = float(settings.get("estimated_time_per_frame_s", 0.0))

    # Allocate storage
    frames = np.empty((n_patches_x, n_patches_z, num_sweeps, num_counts, 8, 8), dtype=np.int64)
    timestamps = np.empty((n_patches_x, n_patches_z, num_sweeps, num_counts), dtype="datetime64[ns]")
    positions = np.empty((n_patches_x, n_patches_z, 3), dtype=float)

    # Metadata: start time
    t0 = datetime.now()
    settings["start_time"] = t0.isoformat()

    # Build save path
    timestamp_str = str(round(t0.timestamp()))
    basepath = os.path.join(
        settings["save_folder"],
        f"{settings['label']}_{timestamp_str}",
    )

    # Rough time estimate
    total_frames = n_patches_x * n_patches_z * num_sweeps * num_counts
    est_per_frame = est_frame_time + sleep_between
    est_total_seconds = total_frames * est_per_frame

    print("Planned scan configuration:")
    print(f"  Patches: {n_patches_x} (X) x {n_patches_z} (Z) = {n_patches_x * n_patches_z} total")
    print(f"  Frames per patch: num_sweeps={num_sweeps}, num_counts={num_counts}")
    print(f"  Total frames: {total_frames}")
    print(f"  Estimated time / frame: {est_per_frame:.3f} s "
          f"(frame acquisition ~{est_frame_time:.3f} s + sleep {sleep_between:.3f} s)")
    print(f"  Rough estimated total measurement time: {format_seconds(est_total_seconds)}")

    # Setup piezo (same controller type and parameters as ODMR_2D.py)
    piezo = PiezoStack(settings)
    steps_to_autozero = int(settings["steps_to_autozero"])
    steps_since_last_autozero = 0

    try:
        print("Starting array + piezo patch scan (xz plane, y fixed)...")
        print(f"Fixed Y position: y0 = {y_fixed:.6f}")

        for ix, x_center in enumerate(x_centers):
            for iz, z_center in enumerate(z_centers):
                # Periodic autozero, identical idea as in ODMR_2D.py
                if steps_since_last_autozero >= steps_to_autozero:
                    print("Performing periodic full autozero of piezo stack...")
                    piezo.full_autozero()
                    steps_since_last_autozero = 0
                steps_since_last_autozero += 1

                acquire_patch(
                    piezo=piezo,
                    x_center=x_center,
                    y_fixed=y_fixed,
                    z_center=z_center,
                    ix=ix,
                    iz=iz,
                    frames=frames,
                    timestamps=timestamps,
                    positions=positions,
                    settings=settings,
                )

        print("Scan complete.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received! Stopping scan early.")
        # We don't save partial data automatically here, but piezo will still be closed in finally.
        # If you want to save partially acquired data, you could move the saving code into this block.
    finally:
        # Always close the piezo connections, even on interrupt or error
        print("Closing piezo connections...")
        piezo.close()

    # Metadata: end time
    t1 = datetime.now()
    settings["end_time"] = t1.isoformat()

    # Prepare dict for saving
    save_dict = {
        "frames": frames,
        "timestamps": timestamps,
        "positions": positions,
        "x_centers": x_centers,
        "z_centers": z_centers,
        "settings": settings,
    }

    np.save(basepath + ".npy", save_dict, allow_pickle=True)
    print(f"Saved full dataset to: {basepath}.npy")

    # Also store settings as JSON for easy manual inspection
    with open(basepath + "_settings.json", "w") as f:
        json.dump(settings, f, indent=2)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
