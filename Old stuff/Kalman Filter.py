import numpy as np
import matplotlib.pyplot as plt
import re

# =============================================================================
# Kalman Filter (position-only measurement) with timestamp-based variable dt
#
# CONTEXT
#   This script is a modified version of a previous capstone’s Python KF demo.
#   We adapted it to work with our MATLAB simulation export format (MateoV5-style).
#
# KEY CHANGES VS ORIGINAL
#   1) Uses the timestamp in each line to build the time array instead of assuming
#      a fixed 0.1 s timestep. This supports variable scan spacing. (Fix A)
#
#   2) Uses only POSITION measurements in the KF update step (H = [1 0]).
#      The "mps" values in our exported files may be forced to 0 (by design),
#      so they are not meaningful as measurements.
#
#   3) For speed visualization (and initialization), we compute speed via
#      finite difference of position: v ≈ Δposition / Δtime.
#      This lets us plot “true-ish” speed even if exported mps is constant zero.
#
# EXPECTED INPUT FORMAT (MateoV5-style)
#     0.26s - "m":   20229.623030
#     0.26s - "mps": 0.000000
#
# OUTPUTS
#   - Plot 1: Position (exported) vs noisy measurement vs KF estimate
#   - Plot 2: Speed (finite-diff or exported) vs KF estimated speed
#
# LIMITATIONS / IMPORTANT NOTES
#   - This is a 1D KF on a single scalar m (range or downrange).
#     It is NOT a full 3D tracker and NOT multi-target.
#   - If your exporter picks the “wrong” detection in a scan (clutter/other target),
#     you’ll see jumps in position which the KF may smooth but cannot “fix”.
# =============================================================================
# -------------------------------------------------------------------------
# NOTE ON "mps" (SPEED) FIELD RETENTION
#
# The MateoV5 export format historically includes both:
#   - "m"   : position measurement (meters)
#   - "mps" : speed measurement (meters/second)
#
# In this project, the MATLAB tracking pipeline exports "mps" primarily for
# INTERFACE COMPATIBILITY with prior capstone and analysis scripts.
#
# IMPORTANT DESIGN DECISION:
#   - The Kalman Filter implemented below intentionally DOES NOT treat "mps"
#     as an independent measurement.
#   - In many scenarios (especially degraded / rainy conditions), exported
#     speed values may be:
#         * forced to zero,
#         * finite-difference derived from position,
#         * or otherwise unreliable as a true sensor measurement.
#
# Using such a signal as a KF measurement would violate independence
# assumptions and can degrade estimation performance.
#
# Therefore:
#   - "mps" is retained for traceability and backward compatibility.
#   - Velocity is estimated internally by the Kalman Filter using a
#     constant-velocity process model.
#   - For visualization only, finite-difference speed derived from position
#     is used when exported "mps" contains no useful information.
#
# This mirrors real-world sensor fusion pipelines, where legacy data fields
# may be preserved even when their semantics change between experiments.
# -------------------------------------------------------------------------

# Select which exported file to run
INPUT_FILE = "MateoV5_nonideal.txt"
# INPUT_FILE = "MateoV5_ideal.txt"

# -----------------------------------------------------------------------------
# Parse MateoV5-style lines into synchronized triplets: (t, m, mps)
#
# The file is “two-line per timestamp” format:
#   t - "m":   <value>
#   t - "mps": <value>
#
# We only commit a sample when BOTH lines for the SAME timestamp are present.
# This prevents position/speed arrays from getting out of sync.
# -----------------------------------------------------------------------------

# Regex captures:
#   group(1) = timestamp (e.g., 0.26)
#   key      = either "m" or "mps"
#   group(3) = numeric value
line_re = re.compile(
    r'^\s*([0-9]*\.?[0-9]+)s\s*-\s*"(?P<key>m|mps)"\s*:\s*([-+]?[0-9]*\.?[0-9]+)'
)

t_list, m_list, mps_list = [], [], []
pending_t = None
pending_m = None
pending_mps = None

with open(INPUT_FILE, "r") as f:
    for line in f:
        match = line_re.match(line)
        if not match:
            continue  # ignore any lines that don't match our expected format

        t = float(match.group(1))
        key = match.group("key")
        val = float(match.group(3))

        # Start / reset a pending pair when timestamp changes
        # We treat a new timestamp as the beginning of a new measurement pair.
        if pending_t is None or abs(t - pending_t) > 1e-12:
            pending_t = t
            pending_m = None
            pending_mps = None

        # Store whichever value this line represents
        if key == "m":
            pending_m = val
        else:  # key == "mps"
            pending_mps = val

        # Commit only when both m AND mps exist for the SAME timestamp
        if pending_m is not None and pending_mps is not None:
            t_list.append(pending_t)
            m_list.append(pending_m)
            mps_list.append(pending_mps)

            # Reset pending state so we don't accidentally mix with the next timestamp
            pending_t = None
            pending_m = None
            pending_mps = None

# Convert lists to arrays
time = np.array(t_list, dtype=float)
position = np.array(m_list, dtype=float)
mps_from_file = np.array(mps_list, dtype=float)

# Basic sanity check: you need enough samples to compute dt and run KF
if len(time) < 3:
    raise ValueError("Not enough samples parsed. Check your MateoV5.txt formatting.")

# Sort just in case the file is out of order (should already be sorted)
order = np.argsort(time)
time = time[order]
position = position[order]
mps_from_file = mps_from_file[order]

# -----------------------------------------------------------------------------
# Finite-difference speed from position
#
# This is used for:
#   - plotting speed (because the exporter may write mps=0 intentionally)
#   - initialization if the exported mps array is all zeros
#
# fd_speed[k] ≈ (position[k] - position[k-1]) / (time[k] - time[k-1])
# -----------------------------------------------------------------------------
fd_speed = np.zeros_like(position)
fd_speed[1:] = np.diff(position) / np.diff(time)
fd_speed[0] = fd_speed[1]  # copy first valid estimate

# If MATLAB exported mps=0 everywhere, using it as a "measurement" is meaningless.
# In that case we use finite-difference speed for initialization + plotting.
if np.allclose(mps_from_file, 0.0, atol=1e-9):
    init_speed = float(fd_speed[0])
    speed_for_plot = fd_speed
    exported_speed_label = "Finite-Difference Speed (from position)"
else:
    init_speed = float(mps_from_file[0])
    speed_for_plot = mps_from_file
    exported_speed_label = 'Exported Speed from file ("mps")'

# -----------------------------------------------------------------------------
# Noise settings (tune as needed)
#
# sigma_meas_pos:
#   Standard deviation of position measurement noise added in this script.
#   This is optional “demo noise” — your MATLAB export may already include
#   measurement noise. If you want to see pure KF smoothing without adding
#   extra noise here, set sigma_meas_pos = 0.
# -----------------------------------------------------------------------------
sigma_meas_pos = 0.1  # meters (demo noise)
R = np.array([[sigma_meas_pos**2]], dtype=float)  # measurement noise covariance

# Create noisy measurement z_pos from exported position
z_pos = position + np.random.normal(0, sigma_meas_pos, size=len(position))

# -----------------------------------------------------------------------------
# Kalman Filter setup
#
# State:
#   x = [position; speed]
#
# Measurement model:
#   z = H x + v
#   where H = [1 0] -> position-only measurement
#
# Process model:
#   x(k+1) = A x(k) + w
#   where A depends on dt for constant-velocity motion:
#       [1 dt]
#       [0  1]
#
# Process noise Q:
#   We assume a constant-acceleration driving noise model.
#   q controls how much random acceleration we allow (bigger q = more agility).
# -----------------------------------------------------------------------------
x_est = np.array([z_pos[0], init_speed], dtype=float)

# Initial covariance:
#   position uncertainty relatively small; speed uncertainty larger
P = np.array([[10.0, 0.0],
              [0.0, 100.0]], dtype=float)

# Position-only measurement matrix
H = np.array([[1.0, 0.0]], dtype=float)

# Process noise scale (tune)
q = 0.1

# Storage for estimated states over time
x_est_hist = np.zeros((2, len(time)), dtype=float)

# -----------------------------------------------------------------------------
# Kalman Filter loop with VARIABLE dt
#
# This is the main “Fix A” change: dt is computed from timestamps each step.
#
# Steps:
#   1) Compute dt from time[k] - time[k-1]
#   2) Build A(dt) and Q(dt)
#   3) Predict: x_pred, P_pred
#   4) Update using position measurement z_pos[k]
# -----------------------------------------------------------------------------
for k in range(len(time)):

    # Compute dt from timestamps
    if k == 0:
        dt = time[1] - time[0]  # best guess for the first step
    else:
        dt = time[k] - time[k - 1]
        # Guard against bad timestamps (duplicate times or NaNs)
        if not (np.isfinite(dt) and dt > 0):
            dt = time[1] - time[0]

    # Constant-velocity state transition matrix
    A = np.array([[1.0, dt],
                  [0.0, 1.0]], dtype=float)

    # Continuous white-noise acceleration discretized to Q(dt)
    # Standard form for CV model:
    #   Q = q * [[dt^4/4, dt^3/2],
    #            [dt^3/2, dt^2  ]]
    Q = q * np.array([[0.25 * dt**4, 0.5 * dt**3],
                      [0.5 * dt**3,  dt**2]], dtype=float)

    # ---- Predict ----
    x_pred = A @ x_est
    P_pred = A @ P @ A.T + Q

    # ---- Update (position-only) ----
    # Innovation covariance
    S = H @ P_pred @ H.T + R

    # Kalman gain
    K = P_pred @ H.T @ np.linalg.inv(S)

    # Innovation / residual: measured - predicted
    innovation = np.array([[z_pos[k]]]) - (H @ x_pred).reshape(1, 1)

    # State update
    x_est = x_pred + (K @ innovation).flatten()

    # Covariance update
    P = (np.eye(2) - K @ H) @ P_pred

    # Store estimate
    x_est_hist[:, k] = x_est

# -----------------------------------------------------------------------------
# Plotting
#
# Plot 1: Position time series
#   - Exported position (treated as “truth-ish” for visualization)
#   - Noisy measurement (z_pos)
#   - KF estimate
#
# Plot 2: Speed time series
#   - If exported mps is all zeros, plot finite-difference speed instead
#   - Always plot KF estimated speed
# -----------------------------------------------------------------------------
plt.figure(figsize=(12, 6))

# Position plot
plt.subplot(2, 1, 1)
plt.plot(time, position, linewidth=2, label="Exported Position (m)")
plt.plot(time, z_pos, ".", markersize=4, label="Noisy Meas Position")
plt.plot(time, x_est_hist[0, :], linewidth=2, label="KF Estimated Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Kalman Filter Position Estimation (timestamp-based dt)")
plt.legend()
plt.grid(True)

# Speed plot
plt.subplot(2, 1, 2)
plt.plot(time, speed_for_plot, linewidth=2, label=exported_speed_label)
plt.plot(time, x_est_hist[1, :], linewidth=2, label="KF Estimated Speed")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Kalman Filter Speed Estimation (timestamp-based dt)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
