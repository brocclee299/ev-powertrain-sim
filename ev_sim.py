"""
EV powertrain simulation - Challenge 3.

Simulates an electric vehicle accelerating from rest at full throttle and
reports how its speed changes over time, following "Approach 2: Dynamic
Calculations" from the reference blog:
https://technoo7blogs.blogspot.com/2022/01/electric-vehicle-powertrain.html

Rather than solving for the answer algebraically, it steps through time in
0.01 s increments. At each step it works out the force the motor can make at
the current speed, subtracts the resistances, and uses the leftover force to
update the speed.

Usage:
    python ev_sim.py          # uses GEAR_RATIO below (9)
    python ev_sim.py 8.5      # try a different ratio without editing the file
"""

import math
import sys

import matplotlib
matplotlib.use("Agg")  # save PNGs directly, no interactive window
import matplotlib.pyplot as plt


# --- Vehicle parameters ---
MASS = 2000.0                    # kg
ROLLING_COEFF = 0.01             # rolling resistance coefficient
DRAG_COEFF = 0.30                # aerodynamic drag coefficient
FRONTAL_AREA = 2.5               # m^2
AIR_DENSITY = 1.2                # kg/m^3
GRAVITY = 10.0                   # m/s^2 (the brief says use 10, not 9.81)
TIRE_RADIUS = 0.4                # m
TIRE_DIAMETER = 2 * TIRE_RADIUS  # m - the speed formula needs diameter
DRIVETRAIN_EFFICIENCY = 0.90     # 90% of motor torque reaches the road

# --- Motor torque curve ---
PEAK_TORQUE = 250.0              # N*m, available below base speed
BASE_SPEED_RPM = 4000.0          # rpm, where constant power takes over
MAX_SPEED_RPM = 10000.0          # rpm, redline

# --- Simulation settings ---
GEAR_RATIO = 9.0                 # <<< the main input, change me
TIME_STEP = 0.01                 # s
END_TIME = 120.0                 # s
TARGET_SPEED = 20.0              # m/s, the speed we time the run to

COMPARISON_RATIOS = [7.0, 9.0, 10.5]
HILL_GRADE = 0.05                # bonus: a 5% slope (rise/run)


def motor_rpm_from_speed(speed, gear_ratio):
    """Vehicle speed (m/s) -> motor speed (rpm)."""
    # One tire revolution moves the car forward one circumference, which the
    # blog writes as V = pi*D*N/60. Rearranged for N, then geared up.
    wheel_rpm = (60.0 * speed) / (math.pi * TIRE_DIAMETER)
    return wheel_rpm * gear_ratio


def motor_torque(motor_rpm):
    """Available motor torque (N*m) at a given motor speed."""
    if motor_rpm <= BASE_SPEED_RPM:
        # Constant torque region - full shove off the line.
        return PEAK_TORQUE
    elif motor_rpm <= MAX_SPEED_RPM:
        # Constant power region - torque is traded away for revs so that
        # torque * speed stays fixed at about 104.7 kW.
        return PEAK_TORQUE * BASE_SPEED_RPM / motor_rpm
    else:
        # Past the redline the motor can't spin any faster.
        return 0.0


def tractive_force(speed, gear_ratio):
    """Forward force (N) the tires push against the road."""
    # The gear ratio is used twice: here it multiplies torque, and inside
    # motor_rpm_from_speed it multiplies rpm. That double use is the whole
    # gear ratio tradeoff - more force, but the redline arrives sooner.
    rpm = motor_rpm_from_speed(speed, gear_ratio)
    wheel_torque = motor_torque(rpm) * gear_ratio * DRIVETRAIN_EFFICIENCY
    return wheel_torque / TIRE_RADIUS


def resistance_force(speed, grade=0.0):
    """Total force (N) resisting motion at a given speed.

    `grade` is a slope as rise/run, so 0.05 is a 5% hill.
    """
    f_rolling = ROLLING_COEFF * MASS * GRAVITY
    f_drag = 0.5 * AIR_DENSITY * DRAG_COEFF * FRONTAL_AREA * speed ** 2
    f_gradient = MASS * GRAVITY * math.sin(math.atan(grade))
    return f_rolling + f_drag + f_gradient


def simulate(gear_ratio, grade=0.0):
    """Run the acceleration simulation, returning (times, speeds) lists."""
    times = [0.0]
    speeds = [0.0]  # the car starts at rest

    speed = 0.0
    for step in range(int(round(END_TIME / TIME_STEP))):
        net_force = tractive_force(speed, gear_ratio) - resistance_force(speed, grade)
        acceleration = net_force / MASS
        speed = speed + acceleration * TIME_STEP

        # A stopped car shouldn't get dragged backwards by its own rolling
        # resistance. Never triggers at these gear ratios, but keeps the model
        # sensible for a very small ratio or a steep hill.
        if speed < 0.0:
            speed = 0.0

        times.append((step + 1) * TIME_STEP)
        speeds.append(speed)

    return times, speeds


def time_to_reach(times, speeds, target):
    """First time (s) the car reaches `target` speed, or None if it never does."""
    for time, speed in zip(times, speeds):
        if speed >= target:
            return time
    return None


def speed_at_time(times, speeds, when):
    """Speed (m/s) at a given time - used for the 1 second checkpoint."""
    return speeds[int(round(when / TIME_STEP))]


def label(gear_ratio):
    """Tidy a ratio for filenames and captions: 9.0 -> '9', 10.5 -> '10.5'."""
    return f"{gear_ratio:g}"


def gear_ratio_from_command_line():
    """Gear ratio typed on the command line, falling back to GEAR_RATIO."""
    # sys.argv holds the words typed in the terminal, with the script name at
    # position 0, so a gear ratio would be at position 1.
    if len(sys.argv) < 2:
        return GEAR_RATIO

    try:
        ratio = float(sys.argv[1])
    except ValueError:
        print(f"'{sys.argv[1]}' is not a number - using {label(GEAR_RATIO)} instead.\n")
        return GEAR_RATIO

    if ratio <= 0:
        print(f"A gear ratio must be positive - using {label(GEAR_RATIO)} instead.\n")
        return GEAR_RATIO

    return ratio


# The two functions below work out the top speed with algebra instead of
# simulation, so the simulated result can be checked against something it
# didn't produce itself.

def rpm_limited_top_speed(gear_ratio):
    """Top speed (m/s) if the motor simply runs out of revs at the redline."""
    wheel_rpm = MAX_SPEED_RPM / gear_ratio
    return wheel_rpm * math.pi * TIRE_DIAMETER / 60.0


def drag_limited_top_speed(grade=0.0):
    """Top speed (m/s) where the available power is entirely absorbed by drag.

    Doesn't depend on gear ratio, since the motor delivers the same power in
    the constant power region whatever the ratio.
    """
    motor_power = PEAK_TORQUE * BASE_SPEED_RPM * 2 * math.pi / 60.0
    wheel_power = motor_power * DRIVETRAIN_EFFICIENCY

    # "power = force * speed" is a cubic in speed, so solve it by bisection:
    # repeatedly halve the range, keeping whichever half contains the answer.
    low, high = 0.0, 200.0
    for _ in range(200):
        mid = (low + high) / 2.0
        if resistance_force(mid, grade) * mid < wheel_power:
            low = mid    # still achievable, look higher
        else:
            high = mid   # resistance already wins, look lower
    return (low + high) / 2.0


def plot_runs(runs, title, filename):
    """Draw speed-vs-time curves and save a PNG. `runs` is (label, times, speeds)."""
    plt.figure(figsize=(9, 5.5))

    for name, times, speeds in runs:
        plt.plot(times, speeds, linewidth=2, label=name)

    # Reference line so the time to 20 m/s can be read off the plot.
    plt.axhline(TARGET_SPEED, color="grey", linestyle="--", linewidth=1)
    plt.text(END_TIME * 0.99, TARGET_SPEED + 0.8, f"{TARGET_SPEED:g} m/s target",
             ha="right", va="bottom", color="grey", fontsize=9)

    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.xlim(0, END_TIME)
    plt.ylim(bottom=0)
    plt.grid(True, alpha=0.3)
    plt.legend(loc="lower right")
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()

    print(f"  saved plot: {filename}")


def main():
    # --- Checkpoint from the brief: ratio 9 should give about 2.4 m/s at 1 s.
    # Asserted rather than just printed, so a broken force chain fails loudly.
    check_times, check_speeds = simulate(9.0)
    checkpoint = speed_at_time(check_times, check_speeds, 1.0)

    print("=" * 62)
    print("CHECKPOINT (gear ratio 9, speed after 1 second)")
    print("=" * 62)
    print(f"  simulated : {checkpoint:.3f} m/s")
    print(f"  expected  : about 2.4 m/s")
    assert 2.3 <= checkpoint <= 2.5, f"Checkpoint failed: got {checkpoint:.3f} m/s"
    print("  PASS\n")

    # --- The selected ratio, and the two figures the challenge asks for.
    selected_ratio = gear_ratio_from_command_line()
    sel_times, sel_speeds = simulate(selected_ratio)
    sel_reached = time_to_reach(sel_times, sel_speeds, TARGET_SPEED)

    print("=" * 62)
    print(f"SELECTED GEAR RATIO: {label(selected_ratio)}")
    print("=" * 62)
    print(f"  time to {TARGET_SPEED:g} m/s : "
          f"{f'{sel_reached:.2f} s' if sel_reached is not None else 'never reached'}")
    print(f"  top speed      : {max(sel_speeds):.2f} m/s")
    print()

    # --- Tasks 1 and 2: the three ratios on flat ground.
    print("=" * 62)
    print("RESULTS - FLAT GROUND")
    print("=" * 62)
    print(f"{'Gear ratio':>11} | {'Time to 20 m/s':>15} | {'Top speed':>11} | {'Limited by':>12}")
    print("-" * 62)

    flat_runs = []
    for ratio in COMPARISON_RATIOS:
        times, speeds = simulate(ratio)
        flat_runs.append((f"Gear ratio {label(ratio)}", times, speeds))

        reached = time_to_reach(times, speeds, TARGET_SPEED)
        reached_text = f"{reached:.2f} s" if reached is not None else "never"

        # A ratio is capped either by the redline or by drag. Whichever
        # ceiling is lower is the one that actually binds.
        limiter = ("motor rpm" if rpm_limited_top_speed(ratio) < drag_limited_top_speed()
                   else "drag/power")

        print(f"{label(ratio):>11} | {reached_text:>15} | {max(speeds):>9.2f} m/s | {limiter:>12}")

    print("-" * 62)
    print("Cross-check of the top speeds (calculated, not simulated):")
    for ratio in COMPARISON_RATIOS:
        print(f"  ratio {label(ratio):>4}: rpm ceiling {rpm_limited_top_speed(ratio):6.2f} m/s"
              f" | drag ceiling {drag_limited_top_speed():6.2f} m/s")
    print()

    # --- Plots.
    print("=" * 62)
    print("PLOTS")
    print("=" * 62)

    plot_runs(
        [(f"Gear ratio {label(selected_ratio)}", sel_times, sel_speeds)],
        f"EV acceleration from rest - gear ratio {label(selected_ratio)}",
        f"speed_vs_time_gr{label(selected_ratio)}.png",
    )

    plot_runs(
        flat_runs,
        "EV acceleration - gear ratio comparison (flat ground)",
        "speed_vs_time_all_ratios.png",
    )

    # --- Bonus: the same launch up a 5% hill.
    print()
    print("=" * 62)
    print(f"BONUS - {HILL_GRADE * 100:g}% HILL")
    print("=" * 62)
    print(f"  extra resisting force: "
          f"{MASS * GRAVITY * math.sin(math.atan(HILL_GRADE)):.1f} N")
    print(f"{'Gear ratio':>11} | {'Time to 20 m/s':>15} | {'Top speed':>11}")
    print("-" * 44)

    for ratio in COMPARISON_RATIOS:
        times, speeds = simulate(ratio, grade=HILL_GRADE)
        reached = time_to_reach(times, speeds, TARGET_SPEED)
        reached_text = f"{reached:.2f} s" if reached is not None else "never"
        print(f"{label(ratio):>11} | {reached_text:>15} | {max(speeds):>9.2f} m/s")

    print("-" * 44)
    print()

    hill_times, hill_speeds = simulate(selected_ratio, grade=HILL_GRADE)
    plot_runs(
        [
            (f"Flat ground (ratio {label(selected_ratio)})", sel_times, sel_speeds),
            (f"{HILL_GRADE * 100:g}% hill (ratio {label(selected_ratio)})", hill_times, hill_speeds),
        ],
        f"Effect of a {HILL_GRADE * 100:g}% gradient - gear ratio {label(selected_ratio)}",
        f"speed_vs_time_hill_5pct_gr{label(selected_ratio)}.png",
    )


if __name__ == "__main__":
    main()
