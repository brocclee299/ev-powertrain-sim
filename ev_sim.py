"""
Electric Vehicle Powertrain Simulation
=======================================
Wisconsin Autonomous - Challenge 3: Programming

Simulates an EV accelerating from a standstill at full throttle and reports
how its speed changes over time.

This implements "Approach 2: Dynamic Calculations" from the reference blog:
    https://technoo7blogs.blogspot.com/2022/01/electric-vehicle-powertrain.html

The idea behind Approach 2 is a feedback loop. Instead of solving for the
answer in one algebraic step (that is Approach 1, which has to assume a
constant acceleration), we step forward through time in small increments.
At each increment we ask "given how fast the car is moving RIGHT NOW, what
force can the motor make, and what forces are pushing back?" The difference
between those two is the net force, which gives acceleration, which nudges
the speed for the next increment. Repeat 12,000 times and the speed curve
draws itself.

The loop, matching the blog's block diagram:

    speed  ->  wheel rpm  ->  motor rpm  ->  motor torque  (torque curve)
      ^                                           |
      |                                           v
      +---  new speed  <---  acceleration  <---  wheel force

Only two imports are needed:
    math               - for pi, sin and atan. The loop works on one number
                         at a time (a scalar), so numpy would add nothing.
    matplotlib.pyplot  - to draw the speed-vs-time plots.
"""

import math

import matplotlib
matplotlib.use("Agg")  # render straight to PNG files; no interactive window needed
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# VEHICLE PARAMETERS
# Every number from the challenge brief is a named constant with its unit, so
# there are no unexplained magic numbers buried in the formulas below.
# ---------------------------------------------------------------------------
MASS = 2000.0                  # kg     - vehicle mass
ROLLING_COEFF = 0.01           # -      - rolling resistance coefficient
DRAG_COEFF = 0.30              # -      - aerodynamic drag coefficient
FRONTAL_AREA = 2.5             # m^2    - frontal area
AIR_DENSITY = 1.2              # kg/m^3 - density of air
GRAVITY = 10.0                 # m/s^2  - the brief says to use 10, not 9.81
TIRE_RADIUS = 0.4              # m      - tire radius
TIRE_DIAMETER = 2 * TIRE_RADIUS  # m    - the blog's speed formula uses DIAMETER
DRIVETRAIN_EFFICIENCY = 0.90   # -      - 90% of motor torque reaches the road

# ---------------------------------------------------------------------------
# MOTOR PARAMETERS - defines the torque-vs-speed characteristic curve
# ---------------------------------------------------------------------------
PEAK_TORQUE = 250.0            # N*m    - flat torque below base speed
BASE_SPEED_RPM = 4000.0        # rpm    - where constant power takes over
MAX_SPEED_RPM = 10000.0        # rpm    - redline; no torque beyond this

# ---------------------------------------------------------------------------
# SIMULATION SETTINGS
#
# GEAR_RATIO is the headline input for the challenge. Change this one number
# to re-run the whole simulation at a different ratio.
# ---------------------------------------------------------------------------
GEAR_RATIO = 9.0               # -      <<< THE MAIN INPUT: change me
TIME_STEP = 0.01               # s      - dt, the size of one increment
END_TIME = 120.0               # s      - how long to simulate
TARGET_SPEED = 20.0            # m/s    - the speed we time the run to

COMPARISON_RATIOS = [7.0, 9.0, 10.5]  # the three ratios the tasks ask for
HILL_GRADE = 0.05              # -      - bonus task: a 5% slope (rise/run)


# ---------------------------------------------------------------------------
# STEP 1 & 2: vehicle speed -> wheel speed -> motor speed
# ---------------------------------------------------------------------------
def motor_rpm_from_speed(speed, gear_ratio):
    """Convert vehicle speed (m/s) to motor speed (rpm).

    The wheel rolls without slipping, so one revolution of the tire moves the
    car forward by one tire circumference (pi * D metres). The blog writes
    this as V = pi*D*N/60, rearranged to N = 60*V/(pi*D).

    The gearbox then spins the motor faster than the wheel by the gear ratio.
    """
    wheel_rpm = (60.0 * speed) / (math.pi * TIRE_DIAMETER)
    return wheel_rpm * gear_ratio


# ---------------------------------------------------------------------------
# STEP 3: look up motor torque at that motor speed
# ---------------------------------------------------------------------------
def motor_torque(motor_rpm):
    """Return available motor torque (N*m) at a given motor speed (rpm).

    Three regions, which is how nearly every real EV motor behaves:

      1. Below 4,000 rpm - CONSTANT TORQUE. The motor makes its full 250 N*m.
         Power is still climbing here (power = torque * speed), so this is the
         region that gives an EV its instant off-the-line shove.

      2. 4,000 to 10,000 rpm - CONSTANT POWER. The motor has hit its power
         ceiling, so to spin faster it must give up torque proportionally.
         250 * 4000 / rpm keeps the product (and therefore power) fixed at
         about 104.7 kW.

      3. Above 10,000 rpm - REDLINE. Nothing left; the motor cannot spin
         faster, so it produces no torque at all.
    """
    if motor_rpm <= BASE_SPEED_RPM:
        return PEAK_TORQUE
    elif motor_rpm <= MAX_SPEED_RPM:
        return PEAK_TORQUE * BASE_SPEED_RPM / motor_rpm
    else:
        return 0.0


# ---------------------------------------------------------------------------
# STEP 4: motor torque -> forward force at the tires
# ---------------------------------------------------------------------------
def tractive_force(speed, gear_ratio):
    """Return the forward force (N) the tires push against the road.

    Two conversions:
      wheel torque   = motor torque * gear ratio * drivetrain efficiency
      forward force  = wheel torque / tire radius

    Note the gear ratio appears here multiplying TORQUE, and it also appeared
    in motor_rpm_from_speed() multiplying SPEED. That double appearance is the
    entire gear-ratio tradeoff in one sentence: a bigger ratio buys more force
    but burns through the motor's rpm range sooner.
    """
    rpm = motor_rpm_from_speed(speed, gear_ratio)
    wheel_torque = motor_torque(rpm) * gear_ratio * DRIVETRAIN_EFFICIENCY
    return wheel_torque / TIRE_RADIUS


# ---------------------------------------------------------------------------
# STEP 5: the forces resisting motion
# ---------------------------------------------------------------------------
def resistance_force(speed, grade=0.0):
    """Return total resisting force (N) at a given speed.

    Rolling resistance - tire deformation. Constant, independent of speed.
    Aerodynamic drag   - grows with the SQUARE of speed, so it is nothing at
                         a standstill but dominates at motorway speeds.
    Gradient (bonus)   - the component of the car's weight pulling it back
                         down a slope. Zero on flat ground.

    `grade` is a slope expressed as rise/run, so 0.05 is a 5% hill. The blog's
    formula wants an angle, hence atan() to convert.
    """
    f_rolling = ROLLING_COEFF * MASS * GRAVITY
    f_drag = 0.5 * AIR_DENSITY * DRAG_COEFF * FRONTAL_AREA * speed ** 2

    slope_angle = math.atan(grade)
    f_gradient = MASS * GRAVITY * math.sin(slope_angle)

    return f_rolling + f_drag + f_gradient


# ---------------------------------------------------------------------------
# THE SIMULATION LOOP
# ---------------------------------------------------------------------------
def simulate(gear_ratio, grade=0.0):
    """Run the acceleration simulation. Returns (times, speeds) lists.

    This is forward Euler integration. Over a short enough slice of time,
    acceleration barely changes, so we can pretend it is constant across the
    slice and use the simplest possible update:

        new speed = old speed + acceleration * dt

    dt = 0.01 s is small enough that the error is negligible here. If the step
    were far too large the curve would overshoot and visibly wobble around the
    top speed instead of settling smoothly onto it.
    """
    times = [0.0]
    speeds = [0.0]  # "The car starts at rest."

    speed = 0.0
    number_of_steps = int(round(END_TIME / TIME_STEP))

    for step in range(number_of_steps):
        # Net force = what the motor pushes with, minus what pushes back.
        net_force = tractive_force(speed, gear_ratio) - resistance_force(speed, grade)

        acceleration = net_force / MASS          # Newton's second law, a = F/m
        speed = speed + acceleration * TIME_STEP  # the Euler update

        # Safety guard: a stopped car should not be dragged backwards by its
        # own rolling resistance. It never triggers for the ratios used here
        # (at rest the motor makes far more force than the resistances), but
        # it keeps the model physically sensible if someone tries a tiny ratio
        # or a very steep hill.
        if speed < 0.0:
            speed = 0.0

        times.append((step + 1) * TIME_STEP)
        speeds.append(speed)

    return times, speeds


# ---------------------------------------------------------------------------
# READING RESULTS OUT OF THE SIMULATION
# ---------------------------------------------------------------------------
def time_to_reach(times, speeds, target):
    """First time (s) the car reaches `target` speed, or None if it never does."""
    for time, speed in zip(times, speeds):
        if speed >= target:
            return time
    return None


def speed_at_time(times, speeds, when):
    """Speed (m/s) at a particular time - used for the 1-second checkpoint."""
    index = int(round(when / TIME_STEP))
    return speeds[index]


def label(gear_ratio):
    """Format a ratio for filenames and captions: 9.0 -> '9', 10.5 -> '10.5'."""
    return f"{gear_ratio:g}"


# ---------------------------------------------------------------------------
# INDEPENDENT CROSS-CHECKS
# These recompute the expected top speed with algebra rather than simulation,
# so the loop can be checked against something it did not produce itself.
# ---------------------------------------------------------------------------
def rpm_limited_top_speed(gear_ratio):
    """Top speed (m/s) if the motor simply runs out of revs at the redline."""
    wheel_rpm = MAX_SPEED_RPM / gear_ratio
    return wheel_rpm * math.pi * TIRE_DIAMETER / 60.0


def drag_limited_top_speed(grade=0.0):
    """Top speed (m/s) where wheel power exactly equals the resisting power.

    Independent of gear ratio, because in the constant-power region the motor
    delivers the same power whatever the ratio. Solved by bisection because
    the balance 'power = force * speed' is a cubic in speed.
    """
    motor_power = PEAK_TORQUE * BASE_SPEED_RPM * 2 * math.pi / 60.0
    wheel_power = motor_power * DRIVETRAIN_EFFICIENCY

    low, high = 0.0, 200.0
    for _ in range(200):  # bisection: 200 halvings is far more than enough
        mid = (low + high) / 2.0
        if resistance_force(mid, grade) * mid < wheel_power:
            low = mid   # this speed is still achievable, look higher
        else:
            high = mid  # resistance already wins here, look lower
    return (low + high) / 2.0


# ---------------------------------------------------------------------------
# PLOTTING
# ---------------------------------------------------------------------------
def plot_runs(runs, title, filename):
    """Draw one or more speed-vs-time curves and save to a PNG.

    `runs` is a list of (label, times, speeds) tuples.
    """
    plt.figure(figsize=(9, 5.5))

    for name, times, speeds in runs:
        plt.plot(times, speeds, linewidth=2, label=name)

    # Reference line so the "time to 20 m/s" is readable straight off the plot.
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


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def main():
    # -- The checkpoint from the brief -------------------------------------
    # "With a gear ratio of 9, the vehicle should be traveling about 2.4 m/s
    # after 1 second." If this fails, something in the force chain is wrong,
    # so it is worth asserting rather than just printing.
    times, speeds = simulate(9.0)
    checkpoint = speed_at_time(times, speeds, 1.0)

    print("=" * 62)
    print("CHECKPOINT (gear ratio 9, speed after 1 second)")
    print("=" * 62)
    print(f"  simulated : {checkpoint:.3f} m/s")
    print(f"  expected  : about 2.4 m/s")
    assert 2.3 <= checkpoint <= 2.5, f"Checkpoint failed: got {checkpoint:.3f} m/s"
    print("  PASS\n")

    # -- Task 1 and 2: the three gear ratios on flat ground ----------------
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
        top_speed = max(speeds)

        # Which ceiling did it actually hit? Whichever is lower is the binding one.
        rpm_limit = rpm_limited_top_speed(ratio)
        drag_limit = drag_limited_top_speed()
        limiter = "motor rpm" if rpm_limit < drag_limit else "drag/power"

        reached_text = f"{reached:.2f} s" if reached is not None else "never"
        print(f"{label(ratio):>11} | {reached_text:>15} | {top_speed:>9.2f} m/s | {limiter:>12}")

    print("-" * 62)
    print("Cross-check of the top speeds (calculated, not simulated):")
    for ratio in COMPARISON_RATIOS:
        print(f"  ratio {label(ratio):>4}: rpm ceiling {rpm_limited_top_speed(ratio):6.2f} m/s"
              f" | drag ceiling {drag_limited_top_speed():6.2f} m/s")
    print()

    # -- Plots -------------------------------------------------------------
    print("=" * 62)
    print("PLOTS")
    print("=" * 62)

    # Task 1: the headline run at whatever GEAR_RATIO is set to at the top.
    times, speeds = simulate(GEAR_RATIO)
    plot_runs(
        [(f"Gear ratio {label(GEAR_RATIO)}", times, speeds)],
        f"EV acceleration from rest - gear ratio {label(GEAR_RATIO)}",
        f"speed_vs_time_gr{label(GEAR_RATIO)}.png",
    )

    # Task 2: all three ratios overlaid so the tradeoff is visible at a glance.
    plot_runs(
        flat_runs,
        "EV acceleration - gear ratio comparison (flat ground)",
        "speed_vs_time_all_ratios.png",
    )

    # -- Bonus: the same launch up a 5% hill -------------------------------
    print()
    print("=" * 62)
    print(f"BONUS - {HILL_GRADE * 100:g}% HILL")
    print("=" * 62)
    print(f"  extra resisting force: "
          f"{MASS * GRAVITY * math.sin(math.atan(HILL_GRADE)):.1f} N")
    print(f"{'Gear ratio':>11} | {'Time to 20 m/s':>15} | {'Top speed':>11}")
    print("-" * 44)

    hill_runs = []
    for ratio in COMPARISON_RATIOS:
        times, speeds = simulate(ratio, grade=HILL_GRADE)
        hill_runs.append((f"Gear ratio {label(ratio)} (hill)", times, speeds))

        reached = time_to_reach(times, speeds, TARGET_SPEED)
        reached_text = f"{reached:.2f} s" if reached is not None else "never"
        print(f"{label(ratio):>11} | {reached_text:>15} | {max(speeds):>9.2f} m/s")

    print("-" * 44)
    print()

    # Flat vs hill at the headline ratio, to show the penalty directly.
    flat_times, flat_speeds = simulate(GEAR_RATIO)
    hill_times, hill_speeds = simulate(GEAR_RATIO, grade=HILL_GRADE)
    plot_runs(
        [
            (f"Flat ground (ratio {label(GEAR_RATIO)})", flat_times, flat_speeds),
            (f"{HILL_GRADE * 100:g}% hill (ratio {label(GEAR_RATIO)})", hill_times, hill_speeds),
        ],
        f"Effect of a {HILL_GRADE * 100:g}% gradient - gear ratio {label(GEAR_RATIO)}",
        "speed_vs_time_hill_5pct.png",
    )


if __name__ == "__main__":
    main()
