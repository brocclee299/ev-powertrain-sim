# EV Powertrain Simulation

A simulation of an electric vehicle accelerating from a standstill at full throttle, written for
**Challenge 3: Programming**.

It implements "Approach 2: Dynamic Calculations" from the reference blog,
[*Electric Vehicle Powertrain*](https://technoo7blogs.blogspot.com/2022/01/electric-vehicle-powertrain.html).

**➡️ See [`report.md`](report.md) for the results, plots and answers to the tasks.**

---

## Running it

Requires Python 3 and matplotlib.

```bash
pip install -r requirements.txt
python ev_sim.py
```

This prints the results to the console and writes three plots:

| File | What it shows |
|---|---|
| `speed_vs_time_gr9.png` | Speed vs time at a gear ratio of 9 (Task 1) |
| `speed_vs_time_all_ratios.png` | Gear ratios 7, 9 and 10.5 overlaid (Task 2) |
| `speed_vs_time_hill_5pct.png` | Flat ground vs a 5% hill (bonus) |

## Changing the gear ratio

The gear ratio is a named constant near the top of [`ev_sim.py`](ev_sim.py):

```python
GEAR_RATIO = 9.0     # <<< THE MAIN INPUT: change me
```

Edit that one number and re-run. The plot filename follows the ratio automatically.
The list of ratios used for the comparison table is right below it:

```python
COMPARISON_RATIOS = [7.0, 9.0, 10.5]
```

---

## How it works

The program does not try to solve for the answer algebraically. Instead it steps forward through
time in increments of 0.01 s, and at each step asks: *given how fast the car is moving right now,
what force can the motor make, and what is pushing back?*

Each step runs this loop:

```
  vehicle speed
       │
       ▼   N_wheel = 60·V / (π·D)          wheel rpm from road speed
  wheel rpm
       │
       ▼   N_motor = N_wheel × gear ratio  the gearbox spins the motor faster
  motor rpm
       │
       ▼   torque curve lookup             250 N·m flat, then constant power, then nothing
  motor torque
       │
       ▼   F = τ × gear ratio × η / r      gearing, losses, then torque → force
  force at the tires
       │
       ▼   a = (F_tractive − F_resist) / m  Newton's second law
  acceleration
       │
       ▼   v_new = v_old + a × dt          and around again with the new speed
  updated speed
```

The resisting forces are rolling resistance (constant), aerodynamic drag (grows with the square of
speed) and, in the bonus section, gradient resistance from the slope.

### The motor torque curve

Available torque depends on how fast the motor is spinning:

| Motor speed | Torque | What it means |
|---|---|---|
| 0 – 4,000 rpm | 250 N·m | Constant torque. Full shove — this is the EV launch feel. |
| 4,000 – 10,000 rpm | 250 × 4000 / rpm | Constant power (~104.7 kW). Torque is traded for revs. |
| above 10,000 rpm | 0 | Redline. The motor cannot spin any faster. |

### Why the gear ratio forces a compromise

The gear ratio shows up **twice** in the loop above — once multiplying torque, and once multiplying
motor rpm. That is the whole tradeoff. Raising it gives more force at the wheels (quicker
acceleration) but also runs the motor into its redline at a lower road speed (lower top speed).
You cannot have both from a single fixed ratio.

---

## Files

| File | Purpose |
|---|---|
| `ev_sim.py` | The simulation. The only code file. |
| `report.md` | Results, plots, and written answers to the tasks. |
| `requirements.txt` | Python dependencies. |
| `*.png` | Generated plots. |

## Verifying it is correct

The brief gives a checkpoint: at a gear ratio of 9 the car should be doing about 2.4 m/s after
1 second. The simulation gives **2.431 m/s**, and this is asserted in `main()` so the program fails
loudly if a change ever breaks it.

As a second, independent check, `main()` also calculates each gear ratio's top speed two ways that
the simulation never uses — the motor's redline ceiling and the point where drag absorbs all
available power — and prints them alongside the simulated result. They agree to within 0.01 m/s.
