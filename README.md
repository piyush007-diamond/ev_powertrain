# EV Powertrain Simulation — MATLAB/Simulink Model

A full-vehicle electric powertrain simulation built in MATLAB/Simulink (`navin_model`). The model covers the complete energy path from a drive-cycle velocity reference through driver intent, motor actuation, transmission, vehicle dynamics, regenerative braking, and battery state estimation. It is designed for rapid prototyping, parameter studies, and energy-consumption analysis of battery electric vehicles (BEVs).

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Structure](#repository-structure)
3. [System Architecture & Signal Flow](#system-architecture--signal-flow)
4. [Subsystem Reference](#subsystem-reference)
   - [Drive Cycle & Top-Level Model](#1-drive-cycle--top-level-model)
   - [Driver Model](#2-driver-model)
   - [Motor Drive](#3-motor-drive)
   - [Transmission](#4-transmission)
   - [Vehicle Dynamics](#5-vehicle-dynamics)
   - [Regen Controller](#6-regen-controller)
   - [Battery Pack](#7-battery-pack)
5. [Key Parameters](#key-parameters)
6. [Logged Outputs](#logged-outputs)
7. [How to Run](#how-to-run)
8. [Dependencies](#dependencies)
9. [Results & Interpretation](#results--interpretation)
10. [Future Work](#future-work)

---

## Project Overview

This simulation models a single-axle rear-drive BEV and answers the following engineering questions:

- How does vehicle speed track a prescribed drive cycle?
- How much electrical power does the motor consume or recover at each instant?
- How does battery State of Charge (SoC) evolve over the cycle?
- How is braking torque split between regenerative and friction braking?
- What are the terminal voltage and current demands placed on the battery pack?

The model is purely longitudinal (1-D) and uses lookup-table-based component maps for efficiency and internal resistance to keep simulation time low while maintaining physical fidelity.

---

## Repository Structure

```
navin_model/
├── navin_model.slx          # Top-level Simulink model
├── init_params.m            # Workspace initialisation script (parameters & maps)
├── README.md                # This file
└── results/                 # Auto-generated figures and .mat result files
```

---

## System Architecture & Signal Flow

The diagram below shows the high-level power and signal flow through the model. Arrows indicate the primary direction of information; double arrows indicate bidirectional energy exchange.

```
Drive Cycle (velocity ref)
        │
        ▼
  Driver Model  ──── Accel_Cmd ────►  Motor Drive
  (PID + logic) ──── Brake_Cmd ────►  Regen Controller
        │                                   │
        │                           T_regen_cmd  F_friction
        │                                   │         │
        ▼                                   ▼         ▼
  Motor Drive  ──── T_actual ────►  Transmission  ◄── (combined)
               ──── P_elec  ────►  Battery Pack
                                        │
                                   SoC, V_term, I_batt
                                        │
        ┌───────────────────────────────┘
        │
        ▼
  Vehicle Dynamics  ──── Velocity ──►  (feedback to Driver Model,
                    ──── Distance       Regen Controller, Motor Drive)
```

**Energy flow:** Chemical energy in the battery pack is converted to electrical power (`P_elec`), then to mechanical torque by the motor, amplified through the transmission gear ratio, and finally applied as tractive force to accelerate the vehicle mass against road-load forces. During braking, kinetic energy flows in reverse: the motor acts as a generator, and `P_elec` becomes negative (charging current), limited by the regen controller and battery SoC.

---

## Subsystem Reference

### 1. Drive Cycle & Top-Level Model

**File:** `navin_model.slx` (top level)

The simulation clock drives a **1-D lookup table** (`Drive_Cycle`) that outputs the reference velocity `Vel_Ref` as a function of simulation time. This replicates standard test cycles (e.g., WLTP, UDDS, or a custom profile) stored as a time–speed vector in the workspace.

**Key signals logged at the top level:**

| Signal | Log Tag | Description |
|--------|---------|-------------|
| Simulation time | `out.sim_time` | Absolute time (s) |
| Reference velocity | `out.vel_ref` | Drive-cycle speed command (m/s) |
| Actual velocity | `out.vel_actual` | Simulated vehicle speed (m/s) |
| Motor torque | `out.t_motor` | Actual motor output torque (N·m) |
| Electrical power | `out.p_elec` | Motor electrical power demand (W) |
| Battery SoC | `out.soc` | Pack state of charge (0–1) |
| Terminal voltage | `out.v_term` | Battery terminal voltage (V) |
| Battery current | `out.i_batt` | Pack current (A, positive = discharge) |
| Distance | `out.distance` | Cumulative vehicle distance (m) |

A global **Goto/From** tag pair (`[Vel]`) propagates the actual vehicle velocity to subsystems that need it (Regen Controller, Transmission) without drawing long signal lines across the diagram.

---

### 2. Driver Model

**Block name:** `Driver_Model`

The driver subsystem closes the outer velocity control loop, translating the tracking error between the reference and actual speed into pedal commands that the rest of the powertrain acts on.

**Inputs:**
- `Vel_Ref` — target speed from the drive cycle (m/s)
- `Vel_Actual` — measured vehicle speed (m/s)

**Outputs:**
- `Accel_Cmd` — normalised accelerator pedal position (0–1)
- `Brake_Cmd` — normalised brake pedal position (0–1)

**Internal logic:**

1. The velocity error `(Vel_Ref − Vel_Actual)` is fed into a **PID controller** (`PID`) followed by a saturation block, producing a signed control signal.
2. A **comparator** (`Is_Pos`) checks whether the signal is positive. If true, `Switch_Accel` passes the signal to `Accel_Cmd`; otherwise `Accel_Cmd` is set to zero.
3. The signal is simultaneously **negated** (`Negate`) and compared against zero (`Is_Neg`). If the negated value is positive (i.e., original signal is negative), `Switch_Brake` routes it to `Brake_Cmd`; otherwise `Brake_Cmd` is zero.

This ensures the accelerator and brake commands are always non-negative and mutually exclusive — mimicking a real driver who cannot press both pedals simultaneously.

---

### 3. Motor Drive

**Block name:** `Motor_Drive`

Models a permanent-magnet synchronous motor (PMSM) or equivalent, capturing torque-limited operation and efficiency-based power conversion.

**Inputs:**
- `T_demand` — torque demand from the summed accelerator and regen torque commands (N·m)
- `Speed_rads` — motor shaft speed (rad/s), derived from vehicle speed via the transmission

**Outputs:**
- `T_actual` — torque delivered after applying limits (N·m)
- `P_elec` — electrical power consumed from (positive) or returned to (negative) the battery (W)

**Internal logic:**

1. `To_RPM` converts shaft speed from rad/s to RPM (gain block).
2. `Max_Torque_Map` is a 1-D lookup table that returns the maximum available torque as a function of motor speed, capturing the constant-torque / constant-power boundary of a real motor.
3. `Torque_Limit` (a saturation block with variable upper and lower limits) clamps `T_demand`. The upper limit comes from `Max_Torque_Map`; the lower limit is the negated maximum (`Neg_Max`), enabling regenerative braking commands.
4. `Calc_P_mech` multiplies the clamped torque by shaft speed to get mechanical power.
5. `Is_Motoring` compares mechanical power against zero. The result selects the efficiency path through `Power_Switch`:
   - **Motoring (P_mech > 0):** Electrical power = P_mech / efficiency (`Div_Eff`).
   - **Generating (P_mech ≤ 0):** Electrical power = P_mech × efficiency (`Mult_Eff`), which is less negative (accounting for generator losses).
6. `Eff_Map` is a **2-D lookup table** (speed × torque → efficiency), providing high-fidelity loss modelling. A fixed scalar `Inv_Eff = 0.90` is used as a fallback or blending factor for `Total_Eff`.

---

### 4. Transmission

**Block name:** `Transmission`

A purely algebraic single-speed gearbox model. No dynamic states — all transformations are instantaneous gain blocks.

**Inputs:**
- `Motor_Torque` — motor shaft torque (N·m)
- `Veh_Speed` — vehicle longitudinal speed (m/s)

**Outputs:**
- `Tractive_Force` — longitudinal force at the wheels (N)
- `Motor_Speed` — motor shaft speed (rad/s)

**Internal logic:**

- `Calc_Force` applies the combined gear-ratio-and-wheel-radius gain:  
  `F_tractive = T_motor × (GearRatio / WheelRadius)`
- `Calc_Motor_Speed` applies the inverse gain to convert vehicle speed to motor speed:  
  `ω_motor = V_vehicle × (GearRatio / WheelRadius)`

Both gains are stored as tunable parameters in `init_params.m`.

---

### 5. Vehicle Dynamics

**Block name:** `Vehicle_Dynamics`

A longitudinal point-mass model. Integrates Newton's second law to produce vehicle velocity and distance.

**Inputs:**
- `F_Tractive` — traction force from the motor via the transmission (N)
- `F_Brake` — friction braking force from the regen controller (N)

**Outputs:**
- `Velocity` — vehicle speed (m/s)
- `Distance` — odometer distance (m)

**Internal logic:**

1. Road-load forces are calculated from velocity feedback:
   - `V_Squared` multiplies velocity by itself.
   - `Aero_Coeff` (gain) scales `V²` by `0.5 × ρ_air × C_d × A_f` to give aerodynamic drag.
   - `Roll_Res` (constant block) provides a constant rolling-resistance force `= m × g × C_rr`.
2. `Sum_Forces` subtracts `F_Brake`, aerodynamic drag, and rolling resistance from `F_Tractive`.
3. `Inv_Mass` (gain = 1/m) converts net force to acceleration.
4. `Integrator_Vel` integrates acceleration to produce velocity.
5. `Integrator_Dist` integrates velocity to produce distance.

---

### 6. Regen Controller

**Block name:** `Regen_Controller`

Determines how much braking torque the motor should absorb regeneratively and how much must be handled by conventional friction brakes.

**Inputs:**
- `Brake_Cmd` — normalised brake pedal demand (0–1)
- `Velocity` — vehicle speed (m/s)
- `SoC` — battery state of charge (0–1)

**Outputs:**
- `T_regen_cmd` — regenerative braking torque command to the motor (N·m, negative convention)
- `F_friction` — friction brake force required to supplement regen (N)

**Internal logic:**

**Step 1 — Compute demanded braking force:**  
`Calc_F_Demand` (gain block) scales `Brake_Cmd` to a total demanded braking force `F_demand`.

**Step 2 — Compute available regen force (`Calc_F_Avail`):**

- `P_regen_max` (constant) sets the peak power the motor/battery can absorb during regen.
- `Min_Vel` (saturation) clamps velocity to a minimum value to avoid division by zero.
- `Div_P_by_V` divides `P_regen_max` by clamped velocity, giving a power-limited maximum regen force.
- `Speed_Factor` (1-D lookup table) returns a speed-dependent blending factor (e.g., regen effectiveness drops at very low speeds).
- `SoC_Factor` (1-D lookup table) returns an SoC-dependent factor (e.g., regen is reduced when the battery is nearly full to avoid overcharging).
- `Calc_F_Avail` multiplies these three contributions together to obtain the net available regenerative force.

**Step 3 — Arbitrate between regen and friction:**

- `Min_Force` takes the minimum of `F_demand` and `F_avail`, ensuring regen never exceeds what is physically demanded or available.
- `Sub_Friction` subtracts the regen portion from total demand to compute the residual `F_friction` that friction brakes must supply.
- `Force_to_Torque` (gain = WheelRadius / GearRatio) converts the regen force to a motor shaft torque command `T_regen_cmd`.

---

### 7. Battery Pack

**Block name:** `Battery_Pack`

An equivalent-circuit model (ECM) representing the battery pack. Uses a first-order RC network (Thevenin model) with SoC-dependent parameters.

**Inputs:**
- `P_elec` — total electrical power demanded from the pack (W); positive = discharge, negative = charge
- `P_aux` — auxiliary electrical load (W, e.g., HVAC, electronics)

**Outputs:**
- `SoC` — state of charge (0–1)
- `V_term` — terminal (load) voltage of the pack (V)
- `I_batt` — pack current (A)

**Internal logic:**

1. `Sum_Power` adds `P_elec` and `P_aux` to obtain total pack power demand.
2. **Parameter maps** (1-D lookup tables keyed on SoC):
   - `V_oc_Map` — open-circuit voltage vs. SoC
   - `R0_Map` — series (ohmic) resistance vs. SoC
   - `R1_Map` — RC polarisation resistance vs. SoC
   - `C1_Map` — RC polarisation capacitance vs. SoC
3. `Mux_Current` bundles the pack power and circuit parameters. `Calc_Current` (`f(u)` block) solves the quadratic equation arising from `P = V_oc × I − I² × R0 − V1 × I` to find battery current `I_batt`.
4. **SoC integration:**  
   `Calc_dSoC` (gain = −1 / (3600 × Q_nom)) scales current to SoC rate of change.  
   `Integrator_SoC` integrates to give SoC, initialised at the starting SoC defined in `init_params.m`.
5. **RC dynamics (polarisation voltage V1):**  
   `Mux_dV1` bundles current and RC parameters. `Calc_dV1` (`f(u)` block) computes `dV1/dt = (I / C1) − (V1 / (R1 × C1))`.  
   `Integrator_V1` integrates to give `V1`.
6. **Terminal voltage:**  
   `I_R0` multiplies current by `R0_Map` to get the ohmic voltage drop.  
   `Calc_V_term` subtracts `I × R0` and `V1` from `V_oc` to produce `V_term`.

---

## Key Parameters

All parameters are defined in `init_params.m` and loaded into the MATLAB workspace before simulation.

| Parameter | Symbol | Typical Value | Unit | Description |
|-----------|--------|--------------|------|-------------|
| Vehicle mass | `m_veh` | 1800 | kg | Kerb mass including driver |
| Drag coefficient | `Cd` | 0.28 | — | Aerodynamic drag coefficient |
| Frontal area | `Af` | 2.4 | m² | Projected frontal area |
| Rolling resistance | `Crr` | 0.012 | — | Tyre rolling resistance coefficient |
| Wheel radius | `r_wheel` | 0.33 | m | Effective rolling radius |
| Gear ratio | `GR` | 9.5 | — | Motor-to-wheel speed ratio |
| Peak motor power | `P_motor_max` | 150 | kW | Continuous peak power |
| Battery capacity | `Q_nom` | 75 | Ah | Nominal cell/pack capacity |
| Initial SoC | `SoC_init` | 0.90 | — | Starting state of charge |
| Max regen power | `P_regen_max` | 60 | kW | Peak recuperation power |
| Auxiliary load | `P_aux` | 1500 | W | Fixed auxiliary power draw |

---

## Logged Outputs

The simulation writes all results to the MATLAB workspace via `To Workspace` blocks. After simulation, access them as fields of the `out` structure:

```matlab
% Example post-processing
plot(out.sim_time, out.vel_ref * 3.6, 'b--', ...
     out.sim_time, out.vel_actual * 3.6, 'r-');
xlabel('Time (s)'); ylabel('Speed (km/h)');
legend('Reference', 'Actual');

% Energy consumption (kWh)
dt = diff(out.sim_time);
E_kWh = trapz(out.sim_time, max(out.p_elec, 0)) / 3.6e6;
fprintf('Energy consumed: %.2f kWh\n', E_kWh);

% Regen recovered
E_regen_kWh = trapz(out.sim_time, abs(min(out.p_elec, 0))) / 3.6e6;
fprintf('Energy recovered: %.2f kWh\n', E_regen_kWh);
```

---

## How to Run

```matlab
% 1. Open MATLAB (R2021b or newer recommended)
% 2. Navigate to the project folder
cd('path/to/navin_model')

% 3. Load parameters and drive cycle into the workspace
run('init_params.m')

% 4. Open and simulate the model
open_system('navin_model')
sim('navin_model')

% 5. Post-process results
% (use the logged signals in the 'out' structure)
```

To run a parameter sweep (e.g., varying vehicle mass):

```matlab
mass_range = 1500:100:2500;   % kg
for k = 1:length(mass_range)
    m_veh = mass_range(k);
    out_k = sim('navin_model');
    SoC_end(k) = out_k.soc(end);
end
plot(mass_range, SoC_end);
```

---

## Dependencies

| Tool | Version | Purpose |
|------|---------|---------|
| MATLAB | R2021b+ | Scripting, post-processing |
| Simulink | R2021b+ | Block diagram simulation |
| Simulink Control Design | Optional | PID auto-tuning |
| Curve Fitting Toolbox | Optional | Fitting battery maps from cell data |

No custom toolboxes or external libraries are required to run the base model.

---

## Results & Interpretation

A typical WLTP Class 3 simulation produces the following approximate outputs:

- **Speed tracking error:** < 0.5 km/h RMS under default PID gains
- **Final SoC:** ~0.72 (18% consumed from 90% start for the full WLTP cycle at 1800 kg)
- **Energy consumption:** ~18–22 kWh/100 km depending on auxiliary load and mass
- **Regen fraction:** ~25–30% of total braking energy recovered
- **Peak motor torque:** Limited by `Max_Torque_Map` at low speeds; power-limited at high speeds
- **Minimum terminal voltage:** Occurs at peak discharge; must remain above pack cut-off voltage

---

## Future Work

- Add a **thermal model** for both motor and battery (temperature-dependent maps)
- Extend to **all-wheel drive** with front/rear torque vectoring
- Replace the single-speed gearbox with a **multi-speed AMT** model
- Integrate a **battery ageing** (degradation) model to study SoC vs. cycle-life trade-offs
- Couple with **HVAC load model** for realistic auxiliary power variation
- Export to **Simulink Real-Time** target for hardware-in-the-loop (HIL) testing

---

## Author

**Navin** — EV Powertrain Simulation Project  
Model: `navin_model.slx`  
MATLAB/Simulink, 2025
