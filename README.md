# Aquabot MPPI

**GPU-accelerated Smooth Model Predictive Path Integral (SMPPI) control for the [Aquabot Challenge](https://github.com/oKermorgant/aquabot)** — an autonomous boat that has to plan a rock-aware route through a wind-turbine farm, orbit each turbine, and read its QR code.

This project is a fork of the [Centrale Nantes ROS 2 Aquabot lab](https://github.com/oKermorgant/aquabot) by O. Kermorgant, itself adapted from the [Sirehna Aquabot Challenge](https://github.com/sirehna/aquabot). The lab scaffolding (simulator, EKF localization, base planner/control interfaces) comes from that repo — what's added here is a full GPU SMPPI controller, a rock-aware global planner, and a continuous-orbit inspection mission.

<!-- TODO: hero GIF/video here, e.g.
![demo](docs/demo.gif)
or link a video: [Watch the full inspection run](docs/demo.mp4)
-->

## Table of contents

- [Overview](#overview)
- [What this fork adds](#what-this-fork-adds)
- [Architecture](#architecture)
- [Controller: Smooth MPPI](#controller-smooth-mppi)
- [Global planner](#global-planner-rock-aware-a)
- [Mission: continuous turbine inspection](#mission-continuous-turbine-inspection)
- [QR inspection](#qr-inspection-in-progress)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#results)
- [Roadmap](#roadmap--known-limitations)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## Overview

The simulation drops an autonomous boat — two independently steerable azimuth thrusters (±π/4), a pan camera — into a Gazebo world scattered with islands, rocks, and wind turbines. Each turbine carries a QR code on one side. The boat has to:

1. Localize itself and the turbines from noisy GPS/IMU/AIS data
2. Plan a route through the field without hitting any rock or island
3. Track that route accurately enough to hold a tight orbit around each turbine
4. Read every QR code, then station-keep in front of a pinger-designated turbine for a final inspection pass

Everything runs over ROS 2 topics in the `/aquabot` namespace — see the [base repo](https://github.com/oKermorgant/aquabot) for the full topic/message reference.

## What this fork adds

| Component | File(s) | Replaces / adds |
|---|---|---|
| SMPPI controller | `motion_node.cpp`, `mppi_kernels.cu` | the baseline PID velocity tracker |
| Rock-aware global planner | `planner.py` | — (bitangent visibility-graph A\*) |
| Continuous orbit mission | `mission_turbines.py` | discrete stop-and-go waypoint inspection |
| QR decoding | `qr_reader.py` | — (multi-strategy OpenCV decode, WIP) |

## Architecture

![architecture](docs/architecture.svg)

Sensors flow through the course's EKF into `/odom`. The turbine AIS positions feed a rock-aware A\* planner, which the mission state machine calls once per turbine to get an approach leg, then appends a dense analytic circle to it — publishing one continuous path rather than a string of waypoints. The SMPPI controller tracks that path: reference-trajectory construction, noise sampling and the control update run on the CPU (OpenMP), while the actual rollout simulation and cost evaluation for all K trajectories run in parallel on the GPU (CUDA). A separate QR-reading node watches the camera independently.

## Controller: Smooth MPPI

### Why "smooth"

Vanilla MPPI samples and applies raw actuator *values*, which tends to produce jittery commands. Here the sampled controls are **rates** — `d(thrust)/dt` and `d(angle)/dt` for each thruster — which get integrated and clamped to `max_thrust_delta` / `max_angle_delta` before being applied. The result is a control sequence that respects real actuator slew limits by construction, instead of relying on the cost function to discourage jerkiness after the fact.

### Dynamics model

A 3-DOF Fossen surge/sway/yaw model, identical on CPU (used to publish the predicted optimal path) and GPU (used inside the rollout kernel):

- Linear + quadratic drag on each of surge (`u`), sway (`v`), yaw rate (`r`)
- Coriolis-centripetal coupling between surge/sway/yaw
- Integrated with RK4 at `dt = 0.1 s`

<!-- TODO: confirm which of these are tuned-to-sim vs. estimated, if worth noting -->

| Param | Meaning | Value |
|---|---|---|
| `L`, `W` | hull length / width | 6.0 m, 1.2 m |
| `m_u`, `m_v`, `m_r` | effective mass/inertia (surge, sway, yaw) | 1000, 1000, 446 |
| `d_u`, `d_v`, `d_r` | linear drag | 182, 183, 1199 |
| `d_u_quad`, `d_v_quad`, `d_r_quad` | quadratic drag | 224, 149, 979 |

### Sampling

| Param | Value |
|---|---|
| Rollouts `K` | 4000 |
| Horizon | 60 steps (6 s lookahead @ 0.1 s) |
| Noise model | Gaussian, common-mode + differential-mode per thruster pair (so trim/vectoring is sampled separately from paired thrust/steer) |
| `λ` (temperature) | 40.0 |
| Update gain | 0.6 |

All `K × horizon` noise draws are generated CPU-side with OpenMP across cores, then shipped to the GPU alongside the current state and nominal control sequence. `mppi_rollout_kernel` assigns one CUDA thread per rollout, forward-simulates the full horizon, and returns a single cost per rollout.

### Reference trajectory — the "ghost boat"

Rather than tracking a fixed-speed carrot, the reference point walks along `/aquabot/plan` at a speed that adapts to the boat's *current* speed (floored at a minimum, led by a small acceleration margin, capped at cruise target). This keeps the reference from running away from a boat that's still accelerating, while still pulling it up to cruise speed.

### Cost function

Evaluated once per rollout per horizon step, summed over the horizon:

| Term | Shape | Purpose |
|---|---|---|
| Cross-track error | Huber (quadratic < 1 m, linear beyond) | keep the boat on the path; doesn't blow up if a sampled rollout is briefly far off |
| Along-track error | quadratic, low weight | loose — doesn't fight the speed term |
| Heading error | quadratic, with deadband | align with the path tangent |
| Speed tracking | quadratic vs. fixed target speed | maintain cruise speed through turns, no brake-slamming |
| Yaw-rate | quadratic | discourage spin-out |
| Thrust effort | quadratic | frugal throttle use |
| Steering effort | quadratic, low weight | prefer vectoring (turning both nozzles) over aggressive differential thrust |

### Control update & receding horizon

Rollout costs are converted to importance weights via `w_k ∝ exp(-(cost_k − min_cost)/λ)`, normalized, and used to nudge the nominal control-rate sequence toward the lower-cost rollouts. The rate sequence is clamped to prevent windup, the first step is integrated and published, and the whole sequence is shifted left by one step (receding horizon) with the tail zero-padded for the next cycle.

### CPU/GPU split

| Stage | Where |
|---|---|
| Reference trajectory construction | CPU |
| Noise sampling (K × horizon × 4) | CPU, OpenMP-parallel across rollouts |
| Rollout simulation (RK4 dynamics × K) | **GPU**, one CUDA thread per rollout |
| Cost evaluation | **GPU**, fused into the same kernel |
| Importance-weighted update | CPU |
| RViz visualization (rollout "tentacles", optimal path) | CPU, throttled |

The control loop logs a `setup / noise / gpu / viz / rest` timing breakdown each cycle so you can see where the budget goes relative to the `dt = 0.1 s` (100 ms) control period.

<!-- TODO: drop in actual measured timings / GPU model once you have numbers, e.g.
On an RTX ????, a full 4000-rollout × 60-step cycle takes ~?? ms, leaving headroom for the 100 ms loop.
-->

## Global planner: rock-aware A*

`planner.py` builds a bitangent-based visibility graph around every obstacle — islands, rocks, and (once known) turbines — and runs A\* over it, exposed as:

- A `nav_msgs/GetPlan` service on `/aquabot/get_plan`
- A `goal_pose` subscription for RViz's **2D Goal Pose** button (uses the boat's current pose as start)
- Obstacle markers published for RViz visualization

## Mission: continuous turbine inspection

`mission_turbines.py` runs a small state machine (`WAITING_FOR_DATA → PLAN_APPROACH → WAITING_FOR_PLAN → ORBITING → …`) that, for each turbine in turn:

1. Calls the A\* planner for an approach leg to the edge of the inspection ring
2. Appends a dense analytic circle (36 points, configurable arc/direction/radius) starting exactly where the approach leg ends
3. Publishes the concatenation as **one continuous path** on `/aquabot/plan`

The single-path design is deliberate: driving discrete stop-and-go waypoints around a turbine saturates the tracker's steering and causes oscillation. One smooth path lets the SMPPI controller stay in its comfortable tracking regime the whole way round. Once a turbine's orbit sweeps its configured completion arc, the mission advances to the next turbine's approach — the boat never fully stops between turbines.

## QR inspection (in progress)

`qr_reader.py` currently covers step 1 of the optional challenge: prove decoding works. It runs OpenCV's QR detector through four strategies in order (plain → curved → 2× upscaled → Otsu-binarized), logs how much of the frame the code covers and its aspect ratio (to find the usable read range), and republishes each newly-seen code to `/vrx/windturbinesinspection/windturbine_checkup`.

Not yet wired: camera aiming, facing-direction computation, and handing off to the mission's 30-second station-keep + round-turn once all codes are read.

## Installation

<!-- TODO: fill in with your actual setup once you split off from the private fork -->

```bash
# same ROS 2 / Gazebo setup as the base repo — see
# https://github.com/oKermorgant/aquabot for distro + install details
git clone <this-repo-url>
cd <this-repo>
# extra dependencies for the SMPPI controller:
#   CUDA toolkit (nvcc)
#   Eigen3
#   OpenMP
# extra dependency for the QR node:
#   python3-opencv, cv_bridge
colcon build --symlink-install
```

## Usage

```bash
ros2 launch aquabot_gz turbines_launch.py world:=medium
# TODO: your launch file(s) for EKF + planner + SMPPI controller + mission
ros2 launch <your_package> aquabot_mppi.launch.py
```

## Results

<!-- TODO: tracking-error plots, before/after vs. the baseline PID, video links -->

## Roadmap / known limitations

- [ ] Wire QR facing/aiming into the mission's station-keep phase
- [ ] 30 s stabilization + round turn in front of the pinger-designated turbine
- [ ] Quantify tracking performance vs. the baseline PID controller

## Acknowledgments

- [O. Kermorgant / Centrale Nantes](https://github.com/oKermorgant/aquabot) — course lab scaffolding, simulator packages, EKF/localization helpers
- [Sirehna](https://github.com/sirehna) — original Aquabot Challenge

## License

This project inherits the Apache-2.0 license of the upstream repository — see [`LICENSE`](LICENSE).
