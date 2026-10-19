# Benchmark

`tools/benchmark.py` runs N autonomous explorations of the house and scores them.

```bash
python3 mobile_robot_bringup/tools/benchmark.py --backend sim2d --runs 20 --duration 300 --write   # about 4 minutes, no ROS needed
python3 mobile_robot_bringup/tools/benchmark.py --backend gazebo --runs 5 --duration 300 --out docs/benchmark_gazebo.json   # needs the built workspace
```

Every run starts at a seeded random free place with a random heading, with 4 walking people and the wet tile present.

## Two backends, and what each one proves
| | `sim2d` (default) | `gazebo` |
|---|---|---|
| What runs | the repository's real `auto_explore.py`, mux safety layer (`guard`, `limit_step`) and `scan_degrader.py`, in a kinematic model of the house read from `house_world.sdf` | the whole stack: Gazebo physics, SLAM, mux, autopilot, `benchmark_monitor.py` |
| Walls, furniture, props, people | collision shapes read from the world file (a test checks them) | the real simulation |
| Speed | 300 s of robot time in about 10 s, reproducible per seed | real time, one fresh simulation per run |
| Proves | the control logic behaves, regressions are caught, numbers are repeatable | the physical system behaves |
| Does **not** prove | contact dynamics, camera, SLAM drift, real timing | n/a |

The table in the README comes from `sim2d`. The Gazebo backend is implemented and its metric code is unit-tested, but **it has not been run
yet**: it needs the simulator. Run it on your machine and commit `docs/benchmark_gazebo.json`.

## Definitions (`scripts/benchmark_metrics.py`)
* **coverage**: share of the reachable floor (97 m², inflated 0.26 m from obstacles, connected to the start) that the robot came within 0.5 m of.
* **collision with walls / furniture / props**: the robot outline touches an obstacle; counted once per contact.
* **person contact**: the outline comes within a person's body radius (0.22 m). People are Gazebo *actors*: they have no collision and
  walk through the robot, so this counts what a real person would have experienced.
* **minimum clearance**: smallest gap between the outline and walls / furniture over the run.
* **stops**: times the autopilot or the safety layer halted the robot in front of something.

## What the benchmark found (kept, because it is the point)
Measured on the same 20 seeds (`docs/benchmark_before.json` is the first autopilot, `docs/benchmark_results.json` the current one):

| | before | after |
|---|---|---|
| runs with a person contact | 11 of 20 | 11 of 20 |
| person contacts per run | 3.90 | 3.25 |
| runs with a wall / furniture collision | 1 of 20 | 1 of 20 |

1. The autopilot never walks into the world's static geometry, but **people walk into the autopilot**: a person approaching head on at
   0.7 to 0.9 m/s closes the 0.38 m stopping distance faster than the robot can leave. The robot's maximum speed is 0.6 m/s, so it cannot outrun a walker; backing away earlier helps only in part.
2. Adding "yield to something that approaches by itself" **cut person contacts per run by about 17 %** (not the number of affected runs) but at first **raised wall contacts**: the robot was backing into
   spots where turning on the spot swept its nose corner (0.30 m from the axle) through a wall. The fix, a clearance check before rotating, is in `auto_explore.py`
   (`TURN_CLEAR`). The benchmark caught both the problem and the regression.
3. Coverage after 5 minutes is low (about a fifth of the floor): a reactive wanderer, not a planner. The patrol mission (`docs/NAVIGATION.md`) is the way to cover the whole floor.
4. What is still open: slow side-swipes past a pallet corner (the guard only looks ahead), and head-on walkers. The honest fix for the second is to plan around people, not to flee them.
