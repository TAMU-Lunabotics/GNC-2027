# luna_sim

Simulation for the Lunabotics competition.

**What belongs here:** anything that only exists because we're the simulation is running: the arena generator, fake sensors, mock hardware, ground truth, the scorer.

**What doesn't:** anything the real rover also runs (path planning, localization, mission logic, drivers). That code lives in its own package, and the sim *launches* it. Nothing in the autonomy stack may import from or depend on `luna_sim`.


---

## Quick start

```bash
# from the workspace root(i.e GNC-2027)
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select luna_sim
source install/setup.bash

# generate an arena and serve its maps (add rviz:=true to view them)
ros2 launch luna_sim arena.launch.py seed:=42

# or just generate the files
ros2 run luna_sim arena_gen --seed 42
```

Omit `seed` to get a random arena; the chosen seed is printed so you can reproduce it.

---

## Layout

```
luna_sim/
├── config/
│   ├── arena/artemis_arena.yaml   arena rules: dimensions, zones, obstacle ranges
│   └── scenarios/                 (planned) fault-injection scenarios
├── launch/
│   └── arena.launch.py            generate an arena, serve /map and /map_truth
├── luna_sim/                      Python source code (importable as luna_sim.*)
│   ├── arena/
│   │   ├── arena_gen.py           Takes an arena YAML and a seed, then generates maps and a layout JSON
│   │   ├── layout.py              load_layout(): reads an arena .json file, all nodes needing arena data should import this
│   │   └── paths.py               helper functions for where generated files go and what they're named
│   └── eval/
│       └── scorer.py              (planned) scores sim runs
├── scripts/                       executables for `ros2 run` (thin wrappers)
├── generated/                     generated arenas (git-ignored)
├── worlds/                        arenas worth keeping (checked in)
├── src/                           C++ source (none yet)
├── CMakeLists.txt
└── package.xml
```
Note: luna_sim is a cpp package, so some extra work is needed so that python code can be run with "ros2 run ...". The scripts folder is what helps do this. If you need to get a
python file to run and get stuck, let me(JP) know and I can help.

### What goes where

| Folder | Contents | In git? |
|---|---|---|
| `config/` | Hand-written settings: arena rules, scenarios. | Yes |
| `launch/` | Launch files. | Yes |
| `luna_sim/` | All Python logic. Every folder needs an `__init__.py`. | Yes |
| `scripts/` | One 3-line wrapper per runnable command (see below). No logic. | Yes |
| `generated/` | Output of `arena_gen`: per seed, a `.json` layout plus `known` and `truth` maps (`.pgm` + `.yaml`). Disposable. | **No** |
| `worlds/` | Arenas we want to keep, e.g. a seed that caught a bug. Copy them here from `generated/`. Gazebo worlds later. | Yes |
| `src/` | C++ nodes, if we ever need them. | Yes |

---

## Generated arenas

Each run of `arena_gen` writes to `generated/`:

| File | What it is | Who uses it |
|---|---|---|
| `artemis_arena_<seed>.json` | Exact obstacle positions, zones, start pose. Ground truth. | Sim nodes and the scorer, via `load_layout()`. **Never the autonomy stack.** |
| `artemis_arena_<seed>_known.pgm` / `_known.yaml` | empty map, what the robot knows before a run. | `map_server` → `/map` (used for path planning) |
| `artemis_arena_<seed>_truth.pgm` / `_truth.yaml` | All obstacles | `map_server_truth` → `/map_truth` (RViz, debugging) |

File names and locations come from `luna_sim/arena/paths.py`. Don't build these paths by hand.

---

## Adding a runnable node or tool

`luna_sim` is an `ament_cmake` package, which has no `entry_points` like `ament_python`. So each command needs a small executable in `scripts/` that calls into the Python source:

1. Put the logic in `luna_sim/<area>/<name>.py` with a `main()` function.
2. Add `scripts/<name>.py`:
   ```python
   #!/usr/bin/env python3
   from luna_sim.<area>.<name> import main

   if __name__ == '__main__':
       main()
   ```
3. Make it executable: `chmod +x scripts/<name>.py`
4. Add `<name>` to `PY_EXECUTABLES` in `CMakeLists.txt`.
5. Rebuild. It now runs as `ros2 run luna_sim <name>`.

Naming: snake_case, script name = command name = node name.

---

## Rules

- **Ground truth stays in the sim.** `/sim/**`, `/map_truth` and `arena_<seed>.json` are for sim nodes, the scorer and debugging only.
- **Use the node clock.** Never `time.time()` or `time.sleep()`; everything runs on `use_sim_time`.
- **Use full imports.** `from luna_sim.arena.paths import ...`, never `from paths import ...`.
- **Nothing generated goes in git.** Promote arenas you want to keep into `worlds/`.

---