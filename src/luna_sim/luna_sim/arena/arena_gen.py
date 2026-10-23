import argparse
import math
import yaml
import random
import json
import os
import hashlib
from pathlib import Path
from luna_sim.arena.paths import default_out_dir, resolve_seed

def load_yaml(file_path):
    """
    Load the arena YAML file.

    Args:
        file_path (str): Path to the arena YAML file.

    Returns:
        dict: The full config, with 'zones' and 'obstacles' as name -> data mappings.
    """
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)

    for key in ('arena', 'zones', 'obstacles', 'start', 'placement'):
        if key not in config:
            raise KeyError(f"{file_path}: missing required section '{key}'")

    return config['arena'], config['zones'], config['obstacles'], config['placement'], config['start'], config.get('tags', [])


def generate_obstacles(obstacle_specs, zones, arena, placement):
    """
    Randomly generates obstacles based on the provided specifications and zones.

    Args:
        obstacle_specs (dict): List of obstacle specifications.
        zones (dict): Dictionary of zones with their properties.
        arena (dict): Dictionary containing arena properties.
        placement (dict): Dictionary containing placement parameters.

    Returns:
        list: List of generated obstacles with their dimensions and positions.
    
    """
    spacing = placement['min_spacing']
    margin = placement['wall_margin']
    max_attempts = placement['max_attempts']
    keep_out = [zones[zone_name] for zone_name in placement['keep_out']]
    generated_obstacles = []

    for obs_type in obstacle_specs.keys():
        zone = zones[obstacle_specs[obs_type]['zone']]

        count = random.randint(obstacle_specs[obs_type]['count'][0], obstacle_specs[obs_type]['count'][1])
        print(f"Generating {count} {obs_type} in zone '{zone}'...")

        for i in range(count):
            for attempt in range(max_attempts):  # Limit the number of attempts to place an obstacle
                diameter = random.uniform(obstacle_specs[obs_type]['diameter'][0], obstacle_specs[obs_type]['diameter'][1])
                r = diameter / 2
                x_lo = max(zone['cx'] - zone['length'] / 2 + r, margin + r)
                x_hi = min(zone['cx'] + zone['length'] / 2 - r, arena['length'] - margin - r)
                y_lo = max(zone['cy'] - zone['width'] / 2 + r, margin + r)
                y_hi = min(zone['cy'] + zone['width'] / 2 - r, arena['width'] - margin - r)

                x = random.uniform(x_lo, x_hi)
                y = random.uniform(y_lo, y_hi)

                if any(circle_hits_rect(x, y, r, keep_out_zone) for keep_out_zone in keep_out):
                    continue # try again if the obstacle overlaps with any keep-out zones
                if any(math.hypot(x - obs['position']['x'], y - obs['position']['y']) < (r + obs['diameter'] / 2 + spacing) for obs in generated_obstacles):
                    continue  # try again if it overlaps with existing obstacles

                break
            else:
                if i< obstacle_specs[obs_type]['count'][0]:
                    raise RuntimeError(f"Could not place {obs_type} after {max_attempts} attempts, and minimum count not reached.")
                
                print(f"Warning: Could not place {obs_type} after {max_attempts} attempts. Skipping.")
                continue  # Skip this obstacle if we couldn't place it after max_attempts

            if 'crater' in obs_type:
                depth = random.uniform(obstacle_specs[obs_type]['depth'][0], obstacle_specs[obs_type]['depth'][1])
                generated_obstacles.append({
                    'name': f"{obs_type}_{i + 1}",
                    'type': obs_type,
                    'diameter': diameter,
                    'depth': depth,
                    'position': {'x': x, 'y': y}
                })
            else:
                height = random.uniform(obstacle_specs[obs_type]['height'][0], obstacle_specs[obs_type]['height'][1])
                generated_obstacles.append({
                    'name': f"{obs_type}_{i + 1}",
                    'type': obs_type,
                    'diameter': diameter,
                    'height': height,
                    'position': {'x': x, 'y': y}
                })

    print(f"Generated {len(generated_obstacles)} obstacles in total.")

    return generated_obstacles


def generate_start_position(start_spec, zones, arena, placement):
    """
    Randomly generates the robot's starting position and orientation within the starting zone

    Args:
        start_spec (dict): Specifications of the starting zone.
        zones (dict): Dictionary of zones with their properties.
        arena (dict): Dictionary containing arena properties.

    Returns:
        dict: The robot's x, y and yaw position as a list.
    """

    zone = zones[start_spec['zone']]

    pose = {}

    margin = placement['wall_margin']

    x_lo = start_spec['length'][0] + margin
    x_hi = start_spec['length'][1] - margin

    y_lo = start_spec['width'][0] + margin
    y_hi = start_spec['width'][1] - margin


    pose['x'] = random.uniform(x_lo, x_hi)
    pose['y'] = random.uniform(y_lo, y_hi)
    pose['yaw'] = random.choice(start_spec['yaw'])

    return pose


def circle_hits_rect(x, y, r, rect):
    """Helper to determine if a circle at (x, y) with radius r overlaps a {cx, cy, length, width} rectangle."""
    dx = max(rect['cx'] - rect['length'] / 2 - x, 0.0, x - (rect['cx'] + rect['length'] / 2))
    dy = max(rect['cy'] - rect['width'] / 2 - y, 0.0, y - (rect['cy'] + rect['width'] / 2))
    return math.hypot(dx, dy) < r


def generate_occupancy_grid(obstacles, arena, resolution):
    """
    Generates the occupancy grid for the arena using the obstacles, as a 1D array.

    Args:
        obstacles (Dict) Dictionary of all generated obstacles and their properties.
        zones (dict): Dictionary of zones with their properties.
        arena (dict): Dictionary containing arena properties.
        resolution (float): The size of map cells in m/cell.

    Returns:
        data: List of cells on the map, 1.0 means an object is present, 0.0 means open.
        int: The number of columns in the grid, equal to the length of the arena scaled by the resolution.
        int: The number of rows in the grid, equal to the width of the arena scaled by the resolution.
    """

    cols = round(arena['length'] / resolution) # x-axis
    rows = round(arena['width'] / resolution) # y-axis

    data = [0.0] * (rows * cols)

    for obstacle in obstacles:
        x_c = obstacle['position']['x']
        y_c = obstacle['position']['y']
        r = obstacle['diameter'] / 2.0

        # calculate the bounding box for the obstacle
        col_lo = max(0, math.floor((x_c - r) / resolution))
        col_hi = min(cols - 1, math.floor((x_c + r) / resolution))

        row_lo = max(0, math.floor((y_c - r) / resolution))
        row_hi = min(rows - 1, math.floor((y_c + r) / resolution))

        for row in range(row_lo, row_hi + 1):
            for col in range(col_lo, col_hi + 1):
                x = (col + 0.5) * resolution # the x, y of the cells center
                y = (row + 0.5) * resolution
                if (x - x_c)**2 + (y - y_c)**2 <= r**2:
                    data[row * cols + col] = 1.0 # sets this point to 1.0, i.e. occupied, if the point is within the cirlce of the obstacle

    return data, cols, rows


def write_map(out_dir, name, data, cols, rows, resolution):
    """
    Write an occupancy grid as a nav2 map_server map: <name>.pgm + <name>_map.yaml.
 
    Args:
        out_dir (str): Directory to write into (created if missing).
        name (str): Base file name, e.g. 'arena_42_truth'.
        data (list): rows * cols values, 1 = occupied, 0 = free. Row 0 is y = 0 (bottom).
        cols (int): Cells along x.
        rows (int): Cells along y.
        resolution (float): Meters per cell.
 
    Returns:
        Path: The _map.yaml path.
    """
    if len(data) != rows * cols:
        raise ValueError(f"grid has {len(data)} cells, expected rows * cols = {rows * cols}")

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / Path(f"{name}.pgm")
    yaml_path = out_dir / Path(f"{name}.yaml")
 
    # PGM pixel values (map_server trinary mode, negate: 0):
    #   0 (black) = occupied, 254 (white) = free.
    OCCUPIED, FREE = 0, 254
 
    # PGM rows go top to bottom; grid row 0 is the BOTTOM of the map (y = 0).
    # Write grid rows in reverse, or the map comes out mirrored top-to-bottom.
    pixels = bytearray()
    for row in reversed(range(rows)):
        start = row * cols
        pixels.extend(OCCUPIED if v else FREE for v in data[start:start + cols])
 
    # Binary PGM ("P5"): text header, then one byte per pixel.
    header = f"P5\n# generated by arena_gen.py\n{cols} {rows}\n255\n".encode("ascii")
    _atomic_write_bytes(pgm_path, header + bytes(pixels))

    # map_server metadata. `image` is relative to this yaml's directory, so the
    # folder can be moved as a whole.
    map_yaml = (
        f"image: {pgm_path.name}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [0.0, 0.0, 0.0]   # bottom-left pixel = arena corner (map frame)\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    _atomic_write_bytes(yaml_path, map_yaml.encode("ascii"))
 
    occupied = sum(1 for v in data if v)
    print(f"Wrote {pgm_path.name} ({cols} x {rows} cells, {occupied} occupied) and {yaml_path.name}")
    return yaml_path


def write_layout_json(out_dir, config_path, seed, arena, zones, placement, obstacles, tags,
                      start_pose, map_files=None):
    """
    Write the generated arena layout as a .json.
 
    This is the ground truth for one generated arena: the collision checker, scorer,
    fake obstacle sensor, and later the Gazebo world read it, so they all agree.
 
    Args:
        out_dir (str): Directory to write into (created if missing).
        config_path (str): Path of the arena YAML this layout came from.
        seed (int): Seed used for generation.
        arena (dict): Dictionary containing arena properties.
        zones (dict): Zone name -> {cx, cy, length, width}.
        placement (dict): The 'placement' section (keep_out is copied into the output).
        obstacles (list): Obstacles from generate_obstacles().
        start_pose (dict): {'x', 'y', 'yaw'} from generate_start_position().
        map_files (dict): Optional, e.g. {'known': Path(...), 'truth': Path(...)} from write_map().
 
    Returns:
        Path: The file written.
    """
    config_path = Path(config_path).resolve()
    layout = {
        "seed": seed,
        "config": {
            "path": str(config_path),
            "sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        },
        "frame": {
            "id": "map",
            "units": "meters, radians",
            "origin": "arena wall corner in the starting zone, regolith surface",
        },
        "arena": {
            "length": arena["length"],
            "width": arena["width"],
            "regolith_depth": arena["depth"],
        },
        "zones": {name: _round_dict(z) for name, z in zones.items()},
        "keep_out": list(placement.get("keep_out", [])),
        "start_pose": _round_dict(start_pose),
        "obstacles": [_obstacle_entry(o) for o in obstacles],
        "tags": tags,
        "maps": {k: Path(v).name for k, v in (map_files or {}).items()},
    }
 
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"artemis_arena_{seed}.json"
    _atomic_write_bytes(out_path, (json.dumps(layout, indent=2) + "\n").encode("utf-8"))
 
    print(f"Wrote {len(obstacles)} obstacles to {out_path.name}")
    return out_path


def _obstacle_entry(obs):
    """Normalize one obstacle: explicit kind, flat x/y, rounded numbers."""
    kind = "crater" if "depth" in obs else "boulder"   # from the data, not the group name
    entry = {
        "name": obs["name"],
        "kind": kind,
        "group": obs["type"],
        "x": round(obs["position"]["x"], 4),
        "y": round(obs["position"]["y"], 4),
        "diameter": round(obs["diameter"], 4),
    }
    if kind == "crater":
        entry["depth"] = round(obs["depth"], 4)
    else:
        entry["height"] = round(obs["height"], 4)
    return entry
 
 
def _atomic_write_bytes(path, content):
    """Write to a temp file, then rename, so readers never see a half-written file."""
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "wb") as f:
        f.write(content)
    os.replace(tmp, path)
 
 
def _round_dict(d):
    return {k: round(float(v), 4) if isinstance(v, (int, float)) and not isinstance(v, bool) else v
            for k, v in d.items()}


def generate_map(config, seed, out_dir, resolution):
    random.seed(seed)
    arena, zones, obstacles, placement, start, tags = load_yaml(config)

    obstacles = generate_obstacles(obstacles, zones, arena, placement)

    start_pose = generate_start_position(start, zones, arena, placement)

    known_name = f"artemis_arena_{seed}_known"
    truth_name = f"artemis_arena_{seed}_truth"

    write_map(out_dir, known_name, *generate_occupancy_grid([], arena, resolution), resolution)
    write_map(out_dir, truth_name, *generate_occupancy_grid(obstacles, arena, resolution), resolution)
    write_layout_json(out_dir, config, seed, arena, zones, placement, obstacles, tags, start_pose, {'known': Path(known_name), 'truth': Path(truth_name)})

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default = "src/luna_sim/config/arena/artemis_arena.yaml", help='path to the arena YAML file')
    parser.add_argument('--seed', type=int, default=None, help='random seed for reproducibility')
    parser.add_argument('--out', default=None, help='output folder (default: luna_sim/generated)')
    parser.add_argument('--resolution', type=float, default=0.05, help='the size of map cells in m/cell')

    args = parser.parse_args()

    seed = resolve_seed(args.seed)
    print(f"Seed: {seed}")

    generate_map(args.config, seed, args.out or default_out_dir(), args.resolution)


if __name__ == "__main__":
    main()