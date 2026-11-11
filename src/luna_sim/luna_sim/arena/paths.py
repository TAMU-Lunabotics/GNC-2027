"""
Resolve the locations and names of arena files.

Launch files and nodes compute the same paths from (out_dir, seed).
"""
import os
import random
import re
from pathlib import Path

PACKAGE = 'luna_sim'


def arena_paths(out_dir, seed):
    """Return the paths the generator writes for one seed."""
    out = Path(out_dir).expanduser().resolve()
    return {
        'json': out / f'artemis_arena_{seed}.json',
        'known_map': out / f'artemis_arena_{seed}_known.yaml',
        'truth_map': out / f'artemis_arena_{seed}_truth.yaml',
    }


def resolve_seed(seed_arg):
    """'' / 'random' -> a fresh random seed; anything else -> int."""
    if seed_arg in ('', 'random', None):
        return random.randint(0, 2**31 - 1)
    return int(seed_arg)


def default_out_dir():
    """
    Find the default arena output directory.

    Use <src>/luna_sim/generated when the source package can be found.
    Otherwise, use $ROS_HOME/luna_sim/arenas.
    """
    try:
        return find_source_package() / 'generated'
    except Exception:
        ros_home = os.environ.get('ROS_HOME', os.path.expanduser('~/.ros'))
        return Path(ros_home) / 'luna_sim' / 'arenas'


def find_source_package(pkg=PACKAGE):
    """Return <ws>/src/.../<pkg>, the package's SOURCE folder (not install/)."""
    from ament_index_python.packages import get_package_prefix
    prefix = Path(get_package_prefix(pkg)).resolve()
    for ws in prefix.parents:
        if (ws / 'src').is_dir() and (ws / 'install').is_dir():
            break
    else:
        raise FileNotFoundError(f"no colcon workspace above {prefix}")
    for package_xml in (ws / 'src').rglob('package.xml'):
        m = re.search(r'<name>\s*([^<\s]+)\s*</name>', package_xml.read_text())
        if m and m.group(1) == pkg:
            return package_xml.parent
    raise FileNotFoundError(f"package '{pkg}' not found under {ws / 'src'}")


def resolve_config(arg, pkg=PACKAGE):
    """Resolve a path or a name in share/<pkg>/config/arena/, with optional .yaml."""
    p = Path(arg).expanduser()
    if p.is_file():
        return p.resolve()
    from ament_index_python.packages import get_package_share_directory
    cfg_dir = Path(get_package_share_directory(pkg)) / 'config' / 'arena'
    for name in (arg, f'{arg}.yaml'):
        if (cfg_dir / name).is_file():
            return (cfg_dir / name).resolve()
    raise FileNotFoundError(
        f"arena config '{arg}' not found (tried it as a path and in {cfg_dir})"
    )
