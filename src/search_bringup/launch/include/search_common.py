"""Shared helpers for the search_bringup launch files."""

import os
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
import yaml

SHARE = get_package_share_directory('search_bringup')


def conda_python() -> str:
    """Use the activated robotics environment, regardless of installed script shebangs."""
    prefix = os.environ.get('CONDA_PREFIX', '')
    executable = os.path.join(prefix, 'bin', 'python')
    if os.path.basename(prefix.rstrip('/')) != 'field_robotics' or not os.path.isfile(executable):
        raise RuntimeError('Activate the field_robotics Conda environment before launching: '
                           'conda activate field_robotics')
    return executable


def load_yaml(*parts: str) -> dict:
    with open(os.path.join(SHARE, *parts)) as stream:
        return yaml.safe_load(stream)


def world_config(world: str) -> dict:
    return load_yaml('config', 'worlds', f'{world}.yaml')


def robots(selection: str = '') -> List[Dict]:
    """Robots from config/robots.yaml, optionally filtered.

    `selection` is a comma-separated list of namespaces or platform names
    (e.g. "husky_a200,j100_0000"); empty means all robots.
    """
    team = load_yaml('config', 'robots.yaml')['robots']
    wanted = [s.strip() for s in selection.split(',') if s.strip()]
    if not wanted:
        return team
    chosen = [r for r in team if r['namespace'] in wanted or r['platform'] in wanted
              or r['platform'].split('_')[0] in wanted]
    if not chosen:
        raise ValueError(f'no robot matches robots:="{selection}"')
    return chosen


def footprint(robot: Dict) -> str:
    """Rectangular Nav2 footprint string centred on base_link."""
    hl, hw = 0.5 * float(robot['length']), 0.5 * float(robot['width'])
    return f'[[{hl}, {hw}], [{hl}, {-hw}], [{-hl}, {-hw}], [{-hl}, {hw}]]'


def setup_dir(namespace: str) -> str:
    """Writable Clearpath setup directory for one robot."""
    return os.path.join(os.path.expanduser('~'), '.ros', 'field_exploration_ws', namespace)
