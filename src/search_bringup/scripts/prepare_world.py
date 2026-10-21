#!/usr/bin/env python3
"""Copy an rmf_demos world and add the Gazebo systems Clearpath sensors need.

rmf_building_map_tools generates worlds with Physics, UserCommands and
SceneBroadcaster only. Clearpath's GPU lidar and camera need the rendering
Sensors system, the IMU needs the Imu system, and NavSat needs the NavSat
system plus <spherical_coordinates> (otherwise every GPS update fails and
floods the log). The RMF demo fleet robots placed in the world are removed:
they belong to RMF's own fleets, not to this experiment. The rmf_demos source
is left untouched; this writes a copy.

    prepare_world.py INPUT_WORLD OUTPUT_WORLD
"""

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

RMF_DEMO_ROBOTS = (
    'model://CleanerBotA',
    'model://CleanerBotE',
    'model://DeliveryRobot',
    'model://TinyRobot',
    'model://Open-RMF/CleanerBotA',
    'model://Open-RMF/CleanerBotE',
    'model://Open-RMF/DeliveryRobot',
    'model://Open-RMF/TinyRobot',
)

ADDITIONS = """<additions>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
  <plugin filename="gz-sim-scene-broadcaster-system"
          name="gz::sim::systems::SceneBroadcaster"/>
  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>30.6188</latitude_deg>
    <longitude_deg>-96.3365</longitude_deg>
    <elevation>103.0</elevation>
    <heading_deg>0.0</heading_deg>
  </spherical_coordinates>
</additions>"""


def add_targets(world: ET.Element, config: Path) -> None:
    """Add fixed sphere targets without modifying upstream RMF assets."""
    settings = yaml.safe_load(config.read_text()) or {}
    for target in settings.get('targets', []):
        name = str(target['name'])
        position = [float(value) for value in target['position']]
        color = [float(value) for value in target['color']]
        radius = float(target['radius'])
        if (not name or len(position) != 3 or not all(map(math.isfinite, position)) or
                not math.isfinite(radius) or radius <= 0 or len(color) != 4 or
                not all(math.isfinite(value) and 0 <= value <= 1 for value in color)):
            raise ValueError(f'Invalid sphere target in {config}: {name}')
        for existing in world.findall('model'):
            if existing.get('name') == name:
                world.remove(existing)
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, position)) + ' 0 0 0'
        link = ET.SubElement(model, 'link', name='ball')
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name='ball_' + kind)
            sphere = ET.SubElement(ET.SubElement(element, 'geometry'), 'sphere')
            ET.SubElement(sphere, 'radius').text = str(radius)
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                for field in ('ambient', 'diffuse'):
                    ET.SubElement(material, field).text = ' '.join(map(str, color))
                ET.SubElement(material, 'specular').text = '0.05 0.05 0.05 1'


def prepare(source: Path, target: Path, config: Path | None = None) -> None:
    root = ET.fromstring(source.read_text())
    world = root.find('world')
    if world is None:
        raise RuntimeError(f'no <world> in {source}')
    for include in list(world.findall('include')):
        uri = (include.findtext('uri') or '').strip()
        if uri in RMF_DEMO_ROBOTS:
            world.remove(include)
    names = {plugin.get('name') for plugin in world.findall('plugin')}
    for element in ET.fromstring(ADDITIONS):
        if element.tag == 'plugin':
            if element.get('name') in names:
                continue
        elif world.find(element.tag) is not None:
            continue
        world.append(element)
    if config is not None:
        add_targets(world, config)
    ET.indent(root)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(ET.tostring(root, encoding='unicode'))


def main() -> None:
    if len(sys.argv) not in (3, 4):
        raise SystemExit(f'usage: {Path(sys.argv[0]).name} INPUT_WORLD OUTPUT_WORLD [WORLD_CONFIG]')
    prepare(Path(sys.argv[1]), Path(sys.argv[2]),
            Path(sys.argv[3]) if len(sys.argv) == 4 else None)


if __name__ == '__main__':
    main()
