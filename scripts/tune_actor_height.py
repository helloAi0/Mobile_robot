#!/usr/bin/env python3
"""Fix walking people that float above / sink into the ground.

   ros2 run mobile_robot tune_actor_height.py 0.8            # women (actors named woman_*)
   ros2 run mobile_robot tune_actor_height.py 1.0 man        # men  (actors named pedestrian_man*)

Smaller number = lower. Typical range 0.7 ... 1.1. Then restart the simulation. Edits worlds/street_world.sdf
in the package source (works because of 'colcon build --symlink-install').
"""
import os
import re
import sys

from ament_index_python.packages import get_package_share_directory


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    z = float(sys.argv[1])
    prefix = 'pedestrian_man' if len(sys.argv) > 2 and sys.argv[2].startswith('man') else 'woman_'
    path = os.path.realpath(os.path.join(get_package_share_directory('mobile_robot'), 'worlds', 'street_world.sdf'))
    text = open(path).read()
    count = 0

    def fix_actor(m):
        nonlocal count
        block = m.group(0)
        def fix_pose(p):
            nonlocal count
            count += 1
            return f'{p.group(1)}{p.group(2)} {p.group(3)} {z} {p.group(5)}'
        return re.sub(r'(<waypoint><time>[^<]*</time><pose>)(\S+) (\S+) (\S+) ([^<]*)', fix_pose, block)

    text = re.sub(r'<actor name="%s[^"]*">.*?</actor>' % prefix, fix_actor, text, flags=re.S)
    open(path, 'w').write(text)
    print(f'{count} waypoints of "{prefix}*" set to z={z} in {path}\nRestart the simulation to see it.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
