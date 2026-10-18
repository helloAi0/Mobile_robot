#!/usr/bin/env python3
"""Fix walking people that float above / sink into the ground (no ROS needed, plain python).

   python3 ~/dev_ws/src/mobile_robot/mobile_robot_gazebo/scripts/tune_actor_height.py 0.0            # women (actors named woman_*)
   python3 ~/dev_ws/src/mobile_robot/mobile_robot_gazebo/scripts/tune_actor_height.py 1.0 man        # men   (actors named man_* / pedestrian_man*)
   (also works as:  ros2 run mobile_robot_gazebo tune_actor_height.py 0.0 )

Smaller number = lower. Rewrites the waypoints of house_world.sdf and street_world.sdf. Restart the simulation afterwards.
To find the right number: ros2 launch mobile_robot_bringup launch_sim.launch.py world:=actor_height_test.sdf slam:=false
(7 women walking at different heights; lane n has n red cubes).
"""
import os
import re
import sys


def worlds_dir():
    here = os.path.dirname(os.path.realpath(__file__))        # symlink-install: realpath = the repo's scripts/
    cand = os.path.join(here, '..', 'worlds')
    if os.path.isdir(cand):
        return os.path.realpath(cand)
    from ament_index_python.packages import get_package_share_directory
    return os.path.realpath(os.path.join(get_package_share_directory('mobile_robot_gazebo'), 'worlds'))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    z = float(sys.argv[1])
    men = len(sys.argv) > 2 and sys.argv[2].startswith('man')
    pat = r'<actor name="(?:man_|pedestrian_man)[^"]*">.*?</actor>' if men else r'<actor name="woman_[^"]*">.*?</actor>'
    total = 0
    for name in ('house_world.sdf', 'street_world.sdf'):
        path = os.path.join(worlds_dir(), name)
        if not os.path.exists(path):
            continue
        text = open(path).read()
        count = 0

        def fix_actor(m):
            nonlocal count

            def fix_pose(p):
                nonlocal count
                count += 1
                return f'{p.group(1)}{p.group(2)} {p.group(3)} {z} {p.group(4)}'
            return re.sub(r'(<waypoint><time>[^<]*</time><pose>)(\S+) (\S+) \S+ ([^<]*)', fix_pose, m.group(0))

        text = re.sub(pat, fix_actor, text, flags=re.S)
        open(path, 'w').write(text)
        total += count
        print(f'{name}: {count} waypoints set to z={z}')
    print('Restart the simulation to see the change.' if total else 'No matching actors found.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
