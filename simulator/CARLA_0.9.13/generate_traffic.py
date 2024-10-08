#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate a single vehicle in a specified location"""

import glob
import os
import sys
import carla
import argparse
import logging

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--filterv', metavar='PATTERN', default='vehicle.tesla.model3', help='Filter vehicle model (default: "vehicle.tesla.model3")')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    try:
        sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except IndexError:
        pass

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    world = client.get_world()
    blueprints = world.get_blueprint_library().filter(args.filterv)

    # Fixed spawn point at the origin with no rotation
    spawn_point = carla.Transform(carla.Location(x=-54.1, y=65.0, z=1.0), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))

    # Select the vehicle blueprint
    blueprint = blueprints[0]  # Assuming the filter returns at least one blueprint
    if blueprint.has_attribute('color'):
        color = blueprint.get_attribute('color').recommended_values[0]
        blueprint.set_attribute('color', color)

    # Spawn the vehicle
    vehicle = world.spawn_actor(blueprint, spawn_point)
    print('spawned vehicle: %s at %s' % (vehicle.type_id, spawn_point.location))

    # Keep the vehicle for some time
    import time
    time.sleep(10)  # Keep the vehicle for 10 seconds

    # Cleanup and exit
    # vehicle.destroy()
    # print('vehicle destroyed')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\ndone.')
