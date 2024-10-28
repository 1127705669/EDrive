#!/usr/bin/env python

import glob
import os
import sys
import carla
import logging

class VehicleGenerator:
    def __init__(self, host='127.0.0.1', port=2000, vehicle_type='vehicle.tesla.model3'):
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
        self.client = self.init_carla_client(host, port)
        self.vehicle_type = vehicle_type
        self.vehicle = None  # 存储单个车辆的引用

    def init_carla_client(self, host, port):
        """
        Initialize and return a CARLA client connected to a specified server.
        """
        try:
            sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
                sys.version_info.major,
                sys.version_info.minor,
                'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
        except IndexError:
            logging.error("CARLA Python API egg file not found.")
            pass

        client = carla.Client(host, port)
        client.set_timeout(10.0)
        return client

    def check_non_ego_vehicles_exist(self):
        """
        Check if there are any vehicles other than the ego vehicle.
        """
        world = self.client.get_world()
        actors = world.get_actors().filter('vehicle.*')
        for actor in actors:
            if 'ego' not in actor.attributes.get('role_name', ''):
                return True
        return False

    def clean_non_ego_vehicles(self):
        """
        Remove all vehicles except for the ego vehicle from the scene.
        """
        world = self.client.get_world()
        actors = world.get_actors().filter('vehicle.*')

        for actor in actors:
            if 'ego' not in actor.attributes.get('role_name', ''):
                try:
                    actor.destroy()
                    logging.info(f'Vehicle {actor.id} destroyed successfully')
                except RuntimeError as e:
                    logging.error(f'Failed to destroy vehicle {actor.id}: {str(e)}')

    def spawn_vehicle(self, location=(0, 0, 0), rotation=(0, 0, 0), color=None):
        """
        Spawn a vehicle at a given location and rotation with an optional color.
        """
        if self.vehicle:
            logging.warning('A vehicle is already spawned. Destroying the existing vehicle before spawning a new one.')
            self.destroy_vehicle()

        world = self.client.get_world()
        blueprints = world.get_blueprint_library().filter(self.vehicle_type)
        blueprint = blueprints[0]

        if blueprint.has_attribute('color'):
            if color:
                blueprint.set_attribute('color', color)
            else:
                blueprint.set_attribute('color', blueprint.get_attribute('color').recommended_values[0])

        spawn_point = carla.Transform(carla.Location(x=location[0], y=location[1], z=location[2]),
                                      carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2]))
        
        self.vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if self.vehicle is None:
            logging.error("Failed to spawn vehicle. Please check the spawn point.")
        else:
            logging.info(f"Vehicle {self.vehicle.id} spawned successfully at {location}")

        return self.vehicle

    def destroy_vehicle(self):
        """
        Destroy the currently spawned vehicle.
        """
        if self.vehicle:
            try:
                self.vehicle.destroy()
                logging.info(f'Vehicle {self.vehicle.id} destroyed successfully')
                self.vehicle = None
            except RuntimeError as e:
                logging.error(f'Failed to destroy vehicle {self.vehicle.id}: {str(e)}')
        else:
            logging.info('No vehicle to destroy.')

if __name__ == '__main__':
    vg = VehicleGenerator()
    try:
        if vg.check_non_ego_vehicles_exist():
            vg.clean_non_ego_vehicles()
        vehicle = vg.spawn_vehicle(location=(-54.1, 65.0, 1.0), rotation=(0, 90, 0))
        import time
        time.sleep(10)  # keep the vehicle for 10 seconds
        if vehicle:
            vg.destroy_vehicle()
    except KeyboardInterrupt:
        print('Operation canceled by user.')
        if vg.vehicle:
            vg.destroy_vehicle()
    except Exception as e:
        logging.error('An unexpected error occurred: %s', str(e))
        if vg.vehicle:
            vg.destroy_vehicle()
