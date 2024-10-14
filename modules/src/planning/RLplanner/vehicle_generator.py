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
        self.vehicles = []  # 存储车辆的列表（车辆ID）

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

    def spawn_vehicle(self, location=(0, 0, 0), rotation=(0, 0, 0), color=None):
        """
        Spawn a vehicle at a given location and rotation with an optional color.
        """
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
        vehicle = world.spawn_actor(blueprint, spawn_point)
        if vehicle:
            self.vehicles.append(vehicle.id)  # 将车辆 ID 添加到列表中
            logging.info('Spawned vehicle: %s at %s', vehicle.type_id, spawn_point.location)
            return vehicle
        else:
            logging.error('Failed to spawn vehicle')
            return None

    def destroy_vehicles(self):
        """
        Destroy all vehicles using batch processing.
        """
        if self.vehicles:
            commands = [carla.command.DestroyActor(vid) for vid in self.vehicles]
            responses = self.client.apply_batch_sync(commands, True)
            for response in responses:
                if response.error:
                    logging.error(f'Failed to destroy vehicle {response.actor_id}: {response.error}')
                else:
                    logging.info(f'Vehicle {response.actor_id} destroyed successfully')
            self.vehicles = []  # 清空列表
        else:
            logging.info('No vehicles to destroy.')

if __name__ == '__main__':
    vg = VehicleGenerator()
    try:
        vehicle = vg.spawn_vehicle(location=(-54.1, 65.0, 1.0), rotation=(0, 90, 0))
        import time
        time.sleep(10)  # keep the vehicle for 10 seconds
        if vehicle:
            vg.destroy_vehicles()
    except KeyboardInterrupt:
        print('Operation canceled.')
    except Exception as e:
        logging.error('An unexpected error occurred: %s', str(e))
