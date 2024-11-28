#!/usr/bin/env python

import glob
import os
import sys
import time
import carla
import argparse
import logging
from numpy import random
import threading

class VehicleGenerator():
    def __init__(self):
        self.host = '127.0.0.1'
        self.port = 2000
        self.number_of_vehicles = 2
        self.safe = False
        self.filterv = 'vehicle.tesla.model3'
        self.generationv = 'All'
        self.generationw = '2'
        self.tm_port = 8000
        self.asynch = False
        self.hybrid = False
        self.seed = None  # None 表示未设置
        self.seedw = 0
        self.car_lights_on = False
        self.hero = False
        self.respawn = False
        self.no_rendering = False
        self.notrain = False

        self.vehicles_list = []
        self.client = self.init_carla_client()
        self.running = True

        # Start the simulation in a separate thread
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()

    def init_carla_client(self):
        try:
            sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
                sys.version_info.major,
                sys.version_info.minor,
                'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
        except IndexError:
            pass

        client = carla.Client(self.host, self.port)
        client.set_timeout(10.0)
        self.synchronous_master = False
        random.seed(self.seed if self.seed is not None else int(time.time()))

        return client
    
    def check_existing_vehicles(self):
        existing_vehicles = self.world.get_actors().filter('vehicle.tesla.model3')
        if existing_vehicles:
            print(f'Found {len(existing_vehicles)} existing vehicles. Destroying them first...')
            self.client.apply_batch([carla.command.DestroyActor(x.id) for x in existing_vehicles])
    
    def get_actor_blueprints(self, world, filter, generation):
        bps = world.get_blueprint_library().filter(filter)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        try:
            int_generation = int(generation)
            # Check if generation is in available generations
            if int_generation in [1, 2, 3]:
                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                return bps
            else:
                print("   Warning! Actor Generation is not valid. No actor will be spawned.")
                return []
        except:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
        
    def get_spawn_points(self):
        # 获取地图的所有生成点
        spawn_points = self.world.get_map().get_spawn_points()

        # 创建一个新的列表，用于保存符合要求的生成点
        filtered_spawn_points = []

        for point in spawn_points:
            if -55 <= point.location.x <= -53 and -100 <= point.location.y <= 50:
                filtered_spawn_points.append(point)

        return filtered_spawn_points


    def spawn_vehicle(self):
        self.vehicles_list = []
        self.world = self.client.get_world()

        self.check_existing_vehicles()

        self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if self.respawn:
            self.traffic_manager.set_respawn_dormant_vehicles(True)
        if self.hybrid:
            self.traffic_manager.set_hybrid_physics_mode(True)
            self.traffic_manager.set_hybrid_physics_radius(70.0)
        if self.seed is not None:
            self.traffic_manager.set_random_device_seed(self.seed)

        settings = self.world.get_settings()
        if not self.asynch:
            self.traffic_manager.set_synchronous_mode(False)
            if not settings.synchronous_mode:
                self.synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                self.synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if self.no_rendering:
            settings.no_rendering_mode = True
        self.world.apply_settings(settings)

        self.blueprints = self.get_actor_blueprints(self.world, self.filterv, self.generationv)
        if not self.blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")

        if self.safe:
            self.blueprints = [x for x in self.blueprints if x.get_attribute('base_type') == 'car']

        self.blueprints = sorted(self.blueprints, key=lambda bp: bp.id)

        spawn_points = self.get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if self.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif self.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, self.number_of_vehicles, number_of_spawn_points)
            self.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = self.hero
        for n, transform in enumerate(spawn_points):
            if n >= self.number_of_vehicles:
                break
            blueprint = random.choice(self.blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, self.traffic_manager.get_port())))

        for response in self.client.apply_batch_sync(batch, self.synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                self.vehicles_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if self.car_lights_on:
            all_vehicle_actors = self.world.get_actors(self.vehicles_list)
            for actor in all_vehicle_actors:
                self.traffic_manager.update_vehicle_lights(actor, True)

        print('spawned %d vehicles' % (len(self.vehicles_list)))

        # Example of how to use Traffic Manager parameters
        self.traffic_manager.global_percentage_speed_difference(30.0)

        self.run_simulation()


    def run_simulation(self):
        while self.running:
            if not self.asynch and self.synchronous_master:
                self.world.tick()

    def destroy_vehicle(self):
        if not self.asynch and self.synchronous_master:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])
        self.running = False

        time.sleep(0.5)
