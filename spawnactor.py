# import carla前必备操作
import glob
import sys
import os

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla
import random

def main():
    try:
        # 创建客户端并与world建立联系
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        world.set_weather(carla.WeatherParameters.ClearSunset)
        blueprint_library = world.get_blueprint_library()

        vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        vehicle_bp.set_attribute('color', '255,0,0')
        vehicle_spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle_transform = carla.Transform(carla.Location(x=201.90,y=-322.96,z=0))
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        #vehicle = world.spawn_actor(vehicle_bp, vehicle_spawn_point)
        vehicle_list = []
        vehicle_list.append(vehicle)        
    
    finally:
        print('Destroy actors.')
        for i in vehicle_list:
             i.destroy()
        print('Done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('--Exited by user.')
