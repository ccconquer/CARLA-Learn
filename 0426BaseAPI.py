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
    actor_list = []
    sensor_list = []
    try:
        # 构建client并和world建立联系
        client = carla.Client('127.0.0.1', 2000)
        world = client.get_world()
        # 生成actor如汽车
        blueprint_library = world.get_blueprint_library()
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        ego_vehicle_bp.set_attribute('color', '0,1,0')
        #ego_vehicle_transform = carla.Transform(carla.Location(x=23,y=25,z=0))
        ego_vehicle_transform= random.choice(world.get_map().get_spawn_points())
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_vehicle_transform)
        # 将该车设置成自动驾驶模式
        ego_vehicle.set_autopilot(True)
        actor_list.append(ego_vehicle)
    
        # 在汽车上加传感器, 如相机和雷达
        # 相机
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5,z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        # 回传数据
        camera.listen(lambda image: image.save_to_disk(os.path.join('C:\carla\PythonAPI\mytest\0426\camera_data','%06d.png' %image.frame)))
        sensor_list.append(camera)
        # Lidar
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        lidar_transform = carla.Transform(carla.Location(x=0,y=0,z=2), carla.Rotation(pitch=0, yaw=0,roll=0))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        # 回传数据
        lidar.listen(lambda point_cloud: point_cloud.save_to_disk(os.path.join('C:\carla\PythonAPI\mytest\0426\lidar_data', '%06d.ply' %point_cloud.frame)))
        sensor_list.append(lidar)

        # 观察者视角
        while True:
            spectator = world.get_spectator()
            car_transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(car_transform.location+carla.Location(z=30), carla.Rotation(pitch=-90)))
            
    finally:
        print('destroy actors and sensors')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list])
        for sensor in sensor_list:
             sensor.destroy()
        print('done')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Exited by user!')
    
