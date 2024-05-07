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
import queue

# 传感器回传数据函数
def sensor_callback(sensor_data, sensor_queue, sensor_name):
    if 'lidar' in sensor_name:
        sensor_data.save_to_disk(os.path.join('../outputs/output_synchronized', '%06d.ply' % sensor_data.frame))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join('../outputs/output_synchronized', '%06d.png' % sensor_data.frame))
    sensor_queue.put((sensor_data.frame, sensor_name))


def main():
    try:
        actor_list = []
        sensor_list = []

        client = carla.Client('127.0.0.1', 2000)
        world = client.get_world()
        
        # 仿真里没有'秒'的概念, 而是'time-step', 一个time-step表示仿真世界进行了一次更新
        # Variable time-step 仿真步长不定, 会尽可能快速运行(默认模式)
        settings = world.get_settings()
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        # Fixed time-step 每次time-step所需要的时间固定
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # 默认为异步模式(client可能会跟不上server导致掉帧现象)+variable time-step, 同步模式(server会等待client)对应fixed time-steps
        # 设置为同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        ego_vehicle_bp.set_attribute('color', '0,1,0')
        ego_vehicle_transform= random.choice(world.get_map().get_spawn_points())
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_vehicle_transform)
        # 同步模式用autopilot时, 依附于开启同步模式的traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        ego_vehicle.set_autopilot(True, 8000)
        actor_list.append(ego_vehicle)

        # 放置传感器camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5,z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        # 回传数据
        sensor_queue = queue.Queue()
        camera.listen(lambda image: sensor_callback(image, sensor_queue, 'camera'))
        sensor_list.append(camera)

        # 放置传感器lidar
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        lidar_transform = carla.Transform(carla.Location(x=0,y=0,z=2), carla.Rotation(pitch=0, yaw=0,roll=0))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        # 回传数据
        lidar.listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, 'lidar'))
        sensor_list.append(lidar)

        while True:
            # 出现于同步模式, 让simulation刷新一次
            world.tick() 
            # queue.get在队列内容都提取出来之前，会阻止其他进程
            data = sensor_queue.get(block=True) 

    finally:
        # 在client完成任务准备销毁时要改为异步模式
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta
