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
import queue
import random

# 传感器回传函数
def sensor_callback(sensor_data, sensor_queue, sensor_name):
    output_path1 = 'C:\carla\PythonAPI\mytest\lidar_data'
    output_path2 = 'C:\carla\PythonAPI\mytest\camera_data'
    if not os.path.exists(output_path1):
        os.makedirs(output_path1)
    if not os.path.exists(output_path2):
        os.makedirs(output_path2)
    if 'lidar' in sensor_name:
        sensor_data.save_to_disk(os.path.join(output_path1, '%06d.ply' %sensor_data.frame))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join(output_path2, '%06d.png' %sensor_data.frame))
    sensor_queue.put((sensor_data.frame, sensor_name))


'''
将client设置成同步模式以便收集相机等传感器数据
'''
def main():
    actor_list = []
    sensor_list = []
    sensor_queue = queue.Queue()

    try:
        # 创建client并和world建立联系
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        
        # 设置为同步模式+fixed step-time防止收集数据时掉帧
        # 最初默认设置为异步模式
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # 生成车辆
        blueprint_library = world.get_blueprint_library()
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        ego_vehicle_transform = random.choice(world.get_map().get_spawn_points())
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_vehicle_transform)
        # 同步模式要用系统的自动驾驶，必须设置同步的交通管理器traffic manager
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        ego_vehicle.set_autopilot(True)
        actor_list.append(ego_vehicle)

        # 生成camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        camera.listen(lambda image: sensor_callback(image, sensor_queue, 'camera'))
        sensor_list.append(camera)

        # 生成lidar
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        lidar_transform = carla.Transform(carla.Location(z=2))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar.listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, 'lidar'))
        sensor_list.append(lidar)

        while True:
            # 更新一次simulation
            world.tick()
            # 对queue中的数据采用block=True提取，queue.get能够在队列内容都提取出来之前阻止其他进程 queue.get(block=True)
            try:
                for i in range(0, len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1)
                    print('Frame: %d, Sensor: %s' %(s_frame[0], s_frame[1]))
            except queue.Empty:
                print('Some of the sensor data is missed.')
            
            # 设置观察者视角使得小车一直在视角内
            spectator = world.get_spectator()
            current_transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(carla.Location(z=50)+current_transform.location, carla.Rotation(pitch=-90)))
    
    finally:
        # 在client完成任务准备销毁时需要设置回异步模式
        world.apply_settings(original_settings)

        # 销毁车辆和传感器
        print('Destroy actors and sensors:')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('Exited by user.')

