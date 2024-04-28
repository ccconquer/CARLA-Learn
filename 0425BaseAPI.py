# import carla前必备操作？
import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

# 其他需要的库
import random

def main():
    actor_list = []
    sensor_list = []
    try:
        '''
        创建client, 并和仿真世界建立联系
        '''
        # 用户通过Client载体、Python API与仿真环境交互
        # 创建client, localhost为主机号, 2000端口
        # 设置timeout防止连接时间过久
        client = carla.Client('127.0.0.1', 2000)   
        #client.set_timeout(2.0)
        # 通过构建的Client获取仿真世界world(正在运行)
        world = client.get_world()
        # 改变world中的天气
        # weather = carla.WeatherParameters(cloudiness=10.0, precipitation=10.0, fog_density=10.0)
        # world.set_weather(weather)
        
         
        '''
        在world中放置、生成actor, actor包括汽车、行人、传感器等
        '''
        # 先拿到该仿真世界的所有蓝图blueprint, 再从蓝图中找到自己需要的actor蓝图
        blueprint_library = world.get_blueprint_library()
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        # 可以修改该actor的属性
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # 给定该actor的初始位置(随机或指定)
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        #start_point = carla.Transform(carla.Location(x=25.0, y=4.0, z=11.0), carla.Rotation(pitch=0, yaw=180, roll=0))
        # 在该位置生成汽车
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, vehicle_transform)
        # 设置成自动驾驶模式或其他方法？
        ego_vehicle.set_autopilot(True)
        actor_list.append(ego_vehicle)

        #在车上放置各种sensor(以RGB相机和Lidar为例)
        # 先创建传感器的蓝图, 再定义位置(相对于车辆中心点的位置), 选择安装的汽车, 再定义callback function(listen)
        # RGB相机
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5,z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        # 将传回来的数据存在硬盘里(简单方式)
        camera.listen(lambda image: image.save_to_disk(os.path.join('C:\carla\PythonAPI\mytest\0426\camera_data', '%06d.png' % image.frame)))
        sensor_list.append(camera)

        # Lidar
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        lidar_transform = carla.Transform(carla.Location(x=0,y=0,z=2), carla.Rotation(pitch=0, yaw=0,roll=0))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        # 将传回来的数据存在硬盘里(简单方式)
        lidar.listen(lambda point_cloud: point_cloud.save_to_disk(os.path.join('C:\carla\PythonAPI\mytest\0426\lidar_data', '%06d.ply' % point_cloud.frame)))
        sensor_list.append(lidar)
        

        '''
        观察者放置, 使车始终在视野中(俯视角度)
        '''
        # 无限循环确保spector一直跟随
        while True: 
            spectator = world.get_spectator()
            # 自车当前的位置和朝向
            transform = ego_vehicle.get_transform()
            # z=20将spector上移20，-90度将方向调整为朝下(俯视)
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20), carla.Rotation(pitch=-90)))
   
    # 无论try是否执行都会执行fianlly后的语句
    # 销毁，否则会一直存在world
    finally:
        print('destroying actors and sensors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list]) 
        for sensor in sensor_list:
            sensor.destroy()
        print('done')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:  
        print('Exited by user')










