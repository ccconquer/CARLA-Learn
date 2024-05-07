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
import logging
import argparse
import queue



'''
定义命令行参数
'''
def parser():
    # description描述命令行的功能和目的
    argparser = argparse.ArgumentParser(description=__doc__)
    # 命名' ', 默认值default, 参数值的类型type, 简单描述help
    argparser.add_argument('--host', default='127.0.0.1', help='主机的IP地址')
    argparser.add_argument('--port', default=2000, type=int, help='要监听的TCP端口')
    # 引用的时候是下划线number_of_vehicles
    argparser.add_argument('--number-of-vehicles', default=20, type=int, help='车辆的数目')
    argparser.add_argument('--number-of-dangerous-vehicles', default=1, type=int, help='危险车辆的数目')
    argparser.add_argument('--tm-port', default=8000, type=int, help='与交通管理器通信的端口')
    # action='store_true'表示如果设置了该参数则初始默认值为True 和 default=True含义一样？(不一样！还是要有default)
    argparser.add_argument('--sync', action='store_true', default=True, help='是否设置为同步模式')
    return argparser.parse_args()



'''
传感器回传函数/存储函数
'''
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
Traffic Manager如何管理自车以外的其他车辆的行为, 并且模拟真实的交通环境(纯C++构造包装被python调用)
作用: 帮助用户集体管理一个车群, 如设定所有车的限速和最小安全车距
注意: 同步模式下设置自动驾驶模式(autopilot)的车必须依附于设置同步模式的traffic manager才能运行
多个Traffic Manager构建原则: (1)可以在一个或多个client里建立多个TM (2)只有一个TM能设置为同步模式 
'''
def main():
    args = parser()
    # 配置日志系统基础设置, 使其以指定格式(级别名：消息内容)记录级别在INFO以上的日志消息
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    actor_list = []
    sensor_list = []
    vehicle_id_list = []
    sensor_queue = queue.Queue()
    synchronous_master = False
    
    try:
        # 创建client客户端并和world服务器端建立联系
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.get_world()
        # world的初始模式(默认是异步模式)
        original_settings = world.get_settings()
        # 客户端创建traffic manager, 默认从8000接口接入
        traffic_manager = client.get_trafficmanager(args.tm_port)
        # 得到world的所有蓝图及所有车辆蓝图
        blueprint_library = world.get_blueprint_library()
        

        '''
        设置traffic manager管辖车群的整体行为模式
        '''
        # 每一辆车都要和前车保持至少3m的车距
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        # 关于hybrid physics mode一旦该模式被设为True, 则只有在ego-vehicle附近范围的车辆会开启物理特性(考虑真实的物理限制, 如轮胎摩擦、道路曲率等), 使得附近交通更具有真实性
        traffic_manager.set_hybrid_physics_mode(True)
        # 每一辆车的默认限速为30km/h, 设置百分比改变速度, 80%的限速为24=30×80%, -80%的限速为54=30×180%
        traffic_manager.global_percentage_speed_difference(80)
        

        '''
        如果客户端为同步模式, 设置Traffic Manager也为同步模式
        '''
        if args.sync:
            settings = world.get_settings()
            # 设置交通管理器为同步模式
            traffic_manager.set_synchronous_mode(True)
            # 设置客户端为同步模式和fixed time step, 使传感器的记录不掉帧
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)

        
        '''
        将车辆分配给Traffic Manager管理
        '''
        # 得到车辆的所有蓝图
        blueprint_vehicle = blueprint_library.filter('vehicle.*')
        # 根据蓝图的id属性进行排序
        blueprint_vehicle = sorted(blueprint_vehicle, key=lambda bp: bp.id)

        # 在地图上得到随机点, 用作车辆的未来初始位置
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)
        if args.number_of_vehicles < number_of_spawn_points:  # 没加=是因为留一个点给自车
            random.shuffle(spawn_points)
        if args.number_of_vehicles >= number_of_spawn_points:
            msg = '要生成%d辆车, 但只有%d个随机点'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points - 1

        # command一次性创建所有车辆并将车辆分配给TM
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        batch = []
        
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprint_vehicle)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            
            blueprint.set_attribute('role_name', 'autopilot')
            
            # 先按照蓝图和位置创建汽车, 再把车辆分配给TM采取自动驾驶模式
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))


        # 执行提前设置好的指令(由于Traffic Manager是同步模式, 故使用apply_batch_sync执行命令)
        # 在指令没有执行完之前, main thread会被block住/ 对每个指令都会有response返回, 通过response可以得知命令是否下达成功并返回driver_id
        for (i, response) in enumerate(client.apply_batch_sync(batch, synchronous_master)):
            if response.error:
                logging.error(response.error)
            else:
                print('Future Actor', response.actor_id)
                vehicle_id_list.append(response.actor_id)

        actor_list = world.get_actors().filter('vehicle.*')

        # tick以确保客户端接收到新创建的车辆(由于Traffic Manager是同步模式)
        if not args.sync or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # 将生成车辆的一部分视为危险车辆(不看交通灯、不保持车距、开得飞快等), 更符合现实情况
        for i in range(args.number_of_dangerous_vehicles):
            dangerous_vehicle = actor_list[i]
            traffic_manager.ignore_lights_percentage(dangerous_vehicle, 100)
            traffic_manager.distance_to_leading_vehicle(dangerous_vehicle, 0)
            traffic_manager.vehicle_percentage_speed_difference(dangerous_vehicle, -50)

        print('there are %d vehicles, and %d of them are dangerous cars.' % (len(actor_list), args.number_of_dangerous_vehicles))


        '''
        生成自车
        '''
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        ego_vehicle_bp.set_attribute('color', '0,255,0')
        ego_vehicle_bp.set_attribute('role_name', 'hero')
        ego_vehicle_transform = spawn_points[len(vehicle_id_list)]
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_vehicle_transform)
        # 自车也需要被TM管理？
        ego_vehicle.set_autopilot(True, args.tm_port)
        vehicle_id_list.append(ego_vehicle.id)


        '''
        生成传感器(例如相机、雷达)
        '''
        # 生成相机
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_tranform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_tranform, attach_to=ego_vehicle)
        camera.listen(lambda image: sensor_callback(image, sensor_queue, 'camera'))
        sensor_list.append(camera)
        
        # 生成雷达(有很多属性需要设置)
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        lidar_transform = carla.Transform(carla.Location(z=2))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar.listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, 'lidar'))
        sensor_list.append(lidar)
 
        
        '''
        client是同步模式需要更新tick
        提取传感器队列sensor_queue
        不需要观察者视角吗？
        '''
        while True:
            if args.sync and synchronous_master:
                world.tick()
                try:
                    # 提取sensor_queue中的数据, 在队列内容提取并删除之前阻止(True)其他进程, 1表示1秒等待
                    for i in range(len(sensor_list)):
                        s_frame = sensor_queue.get(True, 1)
                        print('Frame: %d, Sensor: %s' %(s_frame[0], s_frame[1]))
                except queue.Empty:
                    print('Some of the sensor data has missed.')
            else:
                world.wait_for_tick()

            # 设置观察者视角使自车一直在视野内
            spectator = world.get_spectator()
            current_transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(carla.Location(z=60)+current_transform.location, carla.Rotation(pitch=-90)))

            

    finally:
        # 切换回异步模式
        world.apply_settings(original_settings)
        
        print('Destroy actors and sensors.')
        client.apply_batch([carla.command.DestroyActor(actor) for actor in vehicle_id_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('Done.')        
        



 
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Exited by user.')









 