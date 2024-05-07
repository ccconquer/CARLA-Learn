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
import argparse
import logging
import queue
import random



# 定义命令行参数
def parser():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', default='127.0.0.1', help='主机的IP地址')
    argparser.add_argument('--port', default=2000, type=int, help='要监听的TCP端口')
    argparser.add_argument('--number-of-vehicles', default=20, type=int, help='车辆的数目')
    argparser.add_argument('--number-of-dangerous-vehicles', default=1, type=int, help='危险车辆的数目')
    argparser.add_argument('--tm-port', default=8000, type=int, help='与交通管理器通信的端口')
    argparser.add_argument('--sync', action='store_true', default=True, help='是否设置为同步模式')
    return argparser.parse_args()


# 传感器回传函数/存储函数
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


def main():
    args = parser()
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    actor_list = []
    sensor_list = []
    vehicle_id_list = []
    sensor_queue = queue.Queue()
    synchronous_master = False

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.get_world()
        # world默认设置为异步模式
        original_settings = world.get_settings()

        # 设置交通管理器Traffic Manager
        traffic_manager = client.get_trafficmanager(args.tm_port)
        # 设置车的整体行为模式(车距、速度等)
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.global_percentage_speed_difference(80)
        
        blueprint_library = world.get_blueprint_library()
        
        # 若client为同步模式，TM也要设置为同步模式
        if args.sync:
            settings = world.get_settings()
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

        # 将车辆分配给TM管理
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        batch = []

        # 车的蓝图
        blueprint_vehicle = blueprint_library.filter('vehicle.*')
        blueprint_vehicle = sorted(blueprint_vehicle, key=lambda bp: bp.id)

        # 车的随机生成点
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)
        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        if args.number_of_vehicles >= number_of_spawn_points:
            msg ='There are %d vehicles, but the number of spawn points are %d not enough.'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points - 1
        
        # 开始生成车辆
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

        
        # 执行提前设置好的指令(上述batch)并检查有没有成功生成每辆车
        for (i, response) in enumerate(client.apply_batch_sync(batch, synchronous_master)):
            if response.error:
                logging.error(response.error)
            else:
                print('Future Actor', response.actor_id)
                vehicle_id_list.append(response.actor_id)
            
        actor_list = world.get_actors().filter('vehicle.*')
        

        # 由于TM是同步模式需要更新确保客户端接收到新创建的车辆
        if args.sync and synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()    
        

        # 将一部分车视为危险车辆
        for i in range(args.number_of_dangerous_vehicles):
            dangerous_vehicle = actor_list[i]
            traffic_manager.ignore_lights_percentage(dangerous_vehicle, 100)
            traffic_manager.distance_to_leading_vehicle(dangerous_vehicle, 0)
            traffic_manager.vehicle_percentage_speed_difference(dangerous_vehicle, -50)
        print('%d dangerous of the %d vehicles' %(args.number_of_dangerous_vehicles, args.number_of_vehicles))

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

        

        while True:
            # client是同步模式需要更新
            if args.sync and synchronous_master:
                world.tick()
                try:
                    # 提取传感器队列queue
                    for i in range(len(sensor_list)):
                        s_frame = sensor_queue.get(True, 1)
                        print('Frame: %d, Sensor: %s.' %(s_frame[0], s_frame[1]))
                except queue.Empty:
                    print('some of the sensor data has missed.')
            else:
                world.wait_for_tick()
            
            # 设置观察者视角
            spectator = world.get_spectator()
            current_transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(carla.Location(z=50)+current_transform.location, carla.Rotation(pitch=-90)))


    finally:
        # 切换回异步模式
        world.apply_settings(original_settings)
        # 销毁一切
        print('Destroy sensors and actors.')
        for sensor in sensor_list:
            sensor.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_id_list])
        print('Done.')



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Exited by user.')