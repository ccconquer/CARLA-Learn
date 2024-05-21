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


import os
import random
import carla
sys.path.append('C:\carla\PythonAPI\mytest\202405')

from agents.navigation.behavior_agent import BehaviorAgent


'''
Carla行为规划: 大致分为全局路线规划、行为规划、轨迹规划和底层控制四部分
示例automatic_control.py: 随机生成一辆小车、随机给目的地,让汽车自己在限速内安全丝滑到达终点
'''
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    

    try:

        origin_settings = world.get_settings()
         
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # 在地图上随机生成点
        spawn_points = world.get_map().get_spawn_points()
        # 随机选择点作为起点
        spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()

        ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

        # 需要world.tick()使得client更新车辆生成位置
        world.tick()


        '''
        Behavior Agent初始化(像vehicle的大脑下达指令)
        不像Traffic Manager用C++封装好, 由python构成
        '''
        # 构建Behavior Agent (需要两个输入: 生成的vehicle和驾驶风格)
        agent = BehaviorAgent(vehicle, behavior='normal')

        ##### 全局路径规划  
        # 设置终点
        destination_points = world.get_map().get_spawn_points()
        random.shuffle(destination_points)
        # 避免起点和终点重合
        if destination_points[0].location != agent.vehicle.get_location():
            destination = destination_points[0]
        else:
            destination = destination_points[1]
        # 全局路径规划(包含4步: GlobalRoutePlanner初始化、CarlaMapTopology提取、Graph建立、全局导航路径生成)
        # 局部路径规划(只关注几秒的未来路线)
        agent.set_destination(agent.vehicle.get_location(), destination.location, clean=True)
        
        # runtime部分
        while True:
            # 仿真世界运行一个步长
            world.tick()

            # 更新汽车的实时信息(包括速度信息更新、incoming waypoint更新、信号灯指示牌信息更新)
            agent.update_information(vehicle)
            if len(agent._local_planner.waypoints_queue)<1:
                print('======== Success, Arrivied at Target Point!')
                break
                
            # 观察者视角(鸟瞰视角)
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                    carla.Rotation(pitch=-90)))
            
            speed_limit = vehicle.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)
            

            '''
            行为规划(大致分为5步: 针对交通信号灯的行为规划、针对行人的行为规划、针对其他车辆的行为规划、针对交叉口的行为规划、正常驾驶的行为规划)
            '''
            # 核心步骤, 更新信息之后进行新一步规划并产生相应的控制命令
            control = agent.run_step(debug=True)
            # 执行控制命令
            vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' --Exited by user.')
