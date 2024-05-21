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
sys.path.append('C:\carla\PythonAPI\mytest\202405')
from agents.navigation.behavior_agent import BehaviorAgent


class intersection_planner:
    ## 初始化函数
    def __init__(self, client, world):
        self.client = client
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.actor_list = []


    ## 设置同步模式函数
    def sync_mode(self):
        orignal_settings  = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        return orignal_settings
    

    ## 找到特定坐标
    def draw_waypoints(self, waypoints, road_id=None, life_time=50):
        for waypoint in waypoints:
            if(waypoint.road_id == road_id):
                 self.world.debug.draw_string(waypoint.transform.location, '0', draw_shadow = False, color = carla.Color(0,255,0), life_time = life_time, persistent_lines = True)


    ## 创建自车
    def ego_vehicle_create(self):
        ego_vehicle_bp = self.blueprint_library.find('vehicle.audi.a2')
        ego_vehicle_bp.set_attribute('color', '255,0,0')

        waypoints = self.world.get_map().generate_waypoints(distance = 1)
        filter_waypoints = []
        for waypoint in waypoints:
            if(waypoint.road_id == 0):
                filter_waypoints.append(waypoint)
        self.ego_vehicle_spawn_point = filter_waypoints[20].transform
        self.ego_vehicle_spawn_point.location.z += 2
        self.ego_vehicle_spawn_point.rotation.yaw += 180
        
        self.ego_vehicle = self.world.spawn_actor(ego_vehicle_bp, self.ego_vehicle_spawn_point)
        self.actor_list.append(self.ego_vehicle)
        return self.ego_vehicle
    

    ## 创建他车
    def test_vehicle_create(self):
        self.test_vehicle_list = []
        vehicle_bp = self.blueprint_library.find('vehicle.audi.a2')
        vehicle_bp.set_attribute('color', '0,0,255')
    
        waypoints = self.world.get_map().generate_waypoints(distance = 1)
        self.filter_waypoints_lf = []
        self.filter_waypoints_fr = []
        self.filter_waypoints_rt = []
        for waypoint in waypoints:
            if (waypoint.road_id == 16):
                self.filter_waypoints_lf.append(waypoint)
            if (waypoint.road_id == 51):
                self.filter_waypoints_fr.append(waypoint)
            if (waypoint.road_id ==15):
                self.filter_waypoints_rt.append(waypoint)

        vehicle_left_spawn_point = self.filter_waypoints_lf[25].transform
        vehicle_left_spawn_point.location.z += 2
        vehicle_front_spawn_point = self.filter_waypoints_fr[20].transform
        vehicle_front_spawn_point.location.z += 2
        vehicle_front_spawn_point.rotation.yaw += 180
        vehicle_right_spawn_point = self.filter_waypoints_rt[25].transform
        vehicle_right_spawn_point.location.z += 2                
         
        vehicle_left = self.world.spawn_actor(vehicle_bp, vehicle_left_spawn_point)
        vehicle_front = self.world.spawn_actor(vehicle_bp, vehicle_front_spawn_point)
        vehicle_right = self.world.spawn_actor(vehicle_bp, vehicle_right_spawn_point)
        self.actor_list.append(vehicle_left)
        self.actor_list.append(vehicle_front)
        self.actor_list.append(vehicle_right)
        self.test_vehicle_list.append(vehicle_left)
        self.test_vehicle_list.append(vehicle_front)
        self.test_vehicle_list.append(vehicle_right)
        return self.test_vehicle_list


    ## 自车BehaviorAgent初始化, 全局路径规划
    def ego_vehicle_agent_init(self):
        # BehaviorAgent初始化
        self.ego_agent = BehaviorAgent(self.ego_vehicle, ignore_traffic_light=True, behavior='normal')
        
        # 生成全局路径
        destination = self.filter_waypoints_fr[45].transform.location
        print(destination)
        print(self.ego_vehicle.get_location())
        self.ego_agent.set_destination(self.ego_vehicle_spawn_point.location, destination, clean=True)
        return self.ego_agent


    ## 他车BehaviorAgent初始化, 全局路径规划
    def test_vehicle_agent_init(self):
        # BehaviorAgent初始化
        self.test_vehicle_agent_list = []
        for i in range(len(self.test_vehicle_list)):
            print(len(self.test_vehicle_list))
            
            self.test_vehicle_agent_list.append(BehaviorAgent(self.test_vehicle_list[i], ignore_traffic_light=True, behavior='normal'))
        
        # 生成全局路径
        destination_lf = self.filter_waypoints_rt[45].transform.location
        destination_ft = self.filter_waypoints_lf[45].transform.location
        destination_rt = self.filter_waypoints_rt[55].transform.location
        self.test_destination_list = [destination_lf, destination_ft, destination_rt]

        for i in range(len(self.test_vehicle_list)):
            self.test_vehicle_agent_list[i].set_destination(self.test_vehicle_list[i].get_location(), self.test_destination_list[i], clean=True)
        return self.test_vehicle_agent_list
    

    ## 自车单步行为规划、轨迹规划和控制(暂时用carla自带的)
    def ego_vehicle_plannner(self):
        control = self.ego_agent.run_step(debug=True)
        self.ego_vehicle.apply_control(control)
        

    ## 他车单步行为规划、轨迹规划和控制(暂时用carla自带的)
    def test_vehicle_planner(self):
        for i in range(len(self.test_vehicle_list)):
            control = self.test_vehicle_agent_list[i].run_step(debug=True)
            self.test_vehicle_list[i].apply_control(control)


    ## 设置跟踪管理器(俯视视角, 以自车为中心)
    def spectator_watch(self):
        spectator = self.world.get_spectator()
        transform = self.ego_vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location+carla.Location(z=80), carla.Rotation(yaw=180,pitch=-90)))


    ## 销毁一切
    def destroy(self):
        print('Destroy actors.')
        for actor in self.actor_list:
            if actor.is_alive:
                actor.destroy()
        print('Done.')

    


def main():

    test_vehicle_list = []
    test_vehicle_agent_list = []

    try:                       
        # 创建客户端并与world建立联系
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        world.set_weather(carla.WeatherParameters.ClearSunset)

        test = intersection_planner(client, world)
        
        # 设置为同步模式   
        original_settings = test.sync_mode()  
      
        # 创建actor
        ego_vehicle = test.ego_vehicle_create()
        test_vehicle_list = test.test_vehicle_create()
        
        # 初始化BehaviorAgent
        ego_vehicle_agent = test.ego_vehicle_agent_init()
        test_vehicle_agent_list = test.test_vehicle_agent_init()
       
        while True:
            world.tick()
            
            # 设置跟踪观察器
            test.spectator_watch()

            ## 单步规划控制, 写在while True可以不断进行
            # 更新车辆信息(包括速度、交通灯、waypoint是否到达目标点→判断自车即可)
            ego_vehicle_agent.update_information(ego_vehicle)

            for i in range(len(test_vehicle_agent_list)):
                test_vehicle_agent_list[i].update_information(test_vehicle_list[i])
            if len(ego_vehicle_agent._local_planner.waypoints_queue)<1:
                print('Ego Vehicle has arrived at the target point!')
                break

            # 设置局部规划的目标速度(这里设为道路限速)
            #speed_limit_ego = ego_vehicle.get_speed_limit()
            #ego_vehicle_agent.get_local_planner().set_speed(speed_limit_ego)
            ego_vehicle_agent.get_local_planner().set_speed(5)

            speed_limit_test_list = []
            for i in range(len(test_vehicle_agent_list)):
                speed_limit_test_list.append(test_vehicle_list[i].get_speed_limit())            
                #test_vehicle_agent_list[i].get_local_planner().set_speed(speed_limit_test_list[i])
                test_vehicle_agent_list[i].get_local_planner().set_speed(5)
            
            # 单步规划控制
            test.ego_vehicle_plannner()
         #   test.test_vehicle_planner()


    finally:
        # 恢复为异步模式
        world.apply_settings(original_settings)
        
        # 销毁一切
        test.destroy()
    
        

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('--Exited by user.')
