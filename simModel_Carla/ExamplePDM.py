from utils.load_config import load_config
from ego_vehicle_planning import LLMEgoPlanner
from simModel_Carla.MPGUI import GUI
from simModel_Carla.Model import Model
from trafficManager.traffic_manager_carla import TrafficManager
import carla
import time
from math import sqrt
import random
from dataclasses import field
from datetime import datetime
import os
from AD_algo.pdm_lite.autopilot import AutoPilot
from scenario_runner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from scenario_runner.srunner.scenariomanager.timer import GameTime
from DataQueue import QuestionAndAnswer
from leaderboard_util import initDataProvider,route_transform,setup_sensors
import argparse
from argparse import RawTextHelpFormatter

if __name__=='__main__':
    description = "limsim evaluation: evaluate your Agent in limsim Carla\n"
    
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--host', default='localhost',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default='3000', help='TCP port to listen to (default: 3000)')
    parser.add_argument('--trafficManagerPort', default='1112',
                        help='Port to use for the TrafficManager (default: 1112)')
    
    parser.add_argument('--random_seed', default='1121102',
                        help='scene_rollout_randomseed')
    
    parser.add_argument('--database', default=None,
                    help='simulation data file name')
    parser.add_argument('--config_path', default='./simModel_Carla/exp_config/long_term_config.yaml',
                    help='path to the configuration file')
    arguments = parser.parse_args()
    
    config_name = arguments.config_path  # 使用传入的配置文件路径
    random.seed(int(arguments.random_seed))

    stringTimestamp = datetime.strftime(datetime.now(), '%Y-%m-%d_%H-%M-%S')
    database=f'results/{arguments.database}/'+arguments.database+'.db' if arguments.database else 'results/' + stringTimestamp + '.db'
    total_start_time = time.time()

    model:Model=Model(cfgFile=config_name,dataBase=database,port=int(arguments.port),tm_port=int(arguments.trafficManagerPort))
    model.cfg['carlaImage']=False
    
    planner = TrafficManager(model)
    #For npc vehicle avoid ego car
    planner.config["EGO_PLANNER"] = False
    planner.config['EGO_CONTROL'] = True

    model.start()
    model.runAutoPilot()

    #DATAPROVIDER Initiate
    initDataProvider(model)#CarlaDataProvider的记录actor只能记录调用他的方法生成的actor，导致无法与ego连接，新写一个函数
    GameTime.restart()
    #PDM settup
    pdm=AutoPilot(os.path.abspath(__file__),model.cfg['map_name'])
    gps_route,route=route_transform(model.roadgraph,model.ego)
    

    route_length = 0.0 
    for idx,wp in enumerate(route):
        if idx!=len(route)-1:
            route_length+=wp[0].location.distance(route[idx+1][0].location)
    model.simDescriptionCommit(route_length)
    
    # world=model.world
    # sum_dist=0
    # for j,wp in enumerate(route):
    #     world.debug.draw_point(wp[0].location, color=carla.Color(r=0, g=0, b=255), life_time=5000, size=0.1)
    #     world.debug.draw_string(wp[0].location, str(j), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)
    #     sum_dist+= 0 if j==0 else route[j][0].location.distance(route[j-1][0].location)
    # world.debug.draw_point(route[-1][0].location, color=carla.Color(r=255, g=0, b=0), life_time=5000, size=5)
    
    # settings = world.get_settings()
    # settings.synchronous_mode = False
    # settings.fixed_delta_seconds = None
    # world.apply_settings(settings)
    # # print(model.ego.route)
    # # print(f'distance:{sum_dist}')
    
    
    # # # for item in model.roadgraph.Edges.items():
    # # #     model.world.debug.draw_string(item[1].last_segment[0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)
    # breakpoint()    
    
    pdm.set_global_plan(gps_route,route)#将model中的全局路径传递给pdm
    pdm.setup(os.path.abspath(__file__),model.cfg['map_name'])
    setup_sensors(pdm,model.ego.actor)

    

    gui = GUI(model)
    gui.start()

    while not model.tpEnd:
        model.moveStep()
        if model.shouldUpdate():
            roadgraph, vehicles = model.exportSce()
            try:
                trajectories = planner.plan(
                    model.timeStep * 0.1, roadgraph, vehicles, model.ego.behaviour, other_plan=True
                )
                if model.ego_id in trajectories:
                    print('ego planned',end=' ')
                del trajectories[model.ego_id]
            except:
                trajectories=dict()
            model.setTrajectories(trajectories)

            print(model.ego.lane_id)
            print(model.ego.behaviour)

            world=model.world
            for veh in model.vehicles:
                if veh.trajectory:
                    for state in veh.trajectory.states:
                        world.debug.draw_point(carla.Location(x=state.x,y=state.y,z=0.5),color=carla.Color(r=0, g=0, b=255), life_time=1, size=0.1)
        
        #PDM model
        timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp
        GameTime.on_carla_tick(timestamp)
        CarlaDataProvider.on_carla_tick()#主要更新内部记录的状态数据
        # 在pdm()调用前记录开始时间
        pdm_start_time = time.time()
        ego_action = pdm()
        # 在pdm()调用后记录结束时间  
        pdm_end_time = time.time()
        pdm_inference_time = pdm_end_time - pdm_start_time

        # 构造QuestionAndAnswer对象
        qa = QuestionAndAnswer('', '', '', '', f'PDM: {pdm_inference_time:.4f}s', 
                               0, 0, 0, pdm_inference_time, 0)
        model.putQA(qa)

        controls={model.ego.id:ego_action}
        model.setControls(controls)
        print(f"{ego_action.steer},{ego_action.throttle},{ego_action.brake}")
        model.updateVeh()

    #according to collision sensor
    model.record_result(total_start_time, True)
    print("record result")

    # model.destroy()
    gui.terminate()
    gui.join()

