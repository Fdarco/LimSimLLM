import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import timm
import argparse
from argparse import RawTextHelpFormatter
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
from leaderboard_util import route_transform
from DataQueue import QuestionAndAnswer
from leaderboard_util import initDataProvider,route_transform,setup_sensors
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
    planner.config["EGO_PLANNER"] = False
    planner.config['EGO_CONTROL'] = True

    model.start()
    model.runAutoPilot()
    initDataProvider(model)#CarlaDataProvider的记录actor只能记录调用他的方法生成的actor，导致无法与ego连接，新写一个函数
    

    gps_route,route=route_transform(model.roadgraph,model.ego)
    route_length = 0.0 
    for idx,wp in enumerate(route):
        if idx!=len(route)-1:
            route_length+=wp[0].location.distance(route[idx+1][0].location)
    model.simDescriptionCommit(route_length)
    
    gui = GUI(model)
    gui.start()

    # world=model.world
    # available_dict= model.ego.available_lanes
    # # ----------------- 在carla中画出available lane ----------------- #
    # for edge_id, available_lanes in available_dict.items():
    #     for section_id, lanes in available_lanes['available_lane'].items():
    #         for lane in lanes:
    #             # if lane in available_lanes['change_lane']:
    #             #     continue
    #             world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=0, g=255, b=0), thickness=1.0, life_time=1000)
    #     for junction_lane in available_lanes['junction_lane']:
    #         world.debug.draw_line(junction_lane.start_wp.transform.location+carla.Location(z=1), junction_lane.end_wp.transform.location+carla.Location(z=1), color=carla.Color(r=0, g=0, b=255), thickness=1.0, life_time=1000)
    #     for section_id, lanes in available_lanes['change_lane'].items():
    #         for lane in lanes:
    #             world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=255, g=0, b=0), thickness=1.0, life_time=1000)
    
    # settings = world.get_settings()
    # settings.synchronous_mode = False
    # settings.fixed_delta_seconds = None
    # world.apply_settings(settings)
    # breakpoint()

    while not model.tpEnd:
        model.moveStep()
        if model.shouldUpdate():

            roadgraph, vehicles = model.exportSce()
            start = time.time()
            trajectories = planner.plan(
                model.timeStep * 0.1, roadgraph, vehicles, model.ego.behaviour, other_plan=True
            )
            end = time.time()
            timeCost = end - start
            qa = QuestionAndAnswer('', '', '', '', f'Limsimtm: {timeCost:.4f}s', 
                               0, 0, 0, timeCost, 0)
            model.putQA(qa)
            model.setTrajectories(trajectories)

            print(model.ego.lane_id)
            print(model.ego.behaviour)

            # world=model.world
            # for veh in model.vehicles:
            #     if veh.trajectory:
            #         for state in veh.trajectory.states:
            #             world.debug.draw_point(carla.Location(x=state.x,y=state.y,z=0.5),color=carla.Color(r=0, g=0, b=255), life_time=1, size=0.1)
        

        model.updateVeh()

    #according to collision sensor
    model.record_result(total_start_time, True)

    # model.destroy()
    gui.terminate()
    gui.join()

