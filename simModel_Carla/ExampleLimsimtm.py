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

    planner = TrafficManager(model)
    planner.config["EGO_PLANNER"] = False
    planner.config['EGO_CONTROL'] = True

    model.start()
    model.runAutoPilot()
    
    
    gui = GUI(model)
    gui.start()

    while not model.tpEnd:
        model.moveStep()
        if model.shouldUpdate():

            roadgraph, vehicles = model.exportSce()
            trajectories = planner.plan(
                model.timeStep * 0.1, roadgraph, vehicles, model.ego.behaviour, other_plan=True
            )

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

