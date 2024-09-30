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
from AD_algo.Interfuser.interfuser_agent import InterfuserAgent
from scenario_runner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from scenario_runner.srunner.scenariomanager.timer import GameTime
from leaderboard_util import initDataProvider,route_transform,setup_sensors

if __name__=='__main__':

    config_name='./simModel_Carla/example_config.yaml'
    random.seed(112102)

    stringTimestamp = datetime.strftime(datetime.now(), '%Y-%m-%d_%H-%M-%S')    
    database = 'results/' + stringTimestamp + '.db'
    total_start_time = time.time()

    model:Model=Model(cfgFile=config_name,dataBase=database)

    planner = TrafficManager(model)


    model.start()
    model.runAutoPilot()

    #DATAPROVIDER Initiate
    initDataProvider(model)#CarlaDataProvider的记录actor只能记录调用他的方法生成的actor，导致无法与ego连接，新写一个函数
    GameTime.restart()
    
    #Interfuser Agent settup
    path_to_config=os.path.join('AD_algo/Interfuser','interfuser_config.py')
    interfuser=InterfuserAgent(path_to_config)
    gps_route,route=route_transform(model.roadgraph,model.ego)
    
    # world=model.world
    # for wp in route:
    #     world.debug.draw_point(wp[0].location, color=carla.Color(r=0, g=0, b=255), life_time=5000, size=0.1)
    # world.debug.draw_point(route[-1][0].location, color=carla.Color(r=255, g=0, b=0), life_time=5000, size=5)
    
    # settings = world.get_settings()
    # settings.synchronous_mode = False
    # settings.fixed_delta_seconds = None
    # world.apply_settings(settings)    
    
    # for item in model.roadgraph.Edges.items():
    #     model.world.debug.draw_string(item[1].last_segment[0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)
    
    interfuser.set_global_plan(gps_route,route)#将model中的全局路径传递给pdm
    interfuser.setup(path_to_config)
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
        ego_action = pdm()
        controls={model.ego.id:ego_action}
        model.setControls(controls)
        model.updateVeh()

    #according to collision sensor
    model.record_result(total_start_time, True, None)

    model.destroy()
    gui.terminate()
    gui.join()

