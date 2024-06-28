from RoadInfoGet import RoadInfoGet
from Roadgraph import RoadGraph
from vehicle import Vehicle
from ego_vehicle_planning import LLMEgoPlanner
from dataclasses import field

import carla
from carla import Location, Rotation, Transform
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner

from utils.trajectory import State
from utils.load_config import load_config
import time, copy
import numpy as np
import sys
from typing import Dict, Union, Set, List
import random

class Model:
    def __init__(self,cfgFile: str=None,rouFile: str=None):
        # --------- config ---------#
        if cfgFile:
            self.cfgFile:str=cfgFile
            self.cfg:Dict=load_config(self.cfgFile)
        else:
            self.cfg:Dict=default_config()#负责配置model的各个信息
        self.rouFile=rouFile

        # --------- init carla and roadgraph --------- #
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        self.client.load_world_if_different(self.cfg['map_name'])
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        self.topology = self.carla_map.get_topology()
        self.spectator = self.world.get_spectator()

        self.roadgraph = RoadGraph()
        self.roadInfoGet = RoadInfoGet(self.roadgraph, self.topology)

        # --------- define actors-related property --------- #
        self.v_actors:Dict[int,carla.libcarla.Actor]=field(default_factory=dict)
        self.vehicles:list =[]

        self.p_actors:Dict[int,carla.libcarla.Actor]=field(default_factory=dict)
        self.pedestrians:list=[]

        self.c_actors:Dict[int,carla.libcarla.Actor]=field(default_factory=dict)
        self.cycles:list=[]

        self.ego:Vehicle=None
        self.ego_id:int =-1

        self.timeStep:int = 0

        # tpStart marks whether the trajectory planning is started,
        # when the ego car appears in the network, tpStart turns into 1.
        self.tpStart = 0
        # tpEnd marks whether the trajectory planning is end,
        # when the ego car leaves the network, tpEnd turns into 1.
        self.tpEnd = 0

    def start(self):
        # ---------- init actors --------- #
        #TODO:根据随机初始化结果输出rouFile
        self.vehicles,self.ego=self.initVehicles()
        self.pedestrians=self.initPedestrians()
        self.cycles=self.initCycles()

        self.ego,self.ego_id=self.initEgo()
        self.vehicles.append(self.ego)
        # --------- carla sync mode ---------- #
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        self.world.apply_settings(settings)
        self.world.tick()

        self.timeStep=0

    def setAutoPilot(self):
        for id,actor in self.v_actors.items():
            if id!=self.ego_id:
                actor.set_autopilot()

    def moveStep(self):
        """
        1.carla simulator tick
        2.get vehicles' info
        """
        self.spectator.set_transform(carla.Transform(self.v_actors[self.ego_id].get_transform().location + carla.Location(z=100), carla.Rotation(pitch=-90)))
        self.world.tick()

        self.timeStep+=1

        if self.timeStep%10==0:
            #TODO:动态检测是否到达目的地，并生成新的车辆
            self.getSce()
            self.putRenderData()
            self.putCARLAImage()
            if not self.tpStart:
                self.tpStart = 1

    def getSce(self):
        if not self.ego.arrive_destination():
            self.tpStart = 1
            self.updateSurroundVeh()#更新AOI

            for v in self.vehicles:
                self.getVehInfo(v)
            self.putVehicleINFO()
        else:
            if self.tpStart:
                print('[cyan]The ego car has reached the destination.[/cyan]')
                self.tpEnd = 1

    def getVehInfo(self,vehicle):
        actor=self.v_actors[vehicle.id]

        vehicle.state.x = actor.get_location().x
        vehicle.state.y = actor.get_location().y
        vehicle.state.yaw = np.deg2rad(actor.get_transform().rotation.yaw)
        vehicle.state.s, _ = self.roadgraph.get_lane_by_id(vehicle.lane_id).course_spline.cartesian_to_frenet1D(
            vehicle.state.x, vehicle.state.y)

        cur_lane = self.roadgraph.get_lane_by_id(vehicle.lane_id)
        vehicle.state.s = max(0, vehicle.state.s)

        # ------------ update lane_id and cur_wp ------------ #
        if vehicle.state.s > cur_lane.length:
            if vehicle.cur_wp.is_junction:
                # 说明从junction进入edge，这个时候有可能有重叠的waypoint, 先按照上一个lane给出的next_junction作为id
                next_normal_lane = self.roadgraph.get_lane_by_id(self.roadgraph.WP2Lane[cur_lane.next_lane])
                vehicle.lane_id, vehicle.cur_wp = self.roadgraph.get_laneID_by_s(next_normal_lane.start_wp.road_id,
                                                                            next_normal_lane.start_wp.lane_id,
                                                                            vehicle.state.s - cur_lane.length,
                                                                            self.carla_map)
                while (vehicle.lane_id == None):
                    # 说明这个长度已经超过了下一个lane了
                    vehicle.state.s = vehicle.state.s - cur_lane.length
                    cur_lane = next_normal_lane
                    if next_normal_lane.next_lane:
                        next_normal_lane = self.roadgraph.get_lane_by_id(self.roadgraph.WP2Lane[next_normal_lane.next_lane])
                        vehicle.lane_id, vehicle.cur_wp = self.roadgraph.get_laneID_by_s(next_normal_lane.start_wp.road_id,
                                                                                    next_normal_lane.start_wp.lane_id,
                                                                                    vehicle.state.s - cur_lane.length,
                                                                                    self.carla_map)
                    else:  # next is junction lane
                        cur_edge_id = next_normal_lane.affiliated_section.affliated_edge.id
                        # 找到cur_lane对应的junction_lane
                        for junction_lane in vehicle.available_lanes[cur_edge_id]["junction_lane"]:
                            if self.roadgraph.WP2Lane[junction_lane.previous_lane] == next_normal_lane.id:
                                vehicle.lane_id = junction_lane.id
                                vehicle.lane_id, vehicle.cur_wp = self.roadgraph.get_laneID_by_s(
                                    junction_lane.start_wp.road_id, junction_lane.start_wp.lane_id,
                                    vehicle.state.s - cur_lane.length, self.carla_map)

                vehicle.state.s, _ = self.roadgraph.get_lane_by_id(vehicle.lane_id).course_spline.cartesian_to_frenet1D(
                    vehicle.state.x, vehicle.state.y)
                vehicle.state.s = max(0, vehicle.state.s)

            else:
                # 说明从edge进入junction
                cur_edge_id = self.roadgraph.get_lane_by_id(vehicle.lane_id).affiliated_section.affliated_edge.id
                for junction_lane in vehicle.available_lanes[cur_edge_id]["junction_lane"]:
                    # 找到cur_lane对应的junction_lane
                    if self.roadgraph.WP2Lane[junction_lane.previous_lane] == vehicle.lane_id:
                        vehicle.lane_id = junction_lane.id
                        vehicle.cur_wp=self.carla_map.get_waypoint_xodr(junction_lane.start_wp.road_id,junction_lane.start_wp.lane_id,vehicle.state.s - cur_lane.length)
                        vehicle.state.s, _ = self.roadgraph.get_lane_by_id(
                            vehicle.lane_id).course_spline.cartesian_to_frenet1D(vehicle.state.x, vehicle.state.y)
                        vehicle.state.s = max(0, vehicle.state.s)
                        break

        if vehicle.cur_wp.is_junction:
            # lane_id,cur_wp=self.roadgraph.get_laneID_by_s(vehicle.cur_wp.road_id, vehicle.cur_wp.lane_id,
            #                                                             vehicle.state.s, self.carla_map)
            lane_id=vehicle.lane_id
            lane_id_by_carla,cur_wp=self.roadgraph.get_laneID_by_xy(vehicle.state.x, vehicle.state.y, self.carla_map)
            if not cur_wp.is_junction:
                lane_id=lane_id_by_carla
            vehicle.lane_id,vehicle.cur_wp=lane_id,cur_wp

            vehicle.state.s, _ = self.roadgraph.get_lane_by_id(vehicle.lane_id).course_spline.cartesian_to_frenet1D(
                vehicle.state.x, vehicle.state.y)
        else:
            vehicle.lane_id, vehicle.cur_wp = self.roadgraph.get_laneID_by_xy(vehicle.state.x, vehicle.state.y, self.carla_map)
            vehicle.state.s, _ = self.roadgraph.get_lane_by_id(vehicle.lane_id).course_spline.cartesian_to_frenet1D(
                vehicle.state.x, vehicle.state.y)

    def putVehicleINFO(self):
        pass
    def putRenderData(self):
        pass
    def putCARLAImage(self):
        pass
    def updateSurroundVeh(self):
        pass
    def updateVeh(self):
        """
        update vehicles' actions in carla
        """
        for vehicle in self.vehicles:
            if vehicle.trajectory:
                vehicle.state=vehicle.trajectory.states[0]
                centerx, centery, yaw, speed, accel = vehicle.trajectory.pop_last_state()
                self.v_actors[vehicle.id].set_transform(carla.Transform(location=carla.Location(x=centerx, y=centery, z=0),
                                                    rotation=carla.Rotation(yaw=np.rad2deg(yaw))))
    def destroy(self):
        # ------- close carla sync mode ------- #
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        # self.dbBridge.close()

    def initEgo(self):
        blueprint_library=self.world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))

        if self.cfg['ego_path']:
            #TODO：检查一下route的格式，解析他，设计ego_path的格式
            raise NotImplementedError
        else:
            start_waypoint = self.createSpawnPoints(k=1)[0]
            route, end_waypoint = self.randomRoute(start_waypoint)

        try:
            actor = self.world.try_spawn_actor(vehicle_bp, start_waypoint)
        except:
            start_waypoint = self.createSpawnPoints(k=1)[0]
            route, end_waypoint = self.randomRoute(start_waypoint)
            actor = self.world.try_spawn_actor(vehicle_bp, start_waypoint)

        self.v_actors[actor.id]=actor

        vehicle = Vehicle(actor, start_waypoint, end_waypoint, 100, route)
        vehicle.get_route(self.carla_map, self.roadgraph)
        return vehicle,actor.id

    def initVehicles(self):
        blueprint_library=self.world.get_blueprint_library()

        spawn_points = self.createSpawnPoints(self.cfg['init']['veh_num'])
        for i in self.cfg['init']['veh_num']:
            vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
            actor=self.world.spawn_actor(vehicle_bp, spawn_points[i])
            self.v_actors[actor.id]=actor

        vehicles=[]
        for id, actor in self.v_actors.items():
            start_waypoint = self.carla_map.get_waypoint(actor.get_location())
            # TODO：终点可以是roadgrpah中某条道路的endwp
            route,end_waypoint=self.randomRoute(start_waypoint)
            vehicle = Vehicle(actor,start_waypoint, end_waypoint, 100,route)
            vehicle.get_route(self.carla_map,self.roadgraph)
            vehicles.append(vehicle)

        return vehicles

    def randomRoute(self,start_waypoint):
        #1.找到当前edge
        cur_lane_id=self.roadgraph.WP2Lane[(start_waypoint.road_id,start_waypoint.section_id,start_waypoint.lane_id)]
        cur_edge= self.roadgraph.get_lane_by_id(cur_lane_id).affiliated_section.affliated_edge
        #2.沿着连接关系去找终点，直到满足长度限制：
        search_length=0
        route=[cur_edge.id]
        lane=None
        while search_length < self.cfg['travel_distance']:
            next_edge_list=list(cur_edge.next_edge_connect.keys())
            next_edge_id=random.choice(next_edge_list)
            next_edge=self.roadgraph.Edges[next_edge_id]

            section=self.roadgraph.Sections[next_edge.section_list[0]]
            lane=self.roadgraph.NormalLane_Dict[list(section.lanes.values())[0]]

            search_length+=lane.length
            route.append(next_edge_id)

            cur_edge=next_edge

        end_waypoint = lane.end_wp
        return route,end_waypoint

    def initPedestrians(self):
        pass

    def initCycles(self):
        pass

    def createSpawnPoints(self,k):
        spawn_points = self.carla_map.get_spawn_points()
        assert k<= len(spawn_points)
        randomSpawnPoints=random.sample(spawn_points,k)
        #TODO：待确认spawnPoints的相距是否合理
        return randomSpawnPoints


if __name__=='__main__':
    config_name='example.yaml'

    model=Model(cfgFile=config_name)
    planner = LLMEgoPlanner()

    model.start()
    model.setAutoPilot()

    model.ego.available_lanes = model.roadgraph.get_all_available_lanes(model.ego.route, model.ego.end_waypoint)
    model.ego.next_available_lanes =  model.ego.get_available_lanes(model.roadgraph)
    model.ego.trajectory=planner.plan(model.ego, model.roadgraph, None, model.timeStep)
    #TODO:GUI update
    #gui = GUI(model)
    #gui.start()
    while not model.tpEnd:
        model.moveStep()
        if model.timeStep%10 ==0:
            model.ego.next_available_lanes = model.ego.get_available_lanes(model.roadgraph)
            model.ego.trajectory = planner.plan(model.ego, model.roadgraph, None, model.timeStep)
            print(model.ego.lane_id)
            print(model.ego.behaviour)
            world=model.world
            for state in model.ego.trajectory.states:
                world.debug.draw_point(carla.Location(x=state.x,y=state.y,z=0.5),color=carla.Color(r=0, g=0, b=255), life_time=1, size=0.1)

        model.updateVeh()

    model.destroy()
    # gui.terminate()
    # gui.join()

