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
        self.cfgFile:str=cfgFile
        self.cfg:Dict=load_config(self.cfgFile)
        self.rouFile=rouFile

        # --------- init carla and roadgraph --------- #
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(20.0)
        self.client.load_world(self.cfg['map_name'])
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        self.topology = self.carla_map.get_topology()
        self.spectator = self.world.get_spectator()

        self.roadgraph = RoadGraph()
        self.roadInfoGet = RoadInfoGet(self.roadgraph, self.topology)

        # --------- define actors-related property --------- #
        self.v_actors:Dict[int,carla.libcarla.Vehicle]={}
        self.vehicles:list =[]

        self.p_actors:Dict[int,carla.libcarla.Actor]={}
        self.pedestrians:list=[]

        self.c_actors:Dict[int,carla.libcarla.Actor]={}
        self.cycles:list=[]

        self.ego:Vehicle=None
        self.ego_id:int =-1

        self.timeStep:int = 0
        self.updateInterval=self.cfg['updateInterval']
        self.vehicleSpawnInterval=self.cfg['vehicleSpawnInterval']
        self.vehicleCheckRange=self.cfg['vehicleCheckRange']
        self.vehicleVisbleRange=self.cfg['vehicleVisibleRange']

        # tpStart marks whether the trajectory planning is started,
        # when the ego car appears in the network, tpStart turns into 1.
        self.tpStart = 0
        # tpEnd marks whether the trajectory planning is end,
        # when the ego car leaves the network, tpEnd turns into 1.
        self.tpEnd = 0

    def start(self):
        # ---------- init actors --------- #
        #TODO:根据随机初始化结果输出rouFile,且思考rouFile后续如何使用
        self.vehicles=self.initVehicles()
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

    def setAutoPilot(self,vehicle):
        vehicle.actor.set_autopilot()
        vehicle.isAutoPilot=True

        #route会被改变，由carla的routeplanner
        grp = GlobalRoutePlanner(self.carla_map, 0.5)
        route_carla = grp.trace_route(vehicle.start_waypoint.transform.location, vehicle.end_waypoint.transform.location)
        vehicle.route = self.roadgraph.get_route_edge(route_carla)

        #TODO: check if path can work
        route_carla=[wp[0].transform.location for wp in route_carla]

        self.carla_tm.set_path(vehicle.actor,route_carla)

    def runAutoPilot(self):
        # TODO:创建了新车之后，也要将其设置成autopilot
        self.carla_tm = self.client.get_trafficmanager()
        self.tm_port = self.carla_tm.get_port()

        for vehicle in self.vehicles:
            if vehicle!=self.ego and not vehicle.isAutoPilot:
                self.setAutoPilot(vehicle)

        for id, actor in self.v_actors.items():
            if id != self.ego_id:
                actor.set_autopilot()
    def shouldUpdate(self):
        return self.timeStep%self.updateInterval==0
    def shouldSpawnVeh(self):
        return self.timeStep%self.vehicleSpawnInterval==0

    def checkSpawnPoint(self,spawn_point):
        for veh in self.vehicles:
            cur_lc=veh.actor.get_location()
            if spawn_point.location.distance(cur_lc)<=self.vehicleCheckRange:
                cur_lane_id=veh.lane_id
                sp_lane_id=self.roadgraph.get_laneID_by_xy(spawn_point.location.x,spawn_point.location.y,self.carla_map)
                if cur_lane_id==sp_lane_id:
                    return False
        return True

    def spawnVehicleRuntime(self):
        blueprint_library=self.world.get_blueprint_library()
        spawn_point = self.createSpawnPoints(1)[0]
        if self.checkSpawnPoint(spawn_point):
            vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
            actor = self.world.spawn_actor(vehicle_bp, spawn_point)
            self.v_actors[actor.id] = actor

            start_waypoint = self.carla_map.get_waypoint(actor.get_location())
            route, end_waypoint = self.randomRoute(start_waypoint)

            vehicle = Vehicle(actor,start_waypoint, end_waypoint, self.vehicleVisbleRange,route)
            vehicle.get_route(self.carla_map,self.roadgraph)

            if self.carla_tm:
                self.setAutoPilot(vehicle)

            self.vehicles.append(vehicle)
            return True
        else:
            return False



    def moveStep(self):
        """
        1.carla simulator tick
        2.get vehicles' info
        """
        self.spectator.set_transform(carla.Transform(self.v_actors[self.ego_id].get_transform().location + carla.Location(z=100), carla.Rotation(pitch=-90)))
        self.world.tick()

        self.timeStep+=1

        if self.shouldUpdate():
            self.getSce()
            self.putRenderData()
            self.putCARLAImage()
            if not self.tpStart:
                self.tpStart = 1

        if self.shouldSpawnVeh():
            spawn_success=self.spawnVehicleRuntime()


    def getSce(self):
        if not self.ego.arrive_destination():
            self.tpStart = 1
            self.removeArrivedVeh()
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
        #1.记录每个车是否在AOI内
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
    def removeArrivedVeh(self):
        for veh in self.vehicles:
            if veh!=self.ego and veh.arrive_destination():
                self.vehicles.remove(veh)
                veh.actor.destroy()
                del self.v_actors[veh.id]

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

        if self.cfg['ego_path']!='None':
            #TODO：检查一下route的格式，解析他，设计ego_path的格式
            raise NotImplementedError
        
        start_waypoint = self.createSpawnPoints(k=1)[0]
        wp_isjunction=self.carla_map.get_waypoint(start_waypoint.location).is_junction
        while wp_isjunction:
            start_waypoint = self.createSpawnPoints(k=1)[0]
            wp_isjunction=self.carla_map.get_waypoint(start_waypoint.location).is_junction
        actor = self.world.try_spawn_actor(vehicle_bp, start_waypoint)
        self.v_actors[actor.id]=actor

        start_waypoint=self.carla_map.get_waypoint(actor.get_location())
        route, end_waypoint = self.randomRoute(start_waypoint)
        vehicle = Vehicle(actor, start_waypoint, end_waypoint, self.vehicleVisbleRange, route)
        vehicle.get_route(self.carla_map, self.roadgraph)
        return vehicle,actor.id

    def initVehicles(self):
        blueprint_library=self.world.get_blueprint_library()

        spawn_points = self.createSpawnPoints(self.cfg['init']['veh_num'])
        for i in range(self.cfg['init']['veh_num']):
            try:
                vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
                actor=self.world.spawn_actor(vehicle_bp, spawn_points[i])
                self.v_actors[actor.id]=actor
            except RuntimeError:
                pass

        vehicles=[]
        for id, actor in self.v_actors.items():
            start_waypoint = self.carla_map.get_waypoint(actor.get_location())
            route,end_waypoint=self.randomRoute(start_waypoint)
            vehicle = Vehicle(actor,start_waypoint, end_waypoint, self.vehicleVisbleRange,route)
            vehicle.get_route(self.carla_map,self.roadgraph)
            vehicles.append(vehicle)

        return vehicles

    def randomRoute(self,start_waypoint):
        #1.找到当前edge
        route=[]
        cur_lane_id=self.roadgraph.WP2Lane[(start_waypoint.road_id,start_waypoint.section_id,start_waypoint.lane_id)]
        try:
            cur_edge= self.roadgraph.get_lane_by_id(cur_lane_id).affiliated_section.affliated_edge
        except AttributeError:
            cur_junction=self.roadgraph.get_lane_by_id(cur_lane_id)
            cur_edge=self.roadgraph.Edges[cur_junction.outgoing_edge_id]
            route=[cur_junction.incoming_edge_id]
        #2.沿着连接关系去找终点，直到满足长度限制：
        search_length=0
        route.append(cur_edge.id)
        lane=None
        while search_length < self.cfg['init']['travel_distance']:
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

    def createSpawnPoints(self,k:int)->list:
        spawn_points = self.carla_map.get_spawn_points()
        assert k<= len(spawn_points)
        randomSpawnPoints=random.sample(spawn_points,k)
        #TODO：待确认spawnPoints的相距是否合理，现在默认是合理的，没有检查
        return randomSpawnPoints


if __name__=='__main__':
    config_name='./simModel_Carla/example_config.yaml'

    model=Model(cfgFile=config_name)
    planner = LLMEgoPlanner()

    model.start()
    model.runAutoPilot()
    model.ego.available_lanes = model.roadgraph.get_all_available_lanes(model.ego.route, model.ego.end_waypoint)
    model.ego.next_available_lanes =  model.ego.get_available_lanes(model.roadgraph)
    model.ego.trajectory=planner.plan(model.ego, model.roadgraph, None, model.timeStep)
    #TODO:GUI update
    #gui = GUI(model)
    #gui.start()

    while not model.tpEnd:
        model.moveStep()
        if model.shouldUpdate():
            model.ego.next_available_lanes = model.ego.get_available_lanes(model.roadgraph)
            model.ego.trajectory = planner.plan(model.ego, model.roadgraph, None, model.timeStep)#TODO:采用model方法set_trajectory来给轨迹赋值
            print(model.ego.lane_id)
            print(model.ego.behaviour)
            world=model.world
            for state in model.ego.trajectory.states:
                world.debug.draw_point(carla.Location(x=state.x,y=state.y,z=0.5),color=carla.Color(r=0, g=0, b=255), life_time=1, size=0.1)

        model.updateVeh()

    model.destroy()
    # gui.terminate()
    # gui.join()

