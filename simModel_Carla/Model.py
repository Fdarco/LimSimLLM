from utils.load_config import load_config
from simModel_Carla.DBBridge import DBBridge
from RoadInfoGet import RoadInfoGet
from Roadgraph import RoadGraph
from vehicle import Vehicle,vehType
from ego_vehicle_planning import LLMEgoPlanner
from simModel_Carla.DataQueue import (
    CameraImages, ImageQueue, QAQueue, QuestionAndAnswer, RenderQueue
)
from simModel_Carla.Camera import nuScenesImageExtractor
from simModel_Carla.MPGUI import GUI
from trafficManager.traffic_manager_carla import TrafficManager
from trafficManager.common.vehicle import Behaviour


import carla
from carla import Location, Rotation, Transform
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner

import time, copy
import numpy as np
from math import sqrt
import sqlite3
import sys
from typing import Dict, Union, Set, List
import random
import os
import dill
import pickle
from dataclasses import field
from datetime import datetime

class Model:
    def __init__(self,cfgFile: str=None,rouFile: str=None,dataBase: str=None):
        # --------- config ---------#
        self.cfgFile:str=cfgFile
        self.cfg:Dict=load_config(self.cfgFile)
        self.rouFile=rouFile
        self.sim_mode: str = 'RealTime'

        # --------- init carla and roadgraph --------- #
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(20.0)
        self.client.load_world(self.cfg['map_name'])
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        self.topology = self.carla_map.get_topology()
        self.spectator = self.world.get_spectator()
        self.carla_tm=None

        self.roadgraph = RoadGraph()
        self.map_cache_path=os.path.join(os.getcwd(),'simModel_Carla','map_cache', f"{self.cfg['map_name']}.pkl")
        self.check_and_load_mapcache()

        # --------- define actors-related property --------- #
        self.v_actors:Dict[int,carla.libcarla.Vehicle]={}
        self.vehicles:List[Vehicle] =[]

        self.p_actors:Dict[int,carla.libcarla.Actor]={}
        self.pedestrians:list=[]

        self.c_actors:Dict[int,carla.libcarla.Actor]={}
        self.cycles:list=[]

        self.ego:Vehicle=None
        self.ego_id:int =-1

        self.vehINAoI:Dict[int,Vehicle]={}
        self.outOfAoI: Dict[int, Vehicle] = {}
        self.allvTypes:Dict[int,vehType]={}

        self.timeStep:int = 0
        self.updateInterval=self.cfg['updateInterval']
        self.vehicleSpawnInterval=self.cfg['vehicleSpawnInterval']
        self.vehicleCheckRange=self.cfg['vehicleCheckRange']
        self.vehicleVisbleRange=self.cfg['vehicleVisibleRange']
        self.max_veh_num=self.cfg['max_veh_num']

        if dataBase:
            self.dataBase = dataBase
        else:
            self.dataBase = datetime.strftime(
                datetime.now(), '%Y-%m-%d_%H-%M-%S') + '_egoTracking' + '.db'

        if os.path.exists(self.dataBase):
            os.remove(self.dataBase)
        self.dbBridge = DBBridge(self.dataBase)
        self.dbBridge.createTable()
        self.renderQueue = RenderQueue(5)
        self.imageQueue = ImageQueue(50)
        self.QAQ = QAQueue(5)

        self.imageExtractor=nuScenesImageExtractor()

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
        
        for veh in self.vehicles:
            veh.available_lanes=self.roadgraph.get_all_available_lanes(veh.route, veh.end_waypoint)
            
        self.simDescriptionCommit()
        # --------- carla sync mode ---------- #
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        self.world.apply_settings(settings)
        self.world.tick()

        self.timeStep=0

    def simDescriptionCommit(self):
        currTime = datetime.now()
        insertQuery = '''INSERT INTO simINFO VALUES (?, ?, ?);'''
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            insertQuery,
            (currTime, self.ego.id, '')
        )

        conn.commit()
        cur.close()
        conn.close()

    def check_and_load_mapcache(self):
        if os.path.exists(self.map_cache_path) and not self.cfg['rebuild_roadgraph']:
            with open(self.map_cache_path, 'rb') as f:
                self.roadgraph = dill.load(f)
            #transform wp_list of lane from dict to carla.Waypoint, because waypoint cant be pickled
            self.roadgraph.wp_transform(self.carla_map)
            print("Loaded roadgraph from cache.")
        else:
            self.roadInfoGet = RoadInfoGet(self.roadgraph, self.topology)
            #Waypoint -> dict 
            self.roadgraph.wp_transform(self.carla_map)
            with open(self.map_cache_path, 'wb') as f:
                dill.dump(self.roadgraph, f)
            #dict -> Waypoint
            self.roadgraph.wp_transform(self.carla_map)
            print("Generated roadgraph and saved to cache.")
        #get junction traffic light
        self.roadgraph.get_traffic_light(self.world)
    def setAutoPilot(self,vehicle:Vehicle):
        vehicle.actor.set_simulate_physics(True)
        vehicle.actor.set_autopilot(True)
        vehicle.isAutoPilot=True

        vehicle.route=None#vehicle controled by autopilot cant follow the given route
        vehicle.available_lanes=dict()
        
        self.carla_tm.auto_lane_change(vehicle.actor,True)
        #set_path function is not working properly
        # route_carla=self.routeTransform(vehicle)
        # self.carla_tm.set_path(vehicle.actor,route_carla)

    def runAutoPilot(self):
        self.carla_tm = self.client.get_trafficmanager()
        self.tm_port = self.carla_tm.get_port()
        self.carla_tm.set_synchronous_mode(True)

        for vehicle in self.vehicles:
            if vehicle!=self.ego and not vehicle.isAutoPilot:
                self.setAutoPilot(vehicle)

    def shouldUpdate(self):
        return self.timeStep%self.updateInterval==0
    def shouldSpawnVeh(self):
        return self.timeStep%self.vehicleSpawnInterval==0 and len(self.vehicles)<self.max_veh_num
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
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.mercedes.*'))
        actor = self.world.try_spawn_actor(vehicle_bp, spawn_point)
        if actor:
            self.v_actors[actor.id] = actor

            # start_waypoint = self.carla_map.get_waypoint(actor.get_location())
            start_waypoint=self.carla_map.get_waypoint(spawn_point.location)
            route, end_waypoint = self.randomRoute(start_waypoint)

            vehicle = Vehicle(actor,start_waypoint, end_waypoint, self.vehicleVisbleRange,route)
            vehicle.get_route(self.carla_map,self.roadgraph)

            vehicle.available_lanes=self.roadgraph.get_all_available_lanes(vehicle.route,vehicle.end_waypoint)

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
            if self.cfg['carlaImage']:
                self.putCARLAImage()
            if not self.tpStart:
                self.tpStart = 1

        if self.shouldSpawnVeh():
            spawn_success=self.spawnVehicleRuntime()
            print('spawn_success:',spawn_success)

    def resetRoute(self,vehicle:Vehicle):
        if vehicle.lane_id in self.roadgraph.Junction_Dict.keys():
            grp = GlobalRoutePlanner(self.carla_map, 0.5)
            junction_lane=self.roadgraph.get_lane_by_id(vehicle.lane_id)
            route_carla = grp.trace_route(self.roadgraph.get_lane_by_id(self.roadgraph.WP2Lane[junction_lane.next_lane]).wp_list[0].transform.location, vehicle.end_waypoint.transform.location)
            incoming_edge=vehicle.lane_id.split('-')[0]
            vehicle.route = self.roadgraph.get_route_edge(route_carla)
            if incoming_edge not in vehicle.route:
                vehicle.route=[incoming_edge]+vehicle.route
        else:
            grp = GlobalRoutePlanner(self.carla_map, 0.5)
            route_carla = grp.trace_route(vehicle.actor.get_location(), vehicle.end_waypoint.transform.location)
            # route_carla = grp.trace_route(self.roadgraph.get_lane_by_id(vehicle.lane_id).wp_list[0].transform.location, vehicle.end_waypoint.transform.location)
            vehicle.route = self.roadgraph.get_route_edge(route_carla)

    def getSce(self):
        if not self.ego.arrive_destination():
            self.tpStart = 1
            self.removeArrivedVeh()

            self.getVehInfo(self.ego)
            for v in self.vehicles:
                if v.actor.is_active and v!=self.ego:
                    self.getVehInfo(v)
        
            if self.carla_tm:
                for vehicle in self.vehINAoI.values():
                    if vehicle.route == None:
                        self.resetRoute(vehicle)
                for vehicle in self.outOfAoI.values():
                    if not vehicle.isAutoPilot and vehicle.id != self.ego_id:
                        self.setAutoPilot(vehicle)
            
            self.updateAvailableLane()

            self.putVehicleINFO()
        else:
            if self.tpStart:
                print('[cyan]The ego car has reached the destination.[/cyan]')
                self.tpEnd = 1

    def getVehInfo(self,vehicle:Vehicle):
        if vehicle.id not in self.allvTypes:
            vtins=vehType(str(vehicle.id))
            vtins.maxAccel=5
            vtins.maxDecel=5
            vtins.maxSpeed=vehicle.actor.get_speed_limit()
            vtins.length=vehicle.length
            vtins.width=vehicle.width
            vtins.vclass=vehicle.actor.type_id
            routes =  '' if not vehicle.route else ' '.join(vehicle.route)
            self.commitVehicleInfo(str(vehicle.id), vtins, routes)
            self.allvTypes[vehicle.id]=vtins

        actor=self.v_actors[vehicle.id]

        x=vehicle.state.x = actor.get_location().x
        y=vehicle.state.y = actor.get_location().y
        yaw=vehicle.state.yaw = np.deg2rad(actor.get_transform().rotation.yaw)

        #whether the vehicle is in the AoI
        if vehicle.id !=self.ego_id:
            ex,ey=self.ego.state.x,self.ego.state.y
            if sqrt(pow((ex - x), 2) + pow((ey - y), 2)) <= self.cfg['deArea']:
                if not vehicle.id in self.vehINAoI.keys():
                    self.vehINAoI[vehicle.id]=vehicle
                if vehicle.id in self.outOfAoI.keys():
                    del self.outOfAoI[vehicle.id]
            else:
                if vehicle.id in self.vehINAoI.keys():
                    del self.vehINAoI[vehicle.id]
                if not vehicle.id in self.outOfAoI.keys():
                    self.outOfAoI[vehicle.id] = vehicle

        vehicle.lane_id,vehicle.state.s,vehicle.cur_wp=self.localization(vehicle)

        #get route_idx
        lane=self.roadgraph.get_lane_by_id(lane_id=vehicle.lane_id)
        try:
            #lane is normal_lane
            road_id=lane.affiliated_section.affliated_edge.id
        except:
            #lane is junction_lane
            road_id=lane.id.split('-')[0]

        if vehicle.route and road_id in vehicle.route:
            route_idx=vehicle.route.index(road_id)
        else:
            route_idx=0

        vehicle.xQ.append(x)
        vehicle.yQ.append(y)
        vehicle.yawQ.append(yaw)
        vehicle.speedQ.append(vehicle.actor.get_velocity().length())
        vehicle.accelQ.append(vehicle.actor.get_acceleration().length())
        vehicle.routeIdxQ.append(route_idx)
        vehicle.laneIDQ.append(vehicle.lane_id)
        vehicle.lanePosQ.append(vehicle.state.s)

    def localization(self,vehicle):
        cur_lane = self.roadgraph.get_lane_by_id(vehicle.lane_id)
        cur_wp=vehicle.cur_wp if vehicle.cur_wp else self.carla_map.get_waypoint(carla.Location(x=vehicle.state.x, y=vehicle.state.y))
        carla_wp=self.carla_map.get_waypoint(carla.Location(x=vehicle.state.x, y=vehicle.state.y))

        if carla_wp.is_junction:
            junction=carla_wp.get_junction()
            wp_tuple = junction.get_waypoints(carla.LaneType.Driving)
            possible_junctionlane=[]
            for i in range(len(wp_tuple)):
                (start_wp, _) = wp_tuple[i]
                if (start_wp.road_id,start_wp.section_id,start_wp.lane_id) in self.roadgraph.WP2Lane.keys():
                    tmp_lane=self.roadgraph.get_lane_by_id(self.roadgraph.WP2Lane[(start_wp.road_id,start_wp.section_id,start_wp.lane_id)])
                    if tmp_lane.id in self.roadgraph.Junction_Dict.keys():
                        s,d=tmp_lane.course_spline.cartesian_to_frenet1D(vehicle.state.x, vehicle.state.y)
                        if abs(d)<=2:
                            possible_junctionlane.append(tmp_lane)
            if not possible_junctionlane:
                next_lane_id=vehicle.lane_id
                junction_lane=cur_lane
            else:
                for junction_lane in possible_junctionlane:
                    if junction_lane.id in vehicle.next_available_lanes:
                        break
                next_lane_id=junction_lane.id
            next_s, _ = self.roadgraph.get_lane_by_id(next_lane_id).course_spline.cartesian_to_frenet1D(vehicle.state.x, vehicle.state.y)
            next_s = max(0, next_s)
            next_wp=self.carla_map.get_waypoint_xodr(junction_lane.start_wp.road_id,junction_lane.start_wp.lane_id,next_s)
            return next_lane_id,next_s,next_wp

        elif not carla_wp.is_junction:
            next_lane_id=self.roadgraph.WP2Lane[(carla_wp.road_id,carla_wp.section_id,carla_wp.lane_id)]
            next_s, _ = self.roadgraph.get_lane_by_id(
                               next_lane_id).course_spline.cartesian_to_frenet1D(vehicle.state.x, vehicle.state.y)
            next_s = max(0, next_s)
            next_wp=carla_wp
            return next_lane_id,next_s,next_wp
                        
        raise NotImplementedError 

    def commitFrameInfo(self, vid: int, vtag: str, veh: Vehicle):
        self.dbBridge.putData(
            'frameINFO',
            (
                self.timeStep, vid, vtag, veh.state.x, veh.state.y, veh.state.yaw, veh.speedQ[-1],
                veh.accelQ[-1], veh.lane_id, veh.state.s, veh.routeIdxQ[-1]
            )
        )

    def commitVehicleInfo(self, vid: str, vtins: vehType, routes: str):
        self.dbBridge.putData(
            'vehicleINFO',
            (
                vid, vtins.length, vtins.width, vtins.maxAccel,
                vtins.maxDecel, vtins.maxSpeed, vtins.id, routes
            )
        )

    def putVehicleINFO(self):
        self.commitFrameInfo(self.ego.id, 'ego', self.ego)
        for v1 in self.vehINAoI.values():
            self.commitFrameInfo(v1.id, 'AoI', v1)
        for v2 in self.outOfAoI.values():
            self.commitFrameInfo(v2.id, 'outOfAoI', v2)

    def putRenderData(self):
        if self.tpStart:
            roadgraphRenderData, VRDDict = self.roadgraph.exportRenderData(self.ego,self.vehINAoI,self.outOfAoI)
            self.renderQueue.put((roadgraphRenderData, VRDDict))

    def putCARLAImage(self):
        self.imageExtractor.setCameras(self.ego,self.world)
        ci = self.imageExtractor.getCameraImages()
        if ci:
            ci.resizeImage(560, 315)
            self.imageQueue.put(ci)
            self.dbBridge.putData(
                'imageINFO',
                (
                    self.timeStep,
                    sqlite3.Binary(pickle.dumps(ci.CAM_FRONT)),
                    sqlite3.Binary(pickle.dumps(ci.CAM_FRONT_RIGHT)),
                    sqlite3.Binary(pickle.dumps(ci.CAM_FRONT_LEFT)),
                    sqlite3.Binary(pickle.dumps(ci.CAM_BACK_LEFT)),
                    sqlite3.Binary(pickle.dumps(ci.CAM_BACK)),
                    sqlite3.Binary(pickle.dumps(ci.CAM_BACK_RIGHT))
                )
            )


    def getCARLAImage(
            self, start_frame: int, steps: int = 1
    ) -> List[CameraImages]:
        return self.imageQueue.get(start_frame, steps)

    def putQA(self, QA: QuestionAndAnswer):
        self.QAQ.put(QA)
        self.dbBridge.putData(
            'QAINFO',
            (
                self.timeStep, QA.description, QA.navigation,
                QA.actions, QA.few_shots, QA.response,
                QA.prompt_tokens, QA.completion_tokens, QA.total_tokens, QA.total_time, QA.choose_action
            )
        )
    def shutdownAutoPilot(self,vehicle):
        if self.carla_tm and vehicle.isAutoPilot:
            vehicle.actor.set_autopilot(False)
            vehicle.isAutoPilot=False
            vehicle.actor.set_simulate_physics(False)


    def updateVeh(self):
        """
        update vehicles' actions in carla
        """
        for vehicle in self.vehicles:
            if vehicle.trajectory:
                self.shutdownAutoPilot(vehicle)#use trajectory instead autopilot maybe need to be set in getsce,because of the bug
                vehicle.state=vehicle.trajectory.states[0]
                centerx, centery, yaw, speed, accel = vehicle.trajectory.pop_last_state()
                self.v_actors[vehicle.id].set_transform(carla.Transform(location=carla.Location(x=centerx, y=centery, z=0),
                                                   rotation=carla.Rotation(roll=0,pitch=0,yaw=np.rad2deg(yaw))))
    def removeArrivedVeh(self):
        needRemoved=[]
        for veh in self.vehicles:
            if veh!=self.ego and (veh.arrive_destination() or not veh.actor.is_active):
                needRemoved.append(veh)
                continue
            elif veh!=self.ego and veh.actor.is_active:
                cur_wp=self.carla_map.get_waypoint(veh.actor.get_location())
                if cur_wp.lane_type!=carla.libcarla.LaneType.Driving:
                    needRemoved.append(veh)
                    continue

        for veh in needRemoved:
            self.vehicles.remove(veh)

            try:
                veh.actor.destroy()
            except:
                pass#already destroyed

            del self.v_actors[veh.id]
            if veh.id in self.vehINAoI.keys():
                del self.vehINAoI[veh.id]
            if veh.id in self.outOfAoI.keys():
                del self.outOfAoI[veh.id]
            del veh

    def destroy(self):
        # ------- close carla sync mode ------- #
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        # self.dbBridge.close()

    def initEgo(self):
        blueprint_library=self.world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))

        if self.cfg['ego_path']!='None':
            #TODO：检查一下route的格式，解析他，设计ego_path的格式
            raise NotImplementedError
        
        spawn_success=False
        while not spawn_success:
            start_waypoint = self.createSpawnPoints(k=1)[0]
            actor = self.world.try_spawn_actor(vehicle_bp, start_waypoint)
            if actor:
                spawn_success=True
        self.world.tick()
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
                vehicle_bp = random.choice(blueprint_library.filter('vehicle.mercedes.*'))
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
            if next_edge_id in route:
                break

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
        return randomSpawnPoints

    @property
    def vehiclesDict(self)->Dict:
        vehiclesDict={}
        for veh in self.vehicles:
            vehiclesDict[veh.id]=veh
        return vehiclesDict

    def setTrajectories(self,trajectories):
        vehiclesDict=self.vehiclesDict
        for k, v in trajectories.items():
            if k == self.ego_id:
                self.ego.trajectory = v
            else:
                veh = vehiclesDict[k]
                veh.trajectory = v
        for k,veh in vehiclesDict.items():
            if k in trajectories.keys():
                continue
            else:
                veh.trajectories=None
                

    def exportSce(self):
        if not self.tpStart:
            return None,None
        vehicles = {
            'egoCar': self.ego.export2Dict(self.roadgraph),
            'carInAoI': [av.export2Dict(self.roadgraph) for av in self.vehINAoI.values()],
            'outOfAoI': [sv.export2Dict(self.roadgraph) for sv in self.outOfAoI.values()]
        }
        return self.roadgraph,vehicles

    def updateAvailableLane(self):
        for veh in self.vehicles:
            try:
                if not veh.available_lanes and veh.route:
                    veh.available_lanes=model.roadgraph.get_all_available_lanes(veh.route, veh.end_waypoint)
                if veh.available_lanes and veh.route:
                        veh.next_available_lanes = veh.get_available_lanes(model.roadgraph)#help localization and egoplan
            except:
                self.resetRoute(veh)
                veh.available_lanes=model.roadgraph.get_all_available_lanes(veh.route, veh.end_waypoint)
                veh.next_available_lanes = veh.get_available_lanes(model.roadgraph)#help localization and egoplan

if __name__=='__main__':
    config_name='./simModel_Carla/example_config.yaml'
    random.seed(112)

    stringTimestamp = datetime.strftime(datetime.now(), '%Y-%m-%d_%H-%M-%S')    
    database = 'results/' + stringTimestamp + '.db'

    model=Model(cfgFile=config_name,dataBase=database)
    egoplanner = LLMEgoPlanner()
    planner = TrafficManager(model)

    model.start()

    for item in model.roadgraph.Edges.items():
        model.world.debug.draw_string(item[1].last_segment[0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)
    
    model.runAutoPilot()

    gui = GUI(model)
    gui.start()

    while not model.tpEnd:
        model.moveStep()#TODO:add collisioncheck
        if model.shouldUpdate():

            roadgraph, vehicles = model.exportSce()

            trajectories = planner.plan(
                model.timeStep * 0.1, roadgraph, vehicles, model.ego.behaviour, other_plan=True
            )

            # trajectory = egoplanner.plan(model.ego, model.roadgraph, None, model.timeStep)
            # trajectories[model.ego_id]=trajectory

            model.setTrajectories(trajectories)

            print(model.ego.lane_id)
            print(model.ego.behaviour)

            world=model.world
            for veh in model.vehicles:
                if veh.trajectory:
                    for state in veh.trajectory.states:
                        world.debug.draw_point(carla.Location(x=state.x,y=state.y,z=0.5),color=carla.Color(r=0, g=0, b=255), life_time=1, size=0.1)

        model.updateVeh()

    model.destroy()
    gui.terminate()
    gui.join()

