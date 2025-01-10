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
from simInfo.CustomExceptions import (
    CollisionChecker, CollisionException, 
    record_result, LaneChangeException, 
    BrainDeadlockException, TimeOutException
)
import imageio
from simModel_Carla.Camera import CAMActor
from leaderboard_util import route_transform
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
from threading import Thread, Lock
import math
class Model:
    def __init__(self,cfgFile: str=None,rouFile: str=None,dataBase: str=None,port:int=2000,tm_port:int=2006):
        # --------- config ---------#
        self.cfgFile:str=cfgFile
        self.cfg:Dict=load_config(self.cfgFile)
        self.rouFile=rouFile
        self.sim_mode: str = 'RealTime'

        os.makedirs('/'.join(dataBase.split('/')[:-1]),exist_ok=True)    
        
        if dataBase:
            self.dataBase = dataBase
        else:
            self.dataBase = datetime.strftime(
                datetime.now(), '%Y-%m-%d_%H-%M-%S') + '_egoTracking' + '.db'

        if os.path.exists(self.dataBase):
            os.remove(self.dataBase)
            
        
        self.dbBridge = DBBridge(self.dataBase)
        self.dbBridge.createTable()

        # --------- init carla and roadgraph --------- #
        self.carla_port=port
        self.client = carla.Client('localhost', port)
        self.client.set_timeout(50.0)
        self.client.load_world(self.cfg['map_name'])
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        self.topology = self.carla_map.get_topology()
        self.spectator = self.world.get_spectator()
        self.carla_tm=None
        self.tm_port=tm_port

        self.roadgraph: RoadGraph = RoadGraph()
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
        self.max_steps=self.cfg['max_steps']  # 从配置文件读取最大时间步限制

        self.renderQueue = RenderQueue(5)
        self.imageQueue = ImageQueue(50)
        self.QAQ = QAQueue(5)

        self.imageExtractor=nuScenesImageExtractor()
        
        #collision state
        self.collision_sensor=None
        self.laneInvasion_sensor=None
        self.collision=False
        self.laneInvation=False

        self.collision_opponent=None
        self.laneInvationType=None

        # tpStart marks whether the trajectory planning is started,
        # when the ego car appears in the network, tpStart turns into 1.
        self.tpStart = 0
        # tpEnd marks whether the trajectory planning is end,
        # when the ego car leaves the network, tpEnd turns into 1.
        self.tpEnd = 0
        self.tpEnd_lock = Lock()
        
        self.BEV_Camera=None
    def start(self):
        # ---------- init actors --------- #
        #TODO:根据随机初始化结果输出rouFile,且思考rouFile后续如何使用
        self.ego,self.ego_id=self.initEgo()
        self.vehicles=self.initVehicles()
        self.pedestrians=self.initPedestrians()
        self.cycles=self.initCycles()

        self.vehicles.append(self.ego)
        
        for veh in self.vehicles:
            while True:
                try:
                    veh.available_lanes=self.roadgraph.get_all_available_lanes(veh.route, veh.end_waypoint)
                    break
                except Exception as e:
                    print(f"Error getting available lanes for vehicle {veh.id}: {e}")
                    route,end_waypoint=self.randomRoute(veh.start_waypoint)
                    veh.route=route 
                    veh.end_waypoint=end_waypoint
            
        # self.simDescriptionCommit()
        # --------- carla sync mode ---------- #
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.cfg['delta_seconds']#0.1
        self.dt=self.cfg['delta_seconds']
        self.world.apply_settings(settings)
        
        self.collision_sensor.listen(lambda event:self.violationState(event))
        self.laneInvasion_sensor.listen(lambda event:self.violationState(event))
        self.world.tick()

        self.timeStep=0

    def simDescriptionCommit(self,route_length=0):
        # Calculate route length using ego's route and available lanes
        # gps_route,route=route_transform(self.roadgraph,self.ego)
        # route_length = 0.0 
        # for idx,wp in enumerate(route):
        #     if idx!=len(route)-1:
        #         route_length+=wp[0].location.distance(route[idx+1][0].location)
        # route_length = 0.0
        # available_lanes = self.roadgraph.get_all_available_lanes(self.ego.route, self.ego.end_waypoint)
        
        # for edge_id in self.ego.route:
        #     edge_info = available_lanes[edge_id]
        #     edge = self.roadgraph.Edges[edge_id]
            
        #     # 获取当前edge中所有section的ID，按照顺序排列
        #     section_list = edge.section_list
            
        #     # 遍历每个section
        #     for section_id in section_list:
        #         section = self.roadgraph.Sections[section_id]
                
        #         # 如果这个section在available_lane中
        #         if section_id in edge_info['available_lane']:
        #             lanes = edge_info['available_lane'][section_id]
        #             if lanes:
        #                 lane = lanes[0]  # 取第一条车道作为代表
        #                 # 检查是否是终点所在的section
        #                 if (lane.start_wp.road_id, lane.start_wp.section_id) == (self.ego.end_waypoint.road_id, self.ego.end_waypoint.section_id):
        #                     # 找到终点所在的具体车道
        #                     end_lane = None
        #                     for l in lanes:
        #                         if l.start_wp.lane_id == self.ego.end_waypoint.lane_id:
        #                             end_lane = l
        #                             break
        #                     if end_lane:
        #                         route_length += self.ego.end_waypoint.s
        #                     else:
        #                         route_length += lane.length
        #                 else:
        #                     route_length += lane.length
        #         # 如果这个section不在available_lane中，但在change_lane中
        #         elif section_id in edge_info['change_lane']:
        #             lanes = edge_info['change_lane'][section_id]
        #             if lanes:
        #                 lane = lanes[0]
        #                 if (lane.start_wp.road_id, lane.start_wp.section_id) == (self.ego.end_waypoint.road_id, self.ego.end_waypoint.section_id):
        #                     # 找到终点所在的具体车道
        #                     end_lane = None
        #                     for l in lanes:
        #                         if l.start_wp.lane_id == self.ego.end_waypoint.lane_id:
        #                             end_lane = l
        #                             break
        #                     if end_lane:
        #                         route_length += self.ego.end_waypoint.s
        #                     else:
        #                         route_length += lane.length
        #                 else:
        #                     route_length += lane.length
            
        #     # 如果有junction_lane，也要加上
        #     for junction_lane in edge_info['junction_lane']:
        #         route_length += junction_lane.length
        print('route_length:',route_length)
        currTime = datetime.now()
        insertQuery = '''INSERT INTO simINFO VALUES (?, ?, ?, ?);'''
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            insertQuery,
            (currTime, self.ego.id, self.cfg['map_name'], route_length)
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
        #commit mapdata to db
        self.roadgraph.getDataQue()
        Th = Thread(target=self.roadgraph.insertCommit,args=(self.dataBase,))
        Th.start()

    def setAutoPilot(self,vehicle:Vehicle):
        vehicle.actor.set_simulate_physics(True)
        vehicle.actor.set_autopilot(True,self.tm_port)
        vehicle.isAutoPilot=True

        vehicle.route=None#vehicle controled by autopilot cant follow the given route
        vehicle.available_lanes=dict()
        
        self.carla_tm.auto_lane_change(vehicle.actor,True)
        #set_path function is not working properly
        # route_carla=self.routeTransform(vehicle)
        # self.carla_tm.set_path(vehicle.actor,route_carla)

    def runAutoPilot(self):
        self.carla_tm = self.client.get_trafficmanager(self.tm_port)
        self.tm_port = self.carla_tm.get_port()
        print("tm_port:", self.tm_port)
        self.carla_tm.set_synchronous_mode(True)

        for vehicle in self.vehicles:
            if vehicle.id!=self.ego_id and not vehicle.isAutoPilot:
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
        if spawn_point.location.distance(self.ego.actor.get_location())<50:
            return False
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
        
        # 检查是否超过最大时间步限制
        if self.timeStep >= self.max_steps:
            print('[cyan]已达到最大时间步限制.[/cyan]')
            with self.tpEnd_lock:
                self.tpEnd = 1
            return

        self.getSce()
        if self.shouldUpdate():
            self.putRenderData()
            if self.cfg['carlaImage']:
                self.putCARLAImage()

        if not self.tpStart:
            self.tpStart = 1
                
        if self.timeStep%20==0:
            self.saveBEV()

        if self.shouldSpawnVeh():
            spawn_success=self.spawnVehicleRuntime()
            print('spawn_success:',spawn_success)

    def saveBEV(self):
        pic=self.BEV_Camera.getImage()
        full_path='/'.join(self.dataBase.split('/')[:-1])+f'/{self.timeStep}.png'
        imageio.imwrite(full_path,pic,compress_level=3)
        
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
            self.putTrafficLightINFO()
        else:
            if self.tpStart:
                print('[cyan]The ego car has reached the destination.[/cyan]')
                with self.tpEnd_lock:
                    self.tpEnd = 1
                    self.saveBEV()
    
    def putTrafficLightINFO(self):
        # update all traffic lights junction states in the network
        for jid,junction_lane in self.roadgraph.Junction_Dict.items():
            if junction_lane.traffic_light==None:
                continue
            tls=junction_lane.currTlState
            self.dbBridge.putData(
                'trafficLightStates',
                (self.timeStep, jid,tls)
            )
    
    def getVehInfo(self,vehicle:Vehicle):
        if vehicle.id not in self.allvTypes:
            vtins=vehType(str(vehicle.id))
            vtins.maxAccel=3#为了让背景车能够正确刹车，不追尾
            vtins.maxDecel=6
            vtins.maxSpeed=vehicle.actor.get_speed_limit()#TODO:可能有问题，逻辑有问题
            vtins.length=vehicle.length
            vtins.width=vehicle.width
            vtins.vclass=vehicle.actor.type_id
            routes =  '' if not vehicle.route else ' '.join(vehicle.route)
            self.commitVehicleInfo(str(vehicle.id), vtins, routes)
            self.allvTypes[vehicle.id]=vtins

        actor=self.v_actors[vehicle.id]

        x=vehicle.state.x = actor.get_location().x
        y=vehicle.state.y = actor.get_location().y
        z=vehicle.state.z = actor.get_location().z
        yaw=vehicle.state.yaw = np.deg2rad(actor.get_transform().rotation.yaw)

        vehicle.lane_id,vehicle.state.s,vehicle.state.d,vehicle.cur_wp=self.localization(vehicle)
        
        #whether the vehicle is in the AoI
        if vehicle.id !=self.ego_id:
            ex,ey=self.ego.state.x,self.ego.state.y
            if sqrt(pow((ex - x), 2) + pow((ey - y), 2)) <= self.cfg['deArea']:
                #如果自车在范围内
                if not vehicle.id in self.vehINAoI.keys():#如果车辆不在AoI中
                    #如果车辆在junction或者道路末端，则不加入AoI
                    lane=self.roadgraph.get_lane_by_id(vehicle.lane_id)
                    if not (vehicle.lane_id in self.roadgraph.Junction_Dict.keys() or (vehicle.state.s > lane.course_spline.s[-1]-10)):
                        #如果车辆不在junction或者道路末端，则加入AoI
                        self.vehINAoI[vehicle.id]=vehicle
                        if vehicle.id in self.outOfAoI.keys():
                            del self.outOfAoI[vehicle.id]
                else:
                    pass
            else:
                if vehicle.id in self.vehINAoI.keys():
                    del self.vehINAoI[vehicle.id]
                if not vehicle.id in self.outOfAoI.keys():
                    self.outOfAoI[vehicle.id] = vehicle

        
        #update lane speed limit
        current_lane = self.roadgraph.get_lane_by_id(vehicle.lane_id)
        current_lane.speed_limit = vehicle.actor.get_speed_limit()/3.6 #m/s
        
        #检查ego车辆是否在正常行驶车道上
        check_wp = self.carla_map.get_waypoint(carla.Location(x=vehicle.state.x, y=vehicle.state.y, z=z), project_to_road=False)
        if vehicle.id == self.ego_id:
            if self.shouldUpdate():
                print('ego actor speed:', vehicle.actor.get_velocity().length(),'ego vehicle speed:', vehicle.state.vel)
                print('ego actor accel:', vehicle.actor.get_acceleration().length(),'ego vehicle accel:', vehicle.state.acc)
            if check_wp is None or check_wp.lane_type != carla.LaneType.Driving and not self.laneInvation:
                self.laneInvation = True
            elif check_wp is None or check_wp.lane_type != carla.LaneType.Driving and self.laneInvation:
                self.laneInvationType = None if check_wp is None else check_wp.lane_type
                print('Vehicle left normal driving lane')
                with self.tpEnd_lock:
                    self.tpEnd = 1
            else:
                self.laneInvation = False
                self.laneInvationType = None
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

        carla_vel=vehicle.actor.get_velocity()

        vehicle.xQ.append(x)
        vehicle.yQ.append(y)
        vehicle.yawQ.append(yaw)
        vehicle.routeIdxQ.append(route_idx)
        vehicle.laneIDQ.append(vehicle.lane_id)
        vehicle.lanePosQ.append(vehicle.state.s)
        vehicle.lanePosDQ.append(vehicle.state.d)
        
        if vehicle.isAutoPilot:
            vehicle.speedQ.append(carla_vel.length())
            vehicle.accelQ.append(vehicle.actor.get_acceleration().length())#TODO:如果是减速度，不会被体现出来
        else:
            vehicle.speedQ.append(carla_vel.length())
            vehicle.accelQ.append(vehicle.actor.get_acceleration().length())
            # vehicle.speedQ.append(vehicle.state.vel)
            # vehicle.accelQ.append(vehicle.state.acc)
        if vehicle.id == self.ego_id:
            print('vel diff carla and state:',carla_vel.length()-vehicle.state.vel)
        vehicle.vxQ.append(carla_vel.x)
        vehicle.vyQ.append(carla_vel.y)

        vehicle.state.vel = carla_vel.length()

    def localization(self,vehicle):
        cur_lane = self.roadgraph.get_lane_by_id(vehicle.lane_id)
        cur_wp=vehicle.cur_wp if vehicle.cur_wp else self.carla_map.get_waypoint(carla.Location(x=vehicle.state.x, y=vehicle.state.y))
        carla_wp=self.carla_map.get_waypoint(carla.Location(x=vehicle.state.x, y=vehicle.state.y))#Default setting: wp always is on Driving lane

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
            next_s, next_d = self.roadgraph.get_lane_by_id(next_lane_id).course_spline.cartesian_to_frenet1D(vehicle.state.x, vehicle.state.y)
            next_s = max(0, next_s)
            next_wp=self.carla_map.get_waypoint_xodr(junction_lane.start_wp.road_id,junction_lane.start_wp.lane_id,next_s)
            return next_lane_id,next_s,next_d,next_wp

        elif not carla_wp.is_junction:
            next_lane_id=self.roadgraph.WP2Lane[(carla_wp.road_id,carla_wp.section_id,carla_wp.lane_id)]
            next_s, next_d = self.roadgraph.get_lane_by_id(
                               next_lane_id).course_spline.cartesian_to_frenet1D(vehicle.state.x, vehicle.state.y)
            next_s = max(0, next_s)
            next_wp=carla_wp
            return next_lane_id,next_s,next_d,next_wp
                        
        raise NotImplementedError 

    def commitFrameInfo(self, vid: int, vtag: str, veh: Vehicle):
        self.dbBridge.putData(
            'frameINFO',
            (
                self.timeStep, vid, vtag, veh.state.x, veh.state.y, veh.state.yaw, veh.speedQ[-1],
                veh.accelQ[-1], veh.lane_id, veh.state.s, veh.routeIdxQ[-1],','.join(veh.next_available_lanes)
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
            if vehicle.id==self.ego_id:
                print("shutdown autopilot for ego")
            # vehicle.actor.set_autopilot(False,self.tm_port)#检查只靠关闭物理能否disable autopilot，不能
            vehicle.isAutoPilot=False
            vehicle.actor.set_simulate_physics(False)


    def updateVeh(self):
        """
        update vehicles' actions in carla
        """
        for vehicle in self.vehicles:
            if vehicle.trajectory:
                self.shutdownAutoPilot(vehicle)#use trajectory instead autopilot maybe need to be set in getsce,because of the bug
                vehicle.state=vehicle.trajectory.states[0]#TODO: 会导致state和actor的实际状态不一致
                centerx, centery, yaw, speed, accel = vehicle.trajectory.pop_last_state()
                self.v_actors[vehicle.id].set_transform(carla.Transform(location=carla.Location(x=centerx, y=centery, z=0),
                                                   rotation=carla.Rotation(roll=0,pitch=0,yaw=np.rad2deg(yaw))))
                vehicle.actor.set_target_velocity(carla.Vector3D(x=speed*math.cos(yaw),y=speed*math.sin(yaw),z=0))
            if not vehicle.trajectory and not vehicle.isAutoPilot and vehicle.id!=self.ego_id:
                self.setAutoPilot(vehicle)
            if vehicle.control:
                self.shutdownAutoPilot(vehicle)
                vehicle.actor.apply_control(vehicle.control)
    def removeArrivedVeh(self):
        needRemoved=[]
        for veh in self.vehicles:
            if veh!=self.ego and ((veh.actor.is_active and veh.arrive_destination()) or not veh.actor.is_active):
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
        """安全清理所有资源"""
        try:
            # 关闭carla同步模式
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

            # 清理所有传感器
            if self.collision_sensor:
                self.collision_sensor.destroy()
            if self.laneInvasion_sensor:
                self.laneInvasion_sensor.destroy()
            if self.BEV_Camera:
                self.BEV_Camera.cam.destroy()

            # 清理所有vehicle actors
            for actor_id in list(self.v_actors.keys()):
                try:
                    if self.v_actors[actor_id].is_alive:
                        self.v_actors[actor_id].destroy()
                except:
                    pass  # 如果actor已经被销毁则忽略
            self.v_actors.clear()

            # 清理pedestrian actors
            for actor_id in list(self.p_actors.keys()):
                try:
                    if self.p_actors[actor_id].is_alive:
                        self.p_actors[actor_id].destroy()
                except:
                    pass
            self.p_actors.clear()

            # 清理cycle actors
            for actor_id in list(self.c_actors.keys()):
                try:
                    if self.c_actors[actor_id].is_alive:
                        self.c_actors[actor_id].destroy()
                except:
                    pass
            self.c_actors.clear()

            # 清理车辆列表
            self.vehicles.clear()
            self.pedestrians.clear()
            self.cycles.clear()
            self.vehINAoI.clear()
            self.outOfAoI.clear()

            # 安全关闭数据库连接
            if hasattr(self, 'dbBridge'):
                self.dbBridge.close()

            # 清理交通管理器
            if self.carla_tm:
                self.carla_tm = None

        except Exception as e:
            print(f"在清理资源时发生错误: {str(e)}")
            # 确保数据库连接被关闭
            if hasattr(self, 'dbBridge'):
                self.dbBridge.close()

    def initEgo(self):
        blueprint_library=self.world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.lincoln.*'))#vehicle.tesla.*
        vehicle_bp.set_attribute('role_name','hero')

        start_idx_sp=None
        if self.cfg['preset_ego_route']!='None':
            route = self.cfg['preset_ego_route']
            if isinstance(route, list):
                if len(route) == 2 and isinstance(route[0], (int, str)) and str(route[0]).isdigit():
                    # 索引格式: [247, 41]
                    start_idx_sp, end_idx_sp = map(int, route)
                elif len(route) == 6:  # 坐标格式分割成6个部分
                    # 坐标格式: ['(-60.490871', 239.699402, '0)', '(32.382332', 237.53667, '0)']
                    # 处理起始点
                    start_x = float(route[0].strip('('))
                    start_y = float(route[1])
                    start_z = float(route[2].strip(')'))
                    start_loc = carla.Location(x=start_x, y=start_y, z=start_z)
                    
                    # 处理终点
                    end_x = float(route[3].strip('('))
                    end_y = float(route[4])
                    end_z = float(route[5].strip(')'))
                    end_loc = carla.Location(x=end_x, y=end_y, z=end_z)
                    
                    start_waypoint = self.carla_map.get_waypoint(start_loc)
                    start_transform = start_waypoint.transform
                    start_transform = carla.Transform(start_transform.location+carla.Location(z=0.3), carla.Rotation(yaw=start_transform.rotation.yaw))
            else:
                # 如果是其他格式
                start_idx_sp,end_idx_sp=map(int,self.cfg['preset_ego_route'])
            
        spawn_success=False
        c = 0
        while not spawn_success:
            if c > 1000:
                raise RuntimeError("生成主车失败:尝试次数超过1000次")
            if start_idx_sp is not None:
                start_waypoint = self.createSpawnPoints(k=1,idx=start_idx_sp)[0]
            actor = self.world.try_spawn_actor(vehicle_bp, start_waypoint if start_idx_sp is not None else start_transform)
            if actor:
                spawn_success=True
            c += 1
            
        self.world.tick()
        self.v_actors[actor.id]=actor

        start_waypoint=self.carla_map.get_waypoint(actor.get_location())
        
        if self.cfg['preset_ego_route']=='None':
            route, end_waypoint = self.randomRoute(start_waypoint)
        else:
            if len(route) == 6:
                # 如果是坐标输入，直接使用end_loc
                end_waypoint = self.carla_map.get_waypoint(end_loc)
            else:
                # 原有的索引方式
                end_waypoint=self.createSpawnPoints(k=1,idx=end_idx_sp)[0]
                end_waypoint=self.carla_map.get_waypoint(end_waypoint.location)
            
            while end_waypoint.is_junction:
                nxt_wps=end_waypoint.next(1)
                end_waypoint=nxt_wps[0]
                
            grp = GlobalRoutePlanner(self.carla_map, 0.5)
            route_carla = grp.trace_route(start_waypoint.transform.location, end_waypoint.transform.location)
            route = self.roadgraph.get_route_edge(route_carla)
            
        vehicle = Vehicle(actor, start_waypoint, end_waypoint, self.vehicleVisbleRange, route)
        vehicle.get_route(self.carla_map, self.roadgraph)
        
        #添加碰撞传感器
        self.collision_sensor=self.world.spawn_actor(self.world.get_blueprint_library().find("sensor.other.collision"),carla.Transform(),attach_to=actor)
        self.laneInvasion_sensor=self.world.spawn_actor(self.world.get_blueprint_library().find("sensor.other.lane_invasion"),carla.Transform(),attach_to=actor)
        self.world.tick()
        
        #添加俯视图相机
        #TODO:代码移植到CAMActor.py中
        cameraInfo = {
            'CAM_ABOVE': {
                'transform': carla.Transform(
                    carla.Location(x=0, y=0, z=150),
                    carla.Rotation(yaw=0, pitch=-90, roll=0)
                ),
                'fov': '70',
                'record': False
            }
        }
        for k, v in cameraInfo.items():
            if not v['record']:
                self.BEV_Camera = CAMActor(
                    self.world,k,v['transform'],v['fov'],vehicle
                )
                v['record'] = True
            else:
                continue
            
        return vehicle,actor.id

    def violationState(self,event):
        if event and not self.tpEnd:
            if isinstance(event,carla.CollisionEvent):
                print('collision detected')
                self.collision=True#TODO: after collision, the model should stop
                other_actor=event.other_actor
                self.collision_opponent=other_actor.id
                with self.tpEnd_lock:
                    self.tpEnd=1
                    self.saveBEV()
            elif isinstance(event,carla.LaneInvasionEvent):
                marking_type = event.crossed_lane_markings[0].type
                print('marking_type:',marking_type)
                #TODO:when cross solid, should be punished
                if marking_type == carla.LaneMarkingType.Grass or marking_type == carla.LaneMarkingType.Curb:
                    self.laneInvation=True
                    self.laneInvationType=marking_type
                    print('laneInvasion type:',self.laneInvationType)
                    with self.tpEnd_lock:
                        self.tpEnd=1
                        self.saveBEV()
                
    
    def initVehicles(self):
        blueprint_library=self.world.get_blueprint_library()

        spawn_points = self.createSpawnPoints(self.cfg['init']['veh_num'])
        for i in range(self.cfg['init']['veh_num']):
            try:
                blueprint_garage=['vehicle.mercedes.*','vehicle.audi.*','vehicle.bmw.*']
                vehicle_bp = random.choice(blueprint_library.filter(random.choice(blueprint_garage)))
                actor=self.world.spawn_actor(vehicle_bp, spawn_points[i])
                self.v_actors[actor.id]=actor
            except RuntimeError:
                pass

        vehicles=[]
        for id, actor in self.v_actors.items():
            if id==self.ego_id:
                continue
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

    def createSpawnPoints(self,k:int,idx=None)->list:
        spawn_points = self.carla_map.get_spawn_points()
        assert k<= len(spawn_points)
        if not idx:
            randomSpawnPoints=random.sample(spawn_points,k)
        else:
            randomSpawnPoints=[spawn_points[idx]]
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
                veh.trajectory=None
    
    def setControls(self,controls):
        for k,v in controls.items():
            if k==self.ego.id:
                self.ego.control=v
            else:
                self.vehiclesDict[k].control=v
        for k,veh in self.vehiclesDict.items():
            if k in controls:
                continue
            else:
                veh.control=None

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
                    veh.available_lanes=self.roadgraph.get_all_available_lanes(veh.route, veh.end_waypoint)
                if veh.available_lanes and veh.route:
                        veh.next_available_lanes = veh.get_available_lanes(self.roadgraph)#help localization and egoplan
            except:
                try:    
                    self.resetRoute(veh)
                    veh.available_lanes=self.roadgraph.get_all_available_lanes(veh.route, veh.end_waypoint)
                    veh.next_available_lanes = veh.get_available_lanes(self.roadgraph)#help localization and egoplan
                except:
                    pass
    def record_result(self, start_time: float, result: bool, reason: str = "", error: Exception = None) -> None:
        # self.roadgraph.wp_transform(self.carla_map)
        # self.roadgraph.clear_traffic_lights()
        # with open(self.map_cache_path, 'wb') as f:
        #     dill.dump(self.roadgraph, f)
        # print('save roadgraph speed limits info')#TODO:并行运行时，可能保存多个roadgraph，导致限速信息还是无法正确传递
        
        
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        if self.collision:
            reason+=f'collision happened with {self.collision_opponent}'
        if self.laneInvation:
            reason+=f'laneInvation happened with {self.laneInvationType}'
        if self.timeStep >= self.max_steps:
            reason+=f'reached maximum time steps ({self.max_steps})'
        # add result data
        cur.execute(
            """INSERT INTO resultINFO (
                egoID, result, total_score, complete_percentage, drive_score, use_time, fail_reason
                ) VALUES (?,?,?,?,?,?,?);""",
            (
                str(self.ego_id), result, 0, 0, 0, time.time() - start_time, reason
            )
        )
        conn.commit()
        time.sleep(5)
        conn.close()
        return 

if __name__=='__main__':
    from simInfo.EnvDescriptor import EnvDescription
    from carlaWrapper import carlaRoadGraphWrapper
    descriptor=EnvDescription()

    config_name='./simModel_Carla/exp_config/long_term_config.yaml'
    random.seed(11210238)

    stringTimestamp = datetime.strftime(datetime.now(), '%Y-%m-%d_%H-%M-%S')    
    database = 'results/' + stringTimestamp + '.db'
    total_start_time = time.time()

    model=Model(cfgFile=config_name,dataBase=database,port=3000,tm_port=1112)
    # model:Model=Model(cfgFile=config_name,dataBase=database,port=int(arguments.port),tm_port=int(arguments.trafficManagerPort))
    egoplanner = LLMEgoPlanner()
    planner = TrafficManager(model)

    model.start()

    for item in model.roadgraph.Edges.items():
        model.world.debug.draw_string(item[1].last_segment[0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)
    
    model.runAutoPilot()

    gui = GUI(model)
    gui.start()

    model.record_result(total_start_time, True, None)

    while not model.tpEnd:
        model.moveStep()
        if model.shouldUpdate():

            roadgraph, vehicles = model.exportSce()

            trajectories = planner.plan(
                model.timeStep * 0.1, roadgraph, vehicles, model.ego.behaviour, other_plan=True
            )

            start_time=time.time()
            
            navInfo = descriptor.getNavigationInfo(carlaRoadGraphWrapper(roadgraph), vehicles)
            actionInfo = descriptor.getAvailableActionsInfo(carlaRoadGraphWrapper(roadgraph), vehicles)
            envInfo = descriptor.getEnvPrompt(carlaRoadGraphWrapper(roadgraph), vehicles)

            current_QA = QuestionAndAnswer(envInfo, navInfo, actionInfo, '', '', 0, 0, 0, time.time()-start_time, model.ego.behaviour)
            model.putQA(current_QA)
         
            model.setTrajectories(trajectories)

            print(model.ego.lane_id)
            print(model.ego.behaviour)

            world=model.world
            for veh in model.vehicles:
                if veh.trajectory:
                    for state in veh.trajectory.states:
                        world.debug.draw_point(carla.Location(x=state.x,y=state.y,z=0.5),color=carla.Color(r=0, g=0, b=255), life_time=1, size=0.1)

        model.updateVeh()

    #according to collision sensor
    model.record_result(total_start_time, True, None)

    model.destroy()
    gui.terminate()
    gui.join()

