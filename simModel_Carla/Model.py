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
import time, copy
import numpy as np
import sys
from typing import Dict, Union, Set, List


class Model:
    def __init__(self):
        # --------- init carla and roadgraph ------------ #
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        self.topology = self.carla_map.get_topology()
        self.spectator = self.world.get_spectator()

        self.roadgraph = RoadGraph()
        self.roadInfoGet = RoadInfoGet(self.roadgraph, self.topology)

        self.ego_id:int =-1
        self.actors:Dict[int,carla.libcarla.Actor]=field(default_factory=dict)
        self.vehicles:list =[]

        self.timeStep:int = 0

        # tpStart marks whether the trajectory planning is started,
        # when the ego car appears in the network, tpStart turns into 1.
        self.tpStart = 0
        # tpEnd marks whether the trajectory planning is end,
        # when the ego car leaves the network, tpEnd turns into 1.
        self.tpEnd = 0

        self.ego:Vehicle=None


    def start(self):
        # --------- init route ------------ #
        #TODO: design a method to generate spawn_points for all vehicles
        spawn_points = self.carla_map.get_spawn_points()
        origin = spawn_points[20].location
        destination = spawn_points[100].location

        self.world.debug.draw_point(destination, size=0.1, color=carla.Color(0, 0, 255), life_time=1000)

        grp = GlobalRoutePlanner(self.carla_map, 0.5)
        route = grp.trace_route(origin, destination)
        path_list = self.roadgraph.get_route_edge(route)

        # --------- init vehicles ------------ #
        #TODO: need to initialize all actors
        for actor in self.world.get_actors():
            if 'vehicle' in actor.type_id:
                actor.destroy()

        vehicle_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        actor = self.world.spawn_actor(vehicle_bp, spawn_points[20])

        # --------- carla sync mode ---------- #
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        self.world.apply_settings(settings)
        self.world.tick()

        # create Vehicle object
        # TODO: need to initialize all vehicles
        start_waypoint = self.carla_map.get_waypoint(origin)
        end_waypoint = self.carla_map.get_waypoint(destination)
        cur_lane_id = self.roadgraph.WP2Lane[(start_waypoint.road_id, start_waypoint.section_id, start_waypoint.lane_id)]
        ego_veh = Vehicle(actor.id, path_list, start_waypoint, end_waypoint, 100,
                          State(x=origin.x, y=origin.y, yaw=origin.z), cur_lane_id)
        ego_veh.state.s, _ = self.roadgraph.get_lane_by_id(ego_veh.lane_id).course_spline.cartesian_to_frenet1D(
            ego_veh.state.x, ego_veh.state.y)
        ego_veh.available_lanes = self.roadgraph.get_all_available_lanes(ego_veh.route, ego_veh.end_waypoint)

        self.vehicles=[ego_veh]
        self.actors={actor.id:actor}
        self.ego_id=actor.id
        self.ego=ego_veh
        self.timeStep=0

    def moveStep(self):
        """
        1.carla simulator tick
        2.get vehicles' info
        """
        self.spectator.set_transform(carla.Transform(self.actors[self.ego_id].get_transform().location + carla.Location(z=100), carla.Rotation(pitch=-90)))
        self.world.tick()

        self.timeStep+=1

        if self.timeStep%10==0:
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
        actor=self.actors[vehicle.id]

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
                self.actors[vehicle.id].set_transform(carla.Transform(location=carla.Location(x=centerx, y=centery, z=0),
                                                    rotation=carla.Rotation(yaw=np.rad2deg(yaw))))
    def destroy(self):
        # ------- close carla sync mode ------- #
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)

        # self.dbBridge.close()

if __name__=='__main__':
    model=Model()
    planner = LLMEgoPlanner()

    model.start()
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

