from RoadInfoGet import RoadInfoGet
from Roadgraph import RoadGraph
from vehicle import Vehicle
from ego_vehicle_planning import LLMEgoPlanner

import carla
from carla import Location, Rotation, Transform
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner

from utils.trajectory import State
import time, copy
import numpy as np
import sys

# --------- init carla and roadgraph ------------ #
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()
carla_map = world.get_map()
topology = carla_map.get_topology()
spectator = world.get_spectator()

roadgraph = RoadGraph()
planner = LLMEgoPlanner()
roadInfoGet = RoadInfoGet(roadgraph, topology)

# --------- init route ------------ #
spawn_points = carla_map.get_spawn_points()
origin = spawn_points[20].location
destination = spawn_points[100].location

world.debug.draw_point(destination, size=0.1, color=carla.Color(0,0,255), life_time=1000)

grp = GlobalRoutePlanner(carla_map, 0.5)
route = grp.trace_route(origin, destination)
path_list = roadgraph.get_route_edge(route)

# --------- init ego vehicle ------------ #
for actor in world.get_actors():
    if 'vehicle' in actor.type_id:
        actor.destroy()

transform = Transform(origin, Rotation(yaw=180))
vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
actor = world.spawn_actor(vehicle_bp, spawn_points[20])

# --------- carla sync mode ---------- #
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.1
world.apply_settings(settings)
world.tick()

# create Vehicle object
start_waypoint = carla_map.get_waypoint(origin)
end_waypoint = carla_map.get_waypoint(destination)
cur_lane_id = roadgraph.WP2Lane[(start_waypoint.road_id, start_waypoint.section_id, start_waypoint.lane_id)]
ego_veh = Vehicle(actor.id, path_list, start_waypoint, end_waypoint, 100, State(x = origin.x, y = origin.y, yaw = origin.z), cur_lane_id)
ego_veh.state.s, _ = roadgraph.get_lane_by_id(ego_veh.lane_id).course_spline.cartesian_to_frenet1D(ego_veh.state.x, ego_veh.state.y)
ego_veh.available_lanes = roadgraph.get_all_available_lanes(ego_veh.route, ego_veh.end_waypoint)

# --------- main loop ------------ #

t = 0
ego_veh.next_available_lanes = ego_veh.get_available_lanes(roadgraph)
ego_veh.trajectory = planner.plan(ego_veh, roadgraph, None, t)

while True:
    t+=1
    if ego_veh.arrive_destination():
        print("The target has been reached, stopping the simulation")
        break

    ego_veh.state = ego_veh.trajectory.states[0]
    centerx, centery, yaw, speed, accel = ego_veh.trajectory.pop_last_state()
    actor.set_transform(carla.Transform(location = carla.Location(x=centerx, y=centery, z=0), rotation = carla.Rotation(yaw=np.rad2deg(yaw))))

    spectator.set_transform(carla.Transform(actor.get_transform().location + carla.Location(z=100), carla.Rotation(pitch=-90)))

    world.tick()

    # TODO: 停顿时间太长了，出现卡顿，从log里面读取应该是连续的
    # NOTICE: carla提供的waypoint中的s不可信，需要自己计算
    if t % 10 == 0:
        ego_veh.state.x = actor.get_location().x
        ego_veh.state.y = actor.get_location().y
        ego_veh.state.yaw = np.deg2rad(actor.get_transform().rotation.yaw)
        ego_veh.state.s, _ = roadgraph.get_lane_by_id(ego_veh.lane_id).course_spline.cartesian_to_frenet1D(ego_veh.state.x, ego_veh.state.y)

        cur_lane = roadgraph.get_lane_by_id(ego_veh.lane_id)
        ego_veh.state.s = max(0, ego_veh.state.s)

        # ------------ update lane_id and cur_wp ------------ #
        if ego_veh.state.s > cur_lane.length:
            if ego_veh.cur_wp.is_junction:
            # 说明从junction进入edge，这个时候有可能有重叠的waypoint, 先按照上一个lane给出的next_junction作为id
                next_normal_lane = roadgraph.get_lane_by_id(roadgraph.WP2Lane[cur_lane.next_lane])
                ego_veh.lane_id, ego_veh.cur_wp = roadgraph.get_laneID_by_s(next_normal_lane.start_wp.road_id, next_normal_lane.start_wp.lane_id, ego_veh.state.s-cur_lane.length, carla_map)
                while(ego_veh.lane_id == None):
                    # 说明这个长度已经超过了下一个lane了
                    ego_veh.state.s = ego_veh.state.s-cur_lane.length
                    cur_lane = copy.deepcopy(next_normal_lane)
                    if next_normal_lane.next_lane:
                        next_normal_lane = roadgraph.get_lane_by_id(roadgraph.WP2Lane[next_normal_lane.next_lane])
                        ego_veh.lane_id, ego_veh.cur_wp = roadgraph.get_laneID_by_s(next_normal_lane.start_wp.road_id, next_normal_lane.start_wp.lane_id, ego_veh.state.s-cur_lane.length, carla_map)
                    else: # next is junction lane
                        cur_edge_id = next_normal_lane.affiliated_section.affliated_edge.id
                        # 找到cur_lane对应的junction_lane
                        for junction_lane in ego_veh.available_lanes[cur_edge_id]["junction_lane"]:
                            if roadgraph.WP2Lane[junction_lane.previous_lane] == next_normal_lane.id:
                                ego_veh.lane_id = junction_lane.id
                                ego_veh.lane_id, ego_veh.cur_wp = roadgraph.get_laneID_by_s(junction_lane.start_wp.road_id, junction_lane.start_wp.lane_id, ego_veh.state.s-cur_lane.length, carla_map)

                ego_veh.state.s, _ = roadgraph.get_lane_by_id(ego_veh.lane_id).course_spline.cartesian_to_frenet1D(ego_veh.state.x, ego_veh.state.y)
                ego_veh.state.s = max(0, ego_veh.state.s)

            else:
                # 说明从edge进入junction
                cur_edge_id = roadgraph.get_lane_by_id(ego_veh.lane_id).affiliated_section.affliated_edge.id
                for junction_lane in ego_veh.available_lanes[cur_edge_id]["junction_lane"]:
                    # 找到cur_lane对应的junction_lane
                    if roadgraph.WP2Lane[junction_lane.previous_lane] == ego_veh.lane_id:
                        ego_veh.lane_id = junction_lane.id
                        ego_veh.lane_id, ego_veh.cur_wp = roadgraph.get_laneID_by_s(junction_lane.start_wp.road_id, junction_lane.start_wp.lane_id, ego_veh.state.s-cur_lane.length, carla_map)
                        ego_veh.state.s, _ = roadgraph.get_lane_by_id(ego_veh.lane_id).course_spline.cartesian_to_frenet1D(ego_veh.state.x, ego_veh.state.y)
                        ego_veh.state.s = max(0, ego_veh.state.s)
                        break

        if ego_veh.cur_wp.is_junction:
            ego_veh.lane_id, ego_veh.cur_wp = roadgraph.get_laneID_by_s(ego_veh.cur_wp.road_id, ego_veh.cur_wp.lane_id, ego_veh.state.s, carla_map)
            ego_veh.state.s, _ = roadgraph.get_lane_by_id(ego_veh.lane_id).course_spline.cartesian_to_frenet1D(ego_veh.state.x, ego_veh.state.y)
        else:
            ego_veh.lane_id, ego_veh.cur_wp = roadgraph.get_laneID_by_xy(ego_veh.state.x, ego_veh.state.y, carla_map)
            ego_veh.state.s, _ = roadgraph.get_lane_by_id(ego_veh.lane_id).course_spline.cartesian_to_frenet1D(ego_veh.state.x, ego_veh.state.y)

        # ------------- planning ------------ #
        ego_veh.next_available_lanes = ego_veh.get_available_lanes(roadgraph)
        ego_veh.trajectory = planner.plan(ego_veh, roadgraph, None, t)
        print(ego_veh.lane_id)
        print(ego_veh.behaviour)
            

# ------- close carla sync mode ------- #
settings = world.get_settings()
settings.synchronous_mode = False
settings.fixed_delta_seconds = None
world.apply_settings(settings)