from Roadgraph import RoadGraph
from Network_Structure import NormalLane, JunctionLane, AbstractLane
import carla
from utils.trajectory import State
from trafficManager.common.vehicle import Behaviour
from collections import deque
from utils.trajectory import State, Trajectory
from typing import List, Set
from trafficManager.common.coord_conversion import cartesian_to_frenet2D

# 每个vehicle自行维护自己的路线信息
class Vehicle:
    def __init__(self, 
                 vehicle_id: int,
                 route: list,
                 start_waypoint: carla.Waypoint,
                 end_waypoint: carla.Waypoint,
                 visible_distance: int = 100,
                 init_state: State = State(),
                 lane_id: str = None):
        """Vehicle class for carla-only simulation

        Args:
            vehicle_id (int): user defined vehicle id
            route (list): vehicle's route, list of edge_id, searched by the route planner in carla
            visible_distance (int, optional): looking forward distance. Defaults to 100.
            init_state (State, optional): the vehicle's init state. Defaults to State().
            lane_id (str, optional): the vehicle's init lane id. Defaults to None.
        """
        self.id = vehicle_id
        self.route = route # edge_id
        self.available_lanes = dict() # all of the available lanes in the route.
                                    # edge_id -> {"available_lane": {lane_id: lane, ...}, "change_lane": {lane_id: lane, ...}, "junction_lane": [lane_id, ...]}
        self.visible_distance = visible_distance # forward looking distance
        self.state = init_state # current state of the vehicle
        self.lane_id = lane_id # current lane id
        self.start_waypoint = start_waypoint
        self.end_waypoint = end_waypoint
        self.cur_wp = start_waypoint # current waypoint

        self.length = 4.5 # vehicle length
        self.behaviour = Behaviour(8) # default bahaviour

        self.trajectory: Trajectory = None # vehicle trajectory
        self.next_available_lanes = set() # next available lanes, just include the available lanes in forward looking distance


    def get_available_lanes(self, roadgraph: RoadGraph) -> Set[str]:
        """根据当前车辆的lane以及visiable_distance，获取当前车辆可见范围内的lane信息(PS: 只考虑到本edge和下一个edge的情况)

        Args:
            roadgraph (RoadGraph): 路网信息

        Returns:
            Set[str]: 当前edge/intersection和下一段的available lane_id
        """
        lane = roadgraph.get_lane_by_id(self.lane_id)
        if isinstance(lane, NormalLane):
            cur_section = lane.affiliated_section
            cur_edge = cur_section.affliated_edge
            if cur_edge.id == self.route[-1]: # if the current edge is the destination
                current_LC = carla.Location(x=self.state.x, y=self.state.y, z=self.state.yaw)
                destination_lc = carla.Location(x=self.end_waypoint.transform.location.x, y=self.end_waypoint.transform.location.y, z=self.end_waypoint.transform.location.z)
                remain_distance = destination_lc.distance(current_LC)
            else:
                remain_distance = self.calcaulate_cur_edge_remain_length(roadgraph)
            # if the remain distance is larger than the visible distance, then the vehicle can drive on the all available lanes
            if self.visible_distance <= remain_distance:
                output = set()
                for _, values in self.available_lanes[cur_edge.id]["available_lane"].items():
                    output.update([lane.id for lane in values])
                return output
            # if the remain distance is less than the visible distance, then the vehicle can only drive on the change lane and junction lane
            else:
                output = set()
                for _, values in self.available_lanes[cur_edge.id]["change_lane"].items():
                    output.update([lane.id for lane in values])
                output.update([lane.id for lane in self.available_lanes[cur_edge.id]["junction_lane"]])
                return output
            
        elif isinstance(lane, JunctionLane):
            cur_edge = roadgraph.Edges[lane.incoming_edge_id]
            next_edge = roadgraph.Edges[lane.outgoing_edge_id]
            output = set()
            output.update([lane.id for lane in self.available_lanes[cur_edge.id]["junction_lane"]])
            if next_edge == self.route[-1]:
                current_LC = carla.Location(x=self.state.x, y=self.state.y, z=self.state.yaw)
                remain_distance = self.end_waypoint.distance(current_LC)
                if self.visible_distance <= remain_distance:
                    for _, values in self.available_lanes[next_edge.id]["available_lane"].items():
                        output.update([lane.id for lane in values])
                    return output
                else:
                    for _, values in self.available_lanes[next_edge.id]["change_lane"].items():
                        output.update([lane.id for lane in values])
                    return output
            else:
                for _, values in self.available_lanes[next_edge.id]["available_lane"].items():
                    output.update([lane.id for lane in values])
                return output
            
            
        raise ValueError("cannot find next available lane")
    
    def get_available_next_lane(self, roadgraph: RoadGraph, lane_id: str) -> AbstractLane:
        """获取当前和当前lane连接的下一个available lane

        Args:
            roadgraph (RoadGraph): 路网信息
            lane_id (str): current lane id

        Returns:
            AbstractLane: 下一个available lane
        """
        lane = roadgraph.get_lane_by_id(lane_id)
        if isinstance(lane, NormalLane):
            # 直接查找相连的lane
            if lane.next_lane:
                next_lane_id = roadgraph.WP2Lane[lane.next_lane]
                if next_lane_id in self.next_available_lanes:
                    return roadgraph.get_lane_by_id(next_lane_id)
            # 查找相连的junction lane
            else:
                next_junction_list = roadgraph.get_Normal2Junction(lane.id)
                for next_lane_i in next_junction_list:
                    if next_lane_i in self.next_available_lanes:
                        return roadgraph.get_lane_by_id(next_lane_i)

        # 如果是junction lane，则直接查找和他相连的lane
        elif isinstance(lane, JunctionLane):
            next_lane_id = roadgraph.WP2Lane[lane.next_lane]
            if next_lane_id in self.next_available_lanes:
                return roadgraph.get_lane_by_id(next_lane_id)
        return None
    
    def calcaulate_cur_edge_remain_length(self, roadgraph: RoadGraph) -> float:
        """计算当前edge的剩余长度，包含junction lane的长度

        Args:
            roadgraph (RoadGraph): 路网信息

        Returns:
            float: edge的长度
        """
        cur_lane = roadgraph.get_lane_by_id(self.lane_id)
        if isinstance(cur_lane, NormalLane):
            cur_section = cur_lane.affiliated_section
            cur_edge = cur_section.affliated_edge
            length = cur_lane.length - self.state.s
            cur_index = cur_edge.section_list.index(cur_section.id)
            # section的index是从 [最远的一个section，它的index为0] 开始的
            for index in range(0, cur_index):
                section_index = cur_edge.section_list[index]
                section = roadgraph.Sections[section_index]
                lane_length=0
                for lane_id in section.lanes.values():
                    lane = roadgraph.NormalLane_Dict[lane_id]
                    lane_length += lane.length
                length += lane_length/section.lane_num
            return length
        return 0
        
    
    def get_state_in_lane(self, lane) -> State:
        course_spline = lane.course_spline

        rs = course_spline.find_nearest_rs(self.state.x,
                                           self.state.y)

        rx, ry = course_spline.calc_position(rs)
        ryaw = course_spline.calc_yaw(rs)
        rkappa = course_spline.calc_curvature(rs)

        s, s_d, d, d_d = cartesian_to_frenet2D(rs, rx, ry, ryaw, rkappa,
                                               self.state)
        return State(s=s, s_d=s_d, d=d, d_d=d_d,
                     x=self.state.x,
                     y=self.state.y,
                     yaw=self.state.yaw,
                     vel=self.state.vel,
                     acc=self.state.acc)
        

    def arrive_destination(self):
        current_LC = carla.Location(x=self.state.x, y=self.state.y, z=self.state.yaw)
        destination_lc = carla.Location(x=self.end_waypoint.transform.location.x, y=self.end_waypoint.transform.location.y, z=self.end_waypoint.transform.location.z)
        return destination_lc.distance(current_LC) < 5.0