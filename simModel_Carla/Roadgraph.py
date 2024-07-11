"""
Roadgraph is a data structure that represents the road network of the map. It is composed of edges, sections, lanes and junction_lanes.
"""

import carla
from typing import Tuple
from dataclasses import dataclass, field
from abc import ABC
from typing import Dict, Union, Set, List
import logging
from Network_Structure import Edge, Section, NormalLane, JunctionLane
from agents.navigation.local_planner import RoadOption

RESOLUTION = 0.5

@dataclass
class RoadGraph:
    """
    Road graph of the map

    properties:
        - WP2Section: 根据waypoint找到对应的section_id
        - WP2Lane: 根据waypoint找到对应的lane_id
        - Junction_Dict: 根据junction_lane_id找到对应的 JunctionLane
        - NormalLane_Dict: 根据normal_lane_id找到对应的 NormalLane
        - Edges: 根据edge_id找到对应的 Edge
        - Sections: 根据section_id找到对应的 Section
        - Normal2Junction: 根据normal_lane_id找到它连接(下一个)的 junction_lane_id
    """
    WP2Section: Dict[Tuple[int, int, int], str] = field(default_factory=dict)
    WP2Lane: Dict[Tuple[int, int, int], str] = field(default_factory=dict)
    Junction_Dict: Dict[str, JunctionLane] = field(default_factory=dict)
    NormalLane_Dict: Dict[str, NormalLane] = field(default_factory=dict)
    Edges: Dict[str, Edge] = field(default_factory=dict)
    Sections: Dict[str, Section] = field(default_factory=dict)
    Normal2Junction: Dict[str, List[str]] = field(default_factory=dict)


    def get_lane_by_id(self, lane_id: str) -> Union[NormalLane, JunctionLane]:
        if lane_id in self.NormalLane_Dict:
            return self.NormalLane_Dict[lane_id]
        elif lane_id in self.Junction_Dict:
            return self.Junction_Dict[lane_id]
        else:
            logging.debug(f"cannot find lane {lane_id}")
            return None
        
    def get_Normal2Junction(self, lane_id: str) -> List[str]:
        """get the junction lane that the normal lane can connect to

        Args:
            lane_id (str): normal lane id

        Returns:
            List[str]: junction lane id
        """
        if lane_id in self.Normal2Junction:
            return self.Normal2Junction[lane_id]
        else:
            logging.debug(f"cannot find junction lane {lane_id}")
            return None


    def get_laneID_by_s(self, road_id: str, lane_id: str, s: float, carla_map:carla.Map) -> Tuple[str, carla.Waypoint]:
        """对于一个位置可能对应多个waypoint的情况，需要根据s值找到对应的lane_id

        Args:
            road_id (str): waypoint所在的road_id
            lane_id (str): waypoint所在的lane_id
            s (float): waypoint所在的s值
            carla_map (carla.Map): Map

        Returns:
            str: lane_id
        """
        waypoint = carla_map.get_waypoint_xodr(road_id, lane_id, s)
        if waypoint == None:
            return None, None
        return self.WP2Lane[(waypoint.road_id, waypoint.section_id, waypoint.lane_id)], waypoint
    
    def get_laneID_by_xy(self, x: float, y: float, carla_map:carla.Map) -> Tuple[str, carla.Waypoint]:
        """根据x, y找到对应的lane_id

        Args:
            x (float): waypoint所在的x
            y (float): waypoint所在的y
            carla_map (carla.Map): Map

        Returns:
            str: _description_
        """
        lc = carla.Location(x=x, y=y)
        waypoint = carla_map.get_waypoint(lc)

        try:
            lane_id=self.WP2Lane[(waypoint.road_id, waypoint.section_id, waypoint.lane_id)]
        except KeyError:
            lane_id=None
        return lane_id, waypoint
    
    def get_all_available_lanes(self, route: List[str], destination_wp: carla.Waypoint) -> Dict[str, Dict[str, List[NormalLane]]]:
        """从道路连接处的junction lane倒推，找到edge中每段section的lane

        Args:
            route (List[str]): the vehicle's drive path, list of edge_id
            destination_wp (carla.Waypoint): the destination waypoint

        Returns:
            Dict[str, Dict[str, List[NormalLane]]]: avaliable_lanes
            eg: avaliable_lanes: {
                    edge_id: {
                        available_lane:{section_id: [lane_id, ...]}, # 本edge上所有的lane
                        junction_lane: [junction_lane_id, ...],
                        change_lane: {section_id: [lane_id, ...]}, # 连通下一个edge的lane
                    }
        """
        
        available_lane_dict = dict()
        
        ## 遍历每个edge，找到每个edge上的可行驶lane
        for i in range(len(route)-1):
            cur_edge = self.Edges[route[i]]
            junction_lanes = [self.Junction_Dict[id] for id in cur_edge.next_edge_connect[route[i+1]]]
            # 取出最后一个section,may be section is builded from edge end to beginning
            last_section = self.Sections[cur_edge.section_list[0]]
            
            ## 找到与next_edge之间的junction相连的lane作为change lane
            change_lanes = []
            last_lane_id_set = set()
            for junction_lane in junction_lanes:
                last_lane_id = junction_lane.previous_lane[2]
                if last_lane_id not in last_lane_id_set:
                    last_lane_id_set.add(last_lane_id)
                    change_lanes.append(self.NormalLane_Dict[last_section.lanes[last_lane_id]])

            change_dict = self.get_all_change_lane(change_lanes, last_section)

            available_lane_dict[route[i]] = {
                'available_lane': self.get_all_lanes_in_edge(route[i]),
                'junction_lane': junction_lanes,
                'change_lane': change_dict
            }
        
        ## 对于最后一个edge的可行驶lane，从目标点位置开始往后找
        last_section = self.Sections[self.WP2Section[(destination_wp.road_id, destination_wp.section_id, destination_wp.lane_id)]]
        change_lane = [self.NormalLane_Dict[self.WP2Lane[(destination_wp.road_id, destination_wp.section_id, destination_wp.lane_id)]]] # 当前车道

        available_lane_dict[route[-1]] = {
            'available_lane': self.get_all_lanes_in_edge(route[-1]),
            'junction_lane': [],
            'change_lane': self.get_all_change_lane(change_lane, last_section)
        }
        
        return available_lane_dict

    # 根据change lane进行纵向扩展，如果遇到没有路的情况就找他左右两边车道的previous lane
    def get_all_change_lane(self, last_change_lanes: List[NormalLane], last_section: Section) -> Dict[str, List[NormalLane]]:
        """根据最后一个section的change lane找到这个edge上沿着change_lane搜索到的所有lanes

        Args:
            last_change_lanes (List[NormalLane]): edge上的最后一个section的change lane

        Returns:
            Dict[str: List[NormalLane]]: 在这个edge上沿着change_lane搜索到的lane
        """

        lattitude_lanes = {
            f"{last_section.id}": last_change_lanes[::]
        }
        while last_section.previous_section_id:
            cur_lanes = last_change_lanes[::]
            last_change_lanes.clear()
            last_section = self.Sections[last_section.previous_section_id]
            for lane in cur_lanes:
                if lane.previous_lane:
                    previous_lane = self.WP2Lane[lane.previous_lane]
                    last_change_lanes.append(self.NormalLane_Dict[previous_lane])
                else:
                    if lane.left_lane:
                        left_lane = self.NormalLane_Dict[lane.left_lane]
                        if left_lane.previous_lane:
                            last_change_lanes.append(self.NormalLane_Dict[self.WP2Lane[left_lane.previous_lane]])

                    elif lane.right_lane:
                        right_lane = self.NormalLane_Dict[lane.right_lane]
                        if right_lane.previous_lane:
                            last_change_lanes.append(self.NormalLane_Dict[self.WP2Lane[right_lane.previous_lane]])
                    else:
                        raise("No previous lane")
            lattitude_lanes[last_section.id] = last_change_lanes[::]
        return lattitude_lanes
            
    def get_all_lanes_in_edge(self, edge_id: str) -> Dict[str, List[NormalLane]]:
        """根据edge_id找到这个edge上的所有lane

        Args:
            edge_id (str): edge id

        Returns:
            Dict[str: List[NormalLane]]: section_id: [lane_id, ...]
        """
        lanes_in_edge = dict()
        for section_id in self.Edges[edge_id].section_list:
            section = self.Sections[section_id]
            lanes_in_edge[section_id] = []
            for lane_id in section.lanes.values():
                lanes_in_edge[section_id].append(self.NormalLane_Dict[lane_id])
        return lanes_in_edge

    def __str__(self):
        return 'edges: {}, \nlanes: {}, \njunctions lanes: {}'.format(
            self.Edges.keys(), self.NormalLane_Dict.keys(),
            self.Junction_Dict.keys()
        )
    
    def get_route_edge(self, route: List[Tuple[carla.Waypoint, RoadOption]]) -> List[str]:
        """获取route上的所有edge_id

        Args:
            route (List[Tuple[carla.Waypoint, carla.RoadOption]]): the return value of GlobalRoutePlanner.trace_route()

        Returns:
            List[str]: list of edge_id
        """
        route_edge_list = []
        for wp, _ in route:
            if (wp.road_id, wp.section_id, wp.lane_id) in self.WP2Section.keys() and self.Sections[self.WP2Section[(wp.road_id, wp.section_id, wp.lane_id)]].affliated_edge.id not in route_edge_list:
                route_edge_list.append(self.Sections[self.WP2Section[(wp.road_id, wp.section_id, wp.lane_id)]].affliated_edge.id)
        
        return route_edge_list
    
    def wp_transform(self,carla_map):
        """
        To cache or load RoadGraph, need to mutual conversion between waypoints and dict
        """
        def wp_to_dict(wp:carla.libcarla.Waypoint):
            if type(wp)==dict:
                return wp
            wp_d={}
            wp_d['road_id']=wp.road_id
            wp_d['section_id']=wp.section_id
            wp_d['lane_id']=wp.lane_id
            wp_d['s']=wp.s
            return wp_d
        def dict_to_wp(wp_d:dict):
            if type(wp_d)!=dict:
                return wp_d
            return carla_map.get_waypoint_xodr(wp_d['road_id'],wp_d['lane_id'],wp_d['s'])
            
        for edge in list(self.Edges.values()):
            if edge.last_segment:
                if type(edge.last_segment[0])!=dict:
                    edge.last_segment=[wp_to_dict(wp) for wp in edge.last_segment]
                else:
                    edge.last_segment=[dict_to_wp(wp) for wp in edge.last_segment]
                    wp_list=[]
                    for wp in edge.last_segment:
                        if wp:
                            wp_list.append(wp)
                    edge.last_segment=wp_list
        
        for lane in list(self.NormalLane_Dict.values()):
            if lane.wp_list:
                if type(lane.wp_list[0])!=dict:
                    lane.wp_list=[wp_to_dict(wp) for wp in lane.wp_list]
                    lane.start_wp=wp_to_dict(lane.start_wp)
                    lane.end_wp=wp_to_dict(lane.end_wp)
                else:
                    lane.wp_list=[dict_to_wp(wp) for wp in lane.wp_list]
                    wp_list=[]
                    for wp in lane.wp_list:
                        if wp:
                            wp_list.append(wp)
                    lane.wp_list=wp_list
                    lane.start_wp=dict_to_wp(lane.start_wp) if dict_to_wp(lane.start_wp)!=None else lane.wp_list[0]
                    lane.end_wp=dict_to_wp(lane.end_wp) if dict_to_wp(lane.end_wp)!=None else lane.wp_list[-1]

        for lane in list(self.Junction_Dict.values()):
            if lane.wp_list:
                if type(lane.wp_list[0])!=dict:
                    lane.wp_list=[wp_to_dict(wp) for wp in lane.wp_list]
                    lane.start_wp=wp_to_dict(lane.start_wp)
                    lane.end_wp=wp_to_dict(lane.end_wp)
                else:
                    lane.wp_list=[dict_to_wp(wp) for wp in lane.wp_list]
                    wp_list=[]
                    for wp in lane.wp_list:
                        if wp:
                            wp_list.append(wp)
                    lane.wp_list=wp_list
                    lane.start_wp=dict_to_wp(lane.start_wp) if dict_to_wp(lane.start_wp)!=None else lane.wp_list[0]
                    lane.end_wp=dict_to_wp(lane.end_wp) if dict_to_wp(lane.end_wp)!=None else lane.wp_list[-1]



