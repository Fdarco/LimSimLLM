"""
This file defines the network structure of the map, including the edge, section, and lane.
"""

import carla
from typing import Tuple
from dataclasses import dataclass, field
from abc import ABC
from typing import Dict, Union, Set, List
from utils.cubic_spline import Spline2D
from functools import cached_property

RESOLUTION = 0.5

@dataclass
class Edge:
    """the map is composed of edges and junction_lanes

    properties:
        - last_segment: the last waypoints in the edge, to find the next waypoint in the intersection
        - next_edge_connect: the set of junction_lane that can connect to the next edge
        - section_list: the list of section id in the edge ([id: edge_id-index])
    """
    id: str = 0
    section_num: int = 0
    next_edge_connect: Dict[str, List[str]] = field(default_factory=dict) # to_edge_id: [junction_lane_id, ...]
    section_list: List[str] = field(default_factory=list) # 从终点开始的section_id
    last_segment: List[carla.Waypoint] = field(default_factory=list)

    def __hash__(self): # for comparison
        return hash(self.id)

    def __repr__(self) -> str:
        return f"Edge(id={self.id})"
    
@dataclass
class Section:
    """section is a part of edge

    properties:
        - affliated_edge: the edge that the section belongs to
        - lanes: the dict of lane_id in the section ([lane_id: road_id-section_id-index])
    """
    id: str = 0
    lane_num: int = 0
    affliated_edge: Edge = None
    lanes: Dict[int, str] = field(default_factory=dict)

    @cached_property
    def previous_section_id(self) -> str:
        section_index = int(self.id.split('-')[-1])
        # 因为section_index是从道路终点为0开始的，所以上一个section的index是section_index+1
        if section_index < self.affliated_edge.section_num - 1:
            return self.affliated_edge.section_list[section_index + 1]
        return None
    
    @cached_property
    def next_section_id(self) -> str:
        section_index = int(self.id.split('-')[-1])
        if section_index > 0:
            return self.affliated_edge.section_list[section_index - 1]
        return None
    
    def __hash__(self):
        return hash(self.id)
    
    def __repr__(self) -> str:
        return f"Section(id={self.id})"
    
@dataclass
class AbstractLane(ABC):
    """
    Abstract lane class.
    """
    width: float = 0
    course_spline: Spline2D = None
    id: str = None
    wp_list = []

    @cached_property
    def length(self):
        if self.course_spline:
            return self.course_spline.s[-1]
        else:
            return 0

    @cached_property
    def start_wp(self):
        return self.wp_list[0]
    
    @cached_property
    def end_wp(self):
        return self.wp_list[-1]
    
    def get_spline2D(self):
        """根据wp_list构建spline2D
        """
        xy_list = list(map(lambda wp: [wp.transform.location.x, wp.transform.location.y], self.wp_list))

        #To avoid waypoint overlap,remove same last two wp
        while len(xy_list) >= 2 and xy_list[-1]==xy_list[-2]:
            xy_list.pop(-2)
            self.wp_list.pop(-2)
            
        xy_unzip = list(zip(*xy_list))
        self.course_spline = Spline2D(xy_unzip[0], xy_unzip[1])
    
@dataclass
class NormalLane(AbstractLane):
    """
    Normal lane from edge, just for high-level planning 
    """
    affiliated_section: Section = None
    lane_id: int = 0
    lane_change: carla.LaneChange = carla.LaneChange.NONE

    @cached_property
    def left_lane(self) -> str:
        left_lane_point = self.start_wp.get_left_lane()
        if left_lane_point and left_lane_point.lane_id in self.affiliated_section.lanes.keys():
            return self.affiliated_section.lanes[left_lane_point.lane_id]
        return None

    @cached_property
    def right_lane(self) -> str:
        right_lane_point = self.start_wp.get_right_lane()
        if right_lane_point and right_lane_point.lane_id in self.affiliated_section.lanes.keys():
            return self.affiliated_section.lanes[right_lane_point.lane_id]
        return None

    @cached_property
    def left_access_lane(self) -> str:
        if self.lane_change & carla.LaneChange.Left:
            return self.left_lane
        return None

    @cached_property
    def right_access_lane(self) -> str:
        if self.lane_change & carla.LaneChange.Right:
            return self.right_lane
        return None
    
    @cached_property
    def previous_lane(self) -> Tuple[int, int, int]:
        if self.affiliated_section.previous_section_id:
            for wp in self.start_wp.previous(RESOLUTION):
                if not wp.is_intersection:
                    return (wp.road_id, wp.section_id, wp.lane_id)
            for wp in self.start_wp.previous(5*RESOLUTION):
                if not wp.is_intersection:
                    return (wp.road_id, wp.section_id, wp.lane_id)
        return None

    @cached_property
    def next_lane(self) -> Tuple[int, int, int]:
        """the next normal lane, not include junction lane

        Returns:
            Tuple[int, int, int]: road_id, section_id, lane_id
        """
        if self.affiliated_section.next_section_id:
            for wp in self.end_wp.next(2*RESOLUTION):
                if not (wp.is_intersection or self.wp_equal(wp)):
                    return (wp.road_id, wp.section_id, wp.lane_id)
            for wp in self.end_wp.next(5*RESOLUTION):
                if not (wp.is_intersection or self.wp_equal(wp)):
                    return (wp.road_id, wp.section_id, wp.lane_id)
        return None
    
    def wp_equal(self, wp: carla.Waypoint) -> bool:
        return self.end_wp.road_id == wp.road_id and self.end_wp.section_id == wp.section_id and self.end_wp.lane_id == wp.lane_id
    
    def __repr__(self) -> str:
        # return f"NormalLane(id={self.id}, width = {self.width})"
        return f"NormalLane(id={self.id})"


@dataclass
class JunctionLane(AbstractLane):
    """
    Junction lane in intersection 
    """
    incoming_edge_id: str = 0
    outgoing_edge_id: str = 0

    @property
    def previous_lane(self) -> Tuple[int, int, int]:
        """the junction lane's previous lane is normal lane

        Returns:
            Tuple[int, int, int]: road_id, section_id, lane_id
        """
        for wp in self.start_wp.previous(RESOLUTION):
            if not wp.is_intersection and (wp.road_id, wp.section_id, wp.lane_id)!=(self.start_wp.road_id, self.start_wp.section_id,self.start_wp.lane_id):
                return (wp.road_id, wp.section_id, wp.lane_id)
        for wp in self.start_wp.previous(RESOLUTION*5):
            if not wp.is_intersection and (wp.road_id, wp.section_id, wp.lane_id)!=(self.start_wp.road_id, self.start_wp.section_id,self.start_wp.lane_id):
                return (wp.road_id, wp.section_id, wp.lane_id)
        return None

    @property
    def next_lane(self) -> Tuple[int, int, int]:
        """the junction lane's next lane is normal lane

        Returns:
            Tuple[int, int, int]: road_id, section_id, lane_id
        """
        for wp in self.end_wp.next(RESOLUTION):
            if not wp.is_intersection and (wp.road_id, wp.section_id, wp.lane_id)!=(self.end_wp.road_id, self.end_wp.section_id, self.end_wp.lane_id):
                return (wp.road_id, wp.section_id, wp.lane_id)
        for wp in self.end_wp.next(RESOLUTION*5):
            if not wp.is_intersection and (wp.road_id, wp.section_id, wp.lane_id)!=(self.end_wp.road_id, self.end_wp.section_id, self.end_wp.lane_id):
                return (wp.road_id, wp.section_id, wp.lane_id)
        return None