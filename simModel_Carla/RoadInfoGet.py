"""
    WP2Section: dict, {(road_id, section_id, lane_id): (edge_id, section_id) ...}
    WP2Junction: dict, {(road_id, section_id, lane_id): junction_id ...}
    Junction_Dict: dict, {
        'junction_id': Junction_Lane
        ...
    }
    Edge_Dict: dict, {
        'edge_id': {
            section_id: Section
            ...
        }
    }
    Edge_Connect: dict, {
        'from_edge_id': {
            to_edge_id: [junction_id, ...]
        }...
    }

    traffic_light: 该信息存储在vehicle中
    当前版本的bug: 对于只出现一次的waypoint可能会导致其他waypoint之前存储的edge_id被覆盖，从而出现闲置的edge_id

"""

# 1. 重构一版代码推送到github
# 2. 增加get_current_available_lane函数，根据当前车辆的位置，获取当前车辆可用的lane
# 3. 测试一版，跟车移动，看avaiable更新是否正确

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from trafficManager.common.vehicle import Behaviour
from typing import Tuple
from dataclasses import dataclass, field
from abc import ABC
from typing import Dict, Union, Set, List
from utils.cubic_spline import Spline2D
import logging
from functools import cached_property

RESOLUTION = 0.5

@dataclass
class Edge:
    """the map is composed of edges and junction_lanes

    properties:
        - last_segment: to find the next waypoint in the intersection
        - next_edge_connect: the set of junction_lane that can connect to the next edge
        - section_list: the list of section id in the edge ([id: edge_id-index])
    """
    id: str = 0
    section_num: int = 0
    next_edge_connect: Dict[int, List[str]] = field(default_factory=dict) # to_edge_id: [junction_lane_id, ...]
    section_list: List[str] = field(default_factory=list)
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
        return self.course_spline.s[-1]

    @cached_property
    def start_wp(self):
        return self.wp_list[0]
    
    @cached_property
    def end_wp(self):
        return self.wp_list[-1]
    
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
        return None

    @cached_property
    def next_lane(self) -> Tuple[int, int, int]:
        if self.affiliated_section.next_section_id:
            for wp in self.end_wp.next(RESOLUTION):
                if not wp.is_intersection:
                    return (wp.road_id, wp.section_id, wp.lane_id)
        return None
    
    def __repr__(self) -> str:
        # return f"NormalLane(id={self.id}, width = {self.width})"
        return f"NormalLane(id={self.id})"


@dataclass
class JunctionLane(AbstractLane):
    """
    Junction lane in intersection 
    """
    incoming_edge_id: int = 0
    outgoing_edge_id: int = 0

    @cached_property
    def previous_lane(self) -> Tuple[int, int, int]:
        for wp in self.start_wp.previous(RESOLUTION):
            if not wp.is_intersection:
                return (wp.road_id, wp.section_id, wp.lane_id)
        return None

    @cached_property
    def next_lane(self) -> Tuple[int, int, int]:
        for wp in self.end_wp.next(RESOLUTION):
            if not wp.is_intersection:
                return (wp.road_id, wp.section_id, wp.lane_id)
        return None

@dataclass
class RoadGraph:
    """
    Road graph of the map
    """
    WP2Section: Dict[Tuple[int, int, int], str] = field(default_factory=dict)
    WP2Lane: Dict[Tuple[int, int, int], str] = field(default_factory=dict)
    Junction_Dict: Dict[str, JunctionLane] = field(default_factory=dict)
    NormalLane_Dict: Dict[str, NormalLane] = field(default_factory=dict)
    Edges: Dict[str, Edge] = field(default_factory=dict)
    Sections: Dict[str, Section] = field(default_factory=dict)
    # Intersection: Dict[str, List[str]] = field(default_factory=dict)


    def get_lane_by_id(self, lane_id: str) -> Union[NormalLane, JunctionLane]:
        if lane_id in self.NormalLane_Dict:
            return self.NormalLane_Dict[lane_id]
        elif lane_id in self.Junction_Dict:
            return self.Junction_Dict[lane_id]
        else:
            logging.debug(f"cannot find lane {lane_id}")
            return None
        
    def get_all_available_lanes(self, edge_id: List[str], destination_wp: carla.Waypoint) -> Dict[str, Dict[str, List[NormalLane]]]:
        """从道路连接处的junction lane倒推，找到edge中每段section的lane
        avaliable_lanes: {
            edge_id: {
                available_lane:{section_id: [lane_id, ...]},
                junction_lane: [junction_lane_id, ...],
                change_lane: [lane_id, ...]
            }"""
        available_lane_dict = dict()
        
        for i in range(len(edge_id)-1):
            cur_edge = self.Edges[edge_id[i]]
            junction_lanes = [self.Junction_Dict[id] for id in cur_edge.next_edge_connect[edge_id[i+1]]]
            last_section = self.Sections[cur_edge.section_list[0]]
            
            ## 找到与junction相连的lane作为change lane
            change_lanes = []
            last_lane_id_set = set()
            for junction_lane in junction_lanes:
                last_lane_id = junction_lane.previous_lane[2]
                if last_lane_id not in last_lane_id_set:
                    last_lane_id_set.add(last_lane_id)
                    change_lanes.append(self.NormalLane_Dict[last_section.lanes[last_lane_id]])

            ## 收集这个edge上所有section的可行驶 lane，先扩展change lane
            available_section_lanes = dict()
            available_section_lanes[last_section.id] = self.get_section_available_lanes(last_section, [], change_lanes)
            
            ## 接着向前找到所有section的lane
            while last_section.previous_section_id != None:
                section = self.Sections[last_section.previous_section_id]
                available_section_lanes[section.id] = self.get_section_available_lanes(section, available_section_lanes[last_section.id])
                last_section = section

            available_lane_dict[edge_id[i]] = {
                'available_lane': available_section_lanes,
                'junction_lane': junction_lanes,
                'change_lane': change_lanes
            }
        
        ## 最后一个edge的可行驶lane，从目标点位置开始往后找
        available_section_lanes = dict()
        last_section = self.Sections[self.WP2Section[(destination_wp.road_id, destination_wp.section_id, destination_wp.lane_id)]]
        change_lane = [self.NormalLane_Dict[self.WP2Lane[(destination_wp.road_id, destination_wp.section_id, destination_wp.lane_id)]]] # 当前车道
        available_section_lanes[last_section.id] = change_lane[::]
        ## 接着向前找到所有section的lane
        while last_section.previous_section_id:
            section = self.Sections[last_section.previous_section_id]
            available_section_lanes[section.id] = self.get_section_available_lanes(section, available_section_lanes[last_section.id])
            last_section = section

        available_lane_dict[edge_id[-1]] = {
            'available_lane': available_section_lanes,
            'junction_lane': [],
            'change_lane': change_lane
        }
        
        # print(available_lane_dict)
        return available_lane_dict

    def get_section_available_lanes(self, cur_section: Section, previous_lanes: List[NormalLane], available_lanes: List[NormalLane]=None) -> List[NormalLane]:
        """根据可行驶lane进行横向拓展
        1. 如果previous_lanes为空，说明是和下一个edge连接的第一个section，需要从avaiable_lanes中进行拓展
        2. 如果previous_lanes不为空，说明是后续的section，需要从previous_lanes中找到这个section上的lane，然后进行拓展

        Args:
            cur_section (Section): 当前的section
            previous_lanes (List[NormalLane]): 前一个section上可行驶的lane
            available_lanes (List[NormalLane], optional): 当前这个section上的change lane. Defaults to None.

        Returns:
            List[NormalLane]: 当前这个section上的所有可行驶的lane
        """
        lane_id_set = set()
        if available_lanes is None:
            available_lanes = []
        for lane in previous_lanes:
            if lane.previous_lane != None:
                previous_lane = self.NormalLane_Dict[self.WP2Lane[lane.previous_lane]]
                available_lanes.append(previous_lane)
                lane_id_set.add(previous_lane.id)
        temp_lanes = available_lanes[::]
        for lane in temp_lanes:
            if len(lane_id_set) == cur_section.lane_num:
                break
            while lane.left_access_lane:
                if lane.left_access_lane not in lane_id_set:
                    lane_id_set.add(lane.left_access_lane)
                    available_lanes.append(self.NormalLane_Dict[lane.left_access_lane])
                lane = self.NormalLane_Dict[lane.left_access_lane]
            while lane.right_access_lane:
                if lane.right_access_lane not in lane_id_set:
                    lane_id_set.add(lane.right_access_lane)
                    available_lanes.append(self.NormalLane_Dict[lane.right_access_lane])
                lane = self.NormalLane_Dict[lane.right_access_lane]
        return available_lanes

    def __str__(self):
        return 'edges: {}, \nlanes: {}, \njunctions lanes: {}'.format(
            self.Edges.keys(), self.NormalLane_Dict.keys(),
            self.Junction_Dict.keys()
        )
    

class RoadInfoGet:
    def __init__(self, roadgraph: RoadGraph, topology: List[Tuple[carla.Waypoint, carla.Waypoint]] = None):
        self.topology = topology
        self.road_id_to_edge = {}
        self.road_id_to_junction = {}

        self.roadgraph = roadgraph  # 用于更新路网信息

        self.build_edge()

    def build_edge(self):
        """根据topology建立edge和section
        """
        segment_start_wp = [segment[0] for segment in self.topology]
        for wp in segment_start_wp:
            if wp.is_intersection or wp.lane_type != carla.LaneType.Driving:
                continue

            edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)

            if edge_search_key in self.road_id_to_edge.keys():
                continue

            cur_edge_id = str(len(self.roadgraph.Edges))
            
            last_segment = self.create_edge(cur_edge_id, wp) # 返回edge最后的一排waypoint
            self.create_section(cur_edge_id, last_segment) # 根据edge的最后一排waypoint找到这个edge所有的waypoint，构建section
        
        # 获得所有的junction_lane
        self.get_connect()
                

    def create_edge(self, edge_id: str, wp: carla.Waypoint)->List[carla.Waypoint]:
        """创建Edge，返回最后一排waypoint

        Args:
            edge_id (str): edge id
            wp (carla.Waypoint): 采样的waypoint

        Returns:
            List[carla.Waypoint]: 最后一排waypoint
        """

        edge = Edge(id=edge_id)
        self.roadgraph.Edges[edge_id] = edge

        # 找到这个wp所在的edge的最后一个waypoint
        while wp.next(RESOLUTION) and not wp.next(RESOLUTION)[0].is_intersection:
            wp = wp.next(RESOLUTION)[0]
            # 处理next返回为none的case，遍历左右lane找到next不为none的waypoint
            if wp.next(RESOLUTION) == None:
                while wp.next(RESOLUTION) == None:
                    lwp = wp.get_left_lane()
                    if lwp and lwp.lane_type == carla.LaneType.Driving and lwp.lane_id * wp.lane_id > 0:
                        wp = lwp
                    else:
                        break
                else:
                    continue

                while wp.next(RESOLUTION) == None:
                    rwp = wp.get_right_lane()
                    if rwp and rwp.lane_type == carla.LaneType.Driving and rwp.lane_id * wp.lane_id > 0:
                        wp = rwp
                    else:
                        break
                else:
                    continue

                break # 如果左右都没有找到，直接跳出循环
        
        last_segment = []

        # -- 根据最后一个wp找到exit该edge的所有lane的waypoint -- #
        # 根据lane_id 进行划分，引用xodr的解释：需要使用中心车道对OpenDRIVE中的车道进行定义和描述。中心车道没有宽度，并被用作车道编号的参考，自身的车道编号为0。对其他车道的编号以中心车道为出发点：车道编号向右呈降序，也就是朝负t方向；向左呈升序，也就是朝正t方向。
        lane_id = wp.lane_id
        lwp = wp.get_left_lane()
        while lwp and lwp.lane_id * lane_id > 0 and lwp.lane_type == carla.LaneType.Driving:
            # self.edge_id[edge_id]['last_segment'].append(lwp)
            last_segment.append(lwp)
            lwp = lwp.get_left_lane()

        last_segment.reverse()
        last_segment.append(wp)

        rwp = wp.get_right_lane()
        while rwp and rwp.lane_id * lane_id > 0 and rwp.lane_type == carla.LaneType.Driving:
            last_segment.append(rwp)
            rwp = rwp.get_right_lane()
        self.roadgraph.Edges[edge_id].last_segment = last_segment[::]
        return last_segment # 从左到右的顺序
            
    def create_section(self, edge_id: str, wp_list: List[carla.Waypoint]):
        """根据edge_id中存储的last_segment中的waypoint向previous方向遍历，找到该edge上的所有的section

        Args:
            edge_id (str): section所属的edge
            wp_list (List[carla.Waypoint]): edge的最后一排waypoint
        """
        while wp_list:
            # 根据wp_list创建一个section
            section = Section()
            section.id = edge_id + '-' + str(self.roadgraph.Edges[edge_id].section_num)
            section.affliated_edge = self.roadgraph.Edges[edge_id]
            self.roadgraph.Edges[edge_id].section_num += 1
            self.roadgraph.Edges[edge_id].section_list.append(section.id)

            new_wp = None
            new_wp_list = []
            # 根据wp_list中的每个waypoint创建一个section上的lane
            for wp in wp_list:
                lane, new_wp = self.create_normal_lane(section, edge_id, wp)
                section.lane_num+=1
                section.lanes[lane.lane_id] = lane.id
                self.roadgraph.NormalLane_Dict[lane.id] = lane
            self.roadgraph.Sections[section.id] = section

            # 查看新的section上左右是否有新增的lane
            if new_wp:
                lane_id = new_wp.lane_id
                lwp = new_wp.get_left_lane()
                while lwp and lwp.lane_id * lane_id > 0 and lwp.lane_type == carla.LaneType.Driving:
                    new_wp_list.append(lwp)
                    lwp = lwp.get_left_lane()
                new_wp_list.reverse()
                new_wp_list.append(new_wp)

                rwp = new_wp.get_right_lane()
                while rwp and rwp.lane_id * lane_id > 0 and rwp.lane_type == carla.LaneType.Driving:
                    new_wp_list.append(rwp)
                    rwp = rwp.get_right_lane()
            
            wp_list = new_wp_list[::]


    def create_normal_lane(self, section: Section, edge_id: str, wp: carla.Waypoint) -> Tuple[NormalLane, carla.Waypoint]:
        """根据waypoint创建一个section上的一条lane

        Args:
            section (Section): lane所属的section
            edge_id (str): section所属的edge
            wp (carla.Waypoint): lane的终点waypoint

        Returns:
            NormalLane: 创建的lane
            carla.Waypoint: 下一个section的终点waypoint
        """
        lane = NormalLane()
        lane.id = section.id + '-' + str(section.lane_num)
        lane.lane_id = wp.lane_id
        lane.affiliated_section = section
        lane.width = wp.lane_width
        lane.lane_change = wp.lane_change

        cur_wp = wp
        last_wp = wp
        edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
        self.road_id_to_edge[edge_search_key] = edge_id
        self.roadgraph.WP2Lane[(wp.road_id, wp.section_id, wp.lane_id)] = lane.id
        self.roadgraph.WP2Section[(wp.road_id, wp.section_id, wp.lane_id)] = section.id

        wp_in_line_list = [cur_wp]
        new_wp = None
        while True:
            # 当previous为空的时候，退出
            if len(cur_wp.previous(RESOLUTION)) == 0:
                break

            for wp_pre in cur_wp.previous(RESOLUTION):
                if not wp_pre.is_intersection:
                    cur_wp = wp_pre
                    break
            if(cur_wp == last_wp): # 防止死循环
                break
            
            # 当遇到新的section的时候，退出
            if cur_wp.road_id != last_wp.road_id or cur_wp.section_id != last_wp.section_id:
                new_wp = cur_wp
                break
            wp_in_line_list.append(cur_wp)
            last_wp = cur_wp

        lane.wp_list = wp_in_line_list[::-1]

        return lane, new_wp

    def get_connect(self):
        """根据edge的last_segment找到junction_lane
        """
        junction_points = []
        # -- 采样所有的instersection的waypoint -- #
        for edge in self.roadgraph.Edges.values():
            last_wp = edge.last_segment[-1]
            next_wp = last_wp.next(RESOLUTION)
            if next_wp:
                for wp in next_wp:
                    if wp.is_intersection and wp.lane_type == carla.LaneType.Driving:
                        junction_points.append(wp)

        # -- 遍历所有的junction_waypoint，构建junction_lane -- #
        for wp in junction_points:
            edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
            if edge_search_key in self.road_id_to_junction.keys():
                continue
            
            self.create_junction_lane(wp)

    def create_junction_lane(self, wp):
        """根据junction waypoint创建该wp所在路口的所有junction_lane

        Args:
            wp (carla.Waypoint): junction waypoint
        """
        junction = wp.get_junction()
        wp_tuple = junction.get_waypoints(carla.LaneType.Driving)
        for i in range(len(wp_tuple)):
            (start_wp, _) = wp_tuple[i]
            last_wp = start_wp.previous(RESOLUTION)
            filter_wp = list(filter(lambda x: not x.is_intersection, last_wp))
            if len(filter_wp) == 0:
                continue
            junction_lane = JunctionLane(width=start_wp.lane_width)

            # 获取junction_lane的waypoint
            junction_lane.wp_list = [start_wp] + start_wp.next_until_lane_end(RESOLUTION)
            
            for next_wp in junction_lane.end_wp.next(RESOLUTION):
                if not next_wp.is_intersection:
                    next_edge = self.road_id_to_edge[str(next_wp.road_id) + '_' + str(next_wp.section_id) + '_' + str(next_wp.lane_id)]
                    junction_lane.outgoing_edge_id = next_edge
                    break

            for last_wp in start_wp.previous(RESOLUTION):
                if not last_wp.is_intersection:
                    edge_id = self.road_id_to_edge[str(last_wp.road_id) + '_' + str(last_wp.section_id) + '_' + str(last_wp.lane_id)]
                    junction_lane.incoming_edge_id = edge_id
                    break

            junction_lane.id = junction_lane.incoming_edge_id + '-' + junction_lane.outgoing_edge_id + '-' + str(i)
            if(next_edge not in self.roadgraph.Edges[edge_id].next_edge_connect):
                self.roadgraph.Edges[edge_id].next_edge_connect[next_edge] = []
            self.roadgraph.Edges[edge_id].next_edge_connect[next_edge].append(junction_lane.id)
            
            for wp_i in junction_lane.wp_list:
                edge_search_key = str(wp_i.road_id) + '_' + str(wp_i.section_id) + '_' + str(wp_i.lane_id)
                self.road_id_to_junction[edge_search_key] = junction_lane.id
            self.roadgraph.Junction_Dict[junction_lane.id] = junction_lane

    def get_route_edge(self, route) -> set:
        route_edge_list = []
        for wp in route:
            edge_search_key = str(wp[0].road_id) + '_' + str(wp[0].section_id) + '_' + str(wp[0].lane_id)
            if edge_search_key in self.road_id_to_edge and self.road_id_to_edge[edge_search_key] not in route_edge_list:
                route_edge_list.append(self.road_id_to_edge[edge_search_key])
        
        return route_edge_list
            

    def next_available_action(self, wp) -> Tuple[Behaviour]:

        return Behaviour.LCL
    
# ------------------ draw function ------------------ #
import random

def random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    rgb_code = '#%02x%02x%02x' % (r, g, b)
    # return carla.Color(r=r, g=g, b=b)
    return rgb_code

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def draw_roadgraph(roadgraph: RoadGraph):
    """绘制roadgraph

    Args:
        roadgraph (RoadGraph): 构建的edge section lane junction
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    for edge in roadgraph.Edges.values():
        edge_color = random_color()
        for section_id in edge.section_list:
            section_box = []
            section = roadgraph.Sections[section_id]
            color = random_color()
            for lane_id in section.lanes.values():
                lane = roadgraph.NormalLane_Dict[lane_id]
                ax.plot([wp.transform.location.x for wp in lane.wp_list], [wp.transform.location.y for wp in lane.wp_list], color=color)
            left_lane_id = section.lanes[min(section.lanes.keys())]
            right_lane_id = section.lanes[max(section.lanes.keys())]
            left_lane = roadgraph.NormalLane_Dict[left_lane_id]
            right_lane = roadgraph.NormalLane_Dict[right_lane_id]
            section_box.extend([wp_i.transform.location.x, wp_i.transform.location.y] for wp_i in left_lane.wp_list)
            section_box.reverse()
            section_box.extend([wp_i.transform.location.x, wp_i.transform.location.y] for wp_i in right_lane.wp_list)
            ax.add_patch(Polygon(xy=section_box, fill=True, alpha = 0.4, color='red'))
        for next_edge, junction_list in edge.next_edge_connect.items():
            color = random_color()
            for junction_id in junction_list:
                junction_lane = roadgraph.Junction_Dict[junction_id]
                for wp in junction_lane.start_waypoint.next_until_lane_end(RESOLUTION):
                    ax.plot([wp.transform.location.x], [wp.transform.location.y], color = color, marker='o', markersize=1)

    plt.show()


if __name__ == '__main__':
    # 创建一个新的观察者
    # spectator = world.get_spectator()
    # initial_location = carla.Location(x=2000.0, y=0.0, z=1000.0)  # 例如，x, y, z坐标
    # initial_rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)  # 默认旋转

    # # 设置观察者的位置和方向
    # spectator.set_transform(carla.Transform(initial_location, initial_rotation))

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    # client.load_world('Town06')
    world = client.get_world()
    map = world.get_map()
    topology = map.get_topology()
    

    roadgraph = RoadGraph()
    road_info_get = RoadInfoGet(roadgraph, topology)

    for item in roadgraph.Edges.items():
        world.debug.draw_string(item[1].last_segment[0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100)

        # --------------- test --------------- #
    spawn_points = map.get_spawn_points()
    origin = carla.Location(spawn_points[100].location)
    destination = carla.Location(spawn_points[87].location)  

    grp = GlobalRoutePlanner(map, 0.5)
    route = grp.trace_route(origin, destination)
    route_edge_list = road_info_get.get_route_edge(route)
    print(route_edge_list)

    for wp in route:
        world.debug.draw_point(wp[0].transform.location, color=carla.Color(r=0, g=0, b=255), life_time=50, size=0.1)
    
    available_dict = roadgraph.get_all_available_lanes(route_edge_list, route[-1][0])
    

    ''' 画出available lane'''
    for edge_id, available_lanes in available_dict.items():
        for section_id, lanes in available_lanes['available_lane'].items():
            for lane in lanes:
                if lane in available_lanes['change_lane']:
                    continue
                world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=0, g=255, b=0), thickness=1.0, life_time=100)
        for junction_lane in available_lanes['junction_lane']:
            world.debug.draw_line(junction_lane.start_wp.transform.location, junction_lane.end_wp.transform.location, color=carla.Color(r=0, g=0, b=255), thickness=1.0, life_time=100)
        for lane in available_lanes['change_lane']:
            world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=255, g=0, b=0), thickness=1.0, life_time=100)

