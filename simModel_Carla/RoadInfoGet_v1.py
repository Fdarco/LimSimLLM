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
import uuid, copy
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
        if section_index > 0:
            return self.affliated_edge.section_list[section_index - 1]
        return None
    
    @cached_property
    def next_section_id(self) -> str:
        section_index = int(self.id.split('-')[-1])
        if section_index < len(self.affliated_edge.section_list) - 1:
            return self.affliated_edge.section_list[section_index + 1]
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

    def left_lane(self) -> str:
        if self.lane_change & carla.LaneChange.Left:
            left_lane_point = self.start_wp.get_left_lane()
            if left_lane_point and left_lane_point.lane_id in self.affiliated_section.lanes.keys():
                return self.affiliated_section.lanes[left_lane_point.lane_id]
        return None

    def right_lane(self) -> str:
        if self.lane_change & carla.LaneChange.Right:
            right_lane_point = self.start_wp.get_right_lane()
            if right_lane_point and right_lane_point.lane_id in self.affiliated_section.lanes.keys():
                return self.affiliated_section.lanes[right_lane_point.lane_id]
        return None

    def previous_lane(self) -> Tuple[int, int, int]:
        if self.affiliated_section.previous_section_id():
            for wp in self.start_wp.previous(RESOLUTION):
                if not wp.is_intersection:
                    return (wp.road_id, wp.section_id, wp.lane_id)
        return None

    def next_lane(self) -> Tuple[int, int, int]:
        if self.affiliated_section.next_section_id():
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
    def last_lane_id(self) -> Tuple[int, int, int]:
        for wp in self.start_wp.previous(RESOLUTION):
            if not wp.is_intersection:
                return (wp.road_id, wp.section_id, wp.lane_id)
        return None

    @cached_property
    def next_lane_id(self) -> Tuple[int, int, int]:
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
    Intersection: Dict[str, List[str]] = field(default_factory=dict)


    def get_lane_by_id(self, lane_id: str) -> Union[NormalLane, JunctionLane]:
        if lane_id in self.NormalLane_Dict:
            return self.NormalLane_Dict[lane_id]
        elif lane_id in self.Junction_Dict:
            return self.Junction_Dict[lane_id]
        else:
            logging.debug(f"cannot find lane {lane_id}")
            return None
        
    def get_all_available_lanes(self, edge_id: List[str]) -> Dict[str, Dict[str, List[NormalLane]]]:
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
            last_section = self.Sections[cur_edge.section_list[-1]]
            
            ## 找到与junction相连的lane作为change lane
            change_lanes = []
            last_lane_id_set = set()
            for junction_lane in junction_lanes:
                last_lane_id = junction_lane.last_lane_id()[2]
                if last_lane_id not in last_lane_id_set:
                    last_lane_id_set.add(last_lane_id)
                    change_lanes.append(self.NormalLane_Dict[last_section.lanes[last_lane_id]])

            ## 收集这个edge上所有section的可行驶 lane，先扩展change lane
            available_section_lanes = dict()
            self.get_section_available_lanes(last_section, [], change_lanes)
            lane_id_set = set(lane.id for lane in available_lanes)
            for lane in change_lanes:
                if len(lane_id_set) == last_section.lane_num:
                    break
                while lane.left_lane():
                    if lane.left_lane() not in lane_id_set:
                        lane_id_set.add(lane.left_lane())
                        available_lanes.append(self.NormalLane_Dict[lane.left_lane()])
                    lane = self.NormalLane_Dict[lane.left_lane()]
                while lane.right_lane():
                    if lane.right_lane() not in lane_id_set:
                        lane_id_set.add(lane.right_lane())
                        available_lanes.append(self.NormalLane_Dict[lane.right_lane()])
                    lane = self.NormalLane_Dict[lane.right_lane()]
            
            available_section_lanes[last_section.id] = available_lanes[::]
            
            ## 接着向前找到所有section的lane
            while last_section.previous_section_id():
                section = self.Sections[last_section.previous_section_id()]
                available_section_lanes[section.id] = self.get_section_available_lanes(section, available_section_lanes[last_section.id])
                last_section = section

            available_lane_dict[edge_id[i]] = {
                'available_lane': available_section_lanes,
                'junction_lane': junction_lanes,
                'change_lane': change_lanes
            }
        
        ## 最后一个edge的可行驶lane
        last_edge = self.Edges[edge_id[-1]]

            
        # print(available_lane_dict)
        return available_lane_dict

    def get_section_available_lanes(self, cur_section, previous_lanes: List[NormalLane], available_lanes: List[NormalLane]=None) -> List[NormalLane]:
        lane_id_set = set()
        if available_lanes is None:
            available_lanes = []
        for lane in previous_lanes:
            if lane.previous_lane() != None:
                previous_lane = self.WP2Lane(lane.previous_lane())
                available_lanes.append(previous_lane)
                lane_id_set.add(previous_lane.id)
        temp_lanes = available_lanes[::]
        for lane in temp_lanes:
            if len(lane_id_set) == cur_section.lane_num:
                break
            while lane.left_lane():
                if lane.left_lane() not in lane_id_set:
                    lane_id_set.add(lane.left_lane())
                    available_lanes.append(self.NormalLane_Dict[lane.left_lane()])
                lane = self.NormalLane_Dict[lane.left_lane()]
            while lane.right_lane():
                if lane.right_lane() not in lane_id_set:
                    lane_id_set.add(lane.right_lane())
                    available_lanes.append(self.NormalLane_Dict[lane.right_lane()])
                lane = self.NormalLane_Dict[lane.right_lane()]
        return available_lanes

    def get_previous_lane(self, lane_id: str) -> NormalLane:
        # FIXME: 有些lane的previous lane是空的，需要进一步处理
        lane = self.get_lane_by_id(lane_id)
        if isinstance(lane, NormalLane):
            section = lane.affiliated_section
            if section.previous_section_id():
                for wp in lane.wp_list[0].previous(RESOLUTION):
                    if not wp.is_intersection and self.WP2Lane[(wp.road_id, wp.section_id, wp.lane_id)] != lane_id:
                        return self.NormalLane_Dict[self.WP2Lane[(wp.road_id, wp.section_id, wp.lane_id)]]
        return None

    def get_next_lane(self, lane_id: str)  -> Union[NormalLane, JunctionLane]:
        lane = self.get_lane_by_id(lane_id)
        if isinstance(lane, NormalLane):
            next_lanes = list(lane.next_lanes.values())
            if len(next_lanes) > 0:
                # first_next_lane = list(lane.next_lanes.values())[0][0]
                return self.get_lane_by_id(next_lanes[0][0])
            else:
                return None
        elif isinstance(lane, JunctionLane):
            return self.get_lane_by_id(lane.next_lane_id)
        return None

    def __str__(self):
        return 'edges: {}, \nlanes: {}, \njunctions lanes: {}'.format(
            self.Edges.keys(), self.NormalLane_Dict.keys(),
            self.Junction_Dict.keys()
        )
    

class RoadInfoGet:
    def __init__(self, roadgraph: RoadGraph):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        # self.client.load_world('Town06')
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.road_id_to_edge = {}
        self.road_id_to_junction = {}

        self.roadgraph = roadgraph  # 用于更新路网信息

        self.build_edge()

    def build_edge(self):
        # get road id to edge id
        segment_start_wp = [segment[0] for segment in self.map.get_topology()]
        for wp in segment_start_wp:
            if wp.is_intersection or wp.lane_type != carla.LaneType.Driving:
                continue
            add = False
            edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
            for key in self.road_id_to_edge.keys():
                if key == edge_search_key:
                    add = True
                    break
            if add:
                continue

            cur_edge_id = len(self.roadgraph.Edges)
            
            last_segment = self.create_edge(cur_edge_id, wp)
            self.create_section(cur_edge_id, last_segment)
            # self.get_wp_to_edge(cur_edge_id)
        
        self.get_connect()
    
    # def get_wp_to_edge(self, edge_id):
    #     """根据edge_id中存储的last_segment中的waypoint向previous方向遍历，找到该edge上的所有waypoint
    #     在road_id_to_edge中存储waypoint对应的edge_id

    #     # TODO: 划分section和lane

    #     Args:
    #         edge_id (int): edge的序号
    #     """
    #     for wp in self.edge_id[edge_id]['last_segment']:
    #         while not wp.is_intersection:
    #             edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
    #             self.road_id_to_edge[edge_search_key] = edge_id
    #             self.edge_id[edge_id]['road_list'].add(edge_search_key) # TODO: road_list有必要存储吗？
    #             # TODO: previous有可能是空的，需要根据lane_num划分section
    #             for previous_wp in wp.previous(RESOLUTION):
    #                 if not previous_wp.is_intersection:
    #                     wp = previous_wp
    #                     break
    #             wp = previous_wp
                # world.debug.draw_point(wp.transform.location, color=carla.Color(r=0, g=0, b=255), life_time=100)
                

    def create_edge(self, edge_id, wp)->List[carla.Waypoint]:
        edge = Edge(id=edge_id)
        self.roadgraph.Edges[edge_id] = edge

        # 找到这个wp所在的edge的最后一个waypoint
        while wp.next(RESOLUTION) and not wp.next(RESOLUTION)[0].is_intersection:
            wp = wp.next(RESOLUTION)[0]
            # 处理next返回为none的case
            if wp.next(RESOLUTION) == None:
                lwp = wp.get_left_lane()
                if lwp and lwp.lane_type == carla.LaneType.Driving and lwp.lane_id * wp.lane_id > 0:
                    wp = lwp
                    continue
                rwp = wp.get_right_lane()
                if rwp and rwp.lane_type == carla.LaneType.Driving and rwp.lane_id * wp.lane_id > 0:
                    wp = rwp
                    continue
                break
        
        last_segment = []

        """根据最后一个wp找到exit该edge的所有lane的waypoint"""
        # 根据lane_id 进行划分，引用xodr的解释：需要使用中心车道对OpenDRIVE中的车道进行定义和描述。中心车道没有宽度，并被用作车道编号的参考，自身的车道编号为0。对其他车道的编号以中心车道为出发点：车道编号向右呈降序，也就是朝负t方向；向左呈升序，也就是朝正t方向。
        lane_id = wp.lane_id
        lwp = wp.get_left_lane()
        while lwp and lwp.lane_id * lane_id > 0 and lwp.lane_type == carla.LaneType.Driving:
            # self.edge_id[edge_id]['last_segment'].append(lwp)
            last_segment.append(lwp)
            lwp = lwp.get_left_lane()
        # self.edge_id[edge_id]['last_segment'].reverse()
        last_segment.reverse()
        last_segment.append(wp)
        # self.edge_id[edge_id]['last_segment'].append(wp)

        rwp = wp.get_right_lane()
        while rwp and rwp.lane_id * lane_id > 0 and rwp.lane_type == carla.LaneType.Driving:
            # self.edge_id[edge_id]['last_segment'].append(rwp)
            last_segment.append(rwp)
            rwp = rwp.get_right_lane()
        self.roadgraph.Edges[edge_id].last_segment = last_segment[::]
        return last_segment
            
    def create_section(self, edge_id, wp_list):
        """根据edge_id中存储的last_segment中的waypoint向previous方向遍历，找到该edge上的所有waypoint
        在road_id_to_edge中存储waypoint对应的edge_id"""
        # wp_list = self.edge_id[edge_id]['last_segment']
        # cur_edge = copy.copy(self.roadgraph.Edges[edge_id])
        # 记录下lane上的waypoint，后续转化为frenet坐标系

        while wp_list:
            section = Section()
            section.id = str(uuid.uuid4())
            section.affliated_edge = self.roadgraph.Edges[edge_id]
            section.location = wp_list[-1].transform.location
            self.roadgraph.Edges[edge_id].section_num += 1
            self.roadgraph.Edges[edge_id].section_list.append(section.id)

            new_wp = None
            new_wp_list = []
            for wp in wp_list:
                lane = None
                lane = NormalLane()
                lane.id = str(uuid.uuid4())
                lane.lane_id = abs(wp.lane_id)
                lane.affiliated_section = section
                lane.width = wp.lane_width
                lane.lane_change = wp.lane_change

                cur_wp = wp
                last_wp = wp
                edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
                self.road_id_to_edge[edge_search_key] = edge_id
                self.roadgraph.WP2Lane[(wp.road_id, wp.section_id, wp.lane_id)] = lane.id
                self.roadgraph.WP2Section[(wp.road_id, wp.section_id, wp.lane_id)] = section.id

                wp_in_line_list = []
                while not cur_wp.is_intersection:
                    wp_in_line_list.append(cur_wp)
                    if cur_wp.road_id != last_wp.road_id or cur_wp.section_id != last_wp.section_id:
                        new_wp = cur_wp
                        break
                    last_wp = cur_wp
                    if len(cur_wp.previous(RESOLUTION)) == 0:
                        break
                    for wp_pre in cur_wp.previous(RESOLUTION):
                        if not wp_pre.is_intersection:
                            cur_wp = wp_pre
                            break
                    if(cur_wp == last_wp):
                        break
                lane.wp_list = wp_in_line_list[::]
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

        self.roadgraph.Edges[edge_id].section_list.reverse()

    # def get_edge_connect(self):
    #     """找到每个edge的连接关系，并给出通过哪个waypoint可以连接到下一个edge
    #     TODO: 可以通过junction提供的wp得到连接关系，同时需要维护junction的lane
    #     """
    #     for edge_id in self.edge_id:
    #         for wp in self.edge_id[edge_id]['last_segment']:
    #             # 对于每一个edge的最后一个waypoint，找到其下一个路口的waypoint，遍历所有可能
    #             for next_wp in wp.next(RESOLUTION):
    #                 if next_wp.is_intersection:
    #                     # 找到wp所在lane的最后一个waypoint，从而找到它连接的edge
    #                     next_wp_in_normal = next_wp.next_until_lane_end(RESOLUTION)[-1].next(RESOLUTION)[0]
    #                     edge_search_key = str(next_wp_in_normal.road_id) + '_' + str(next_wp_in_normal.section_id) + '_' + str(next_wp_in_normal.lane_id)
    #                     connect_edge_id = self.road_id_to_edge[edge_search_key]
    #                     # TODO:这里的记录对象应该进一步细化？或者是不是应该记录location，然后直接通过方向判断换道？
    #                     if connect_edge_id not in self.edge_id[edge_id]['connect']:
    #                         self.edge_id[edge_id]['connect'][connect_edge_id] = [(wp.road_id, wp.section_id, wp.lane_id)]
    #                     else:
    #                         self.edge_id[edge_id]['connect'][connect_edge_id].append((wp.road_id, wp.section_id, wp.lane_id))

    def get_connect(self):
        
        # segment_end_wp = [segment[1] for segment in self.map.get_topology()]
        junction_points = []
        for edge in self.roadgraph.Edges.values():
            last_wp = edge.last_segment[-1]
            next_wp = last_wp.next(RESOLUTION)
            if next_wp:
                for wp in next_wp:
                    if wp.is_intersection:
                        junction_points.append(wp)
        for wp in junction_points:
            add = True
            if wp.is_intersection and wp.lane_type == carla.LaneType.Driving:   
                add = False
            edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
            if edge_search_key in self.road_id_to_junction.keys():
                add = True
            if add:
                continue

            cur_junction_id = len(self.roadgraph.Intersection)
            
            self.create_junction_lane(cur_junction_id, wp)

    def create_junction_lane(self, junction_id, wp):
        # TODO: 有可能junction_lane的下一个还是junction_lane，需要进一步处理，将他们合并为一个，防止重复
        # junction_lane.start_waypoint.next_until_lane_end(RESOLUTION)??
        # FIXME: 有些junction_lane的start和end不匹配
        junction = wp.get_junction()
        wp_tuple = junction.get_waypoints(carla.LaneType.Driving)
        wp_tuple_dict = {(str(start_wp.road_id) + '_' + str(start_wp.section_id) + '_' + str(start_wp.lane_id)): (start_wp, end_wp) for (start_wp, end_wp) in wp_tuple}
        for i in range(len(wp_tuple)):
            (start_wp, end_wp) = wp_tuple[i]
            junction_lane = JunctionLane()
            junction_lane.id = str(uuid.uuid4())
            junction_lane.start_waypoint = start_wp
            junction_lane.end_waypoint = end_wp
            

            next_wp = end_wp.next(RESOLUTION)
            # wp_is_intersection = list(map(lambda x: x.is_intersection, next_wp))
            junction_list = [start_wp]
            filter_wp = list(filter(lambda x: not x.is_intersection, next_wp))
            if len(filter_wp) == 0:
                while next_wp and (str(next_wp[0].road_id) + '_' + str(next_wp[0].section_id) + '_' + str(next_wp[0].lane_id)) in wp_tuple_dict.keys():
                    junction_list.append(next_wp[0])
                    junction_lane.end_waypoint = wp_tuple_dict[str(next_wp[0].road_id) + '_' + str(next_wp[0].section_id) + '_' + str(next_wp[0].lane_id)][1]
                    next_wp = wp_tuple_dict[str(next_wp[0].road_id) + '_' + str(next_wp[0].section_id) + '_' + str(next_wp[0].lane_id)][1].next(RESOLUTION)
                    filter_wp = list(filter(lambda x: not x.is_intersection, next_wp))
                    if len(filter_wp) != 0:
                        break
                else:
                    print('next_wp is empty')
                    continue
                
            next_edge = self.road_id_to_edge[str(filter_wp[0].road_id) + '_' + str(filter_wp[0].section_id) + '_' + str(filter_wp[0].lane_id)]
            junction_lane.outgoing_edge_id = next_edge
            
                # if next_edge == 41:
                #     self.world.debug.draw_point(wp_i.transform.location, color=carla.Color(r=255, g=0, b=0), life_time=100)

            last_wp = start_wp.previous(RESOLUTION)
            filter_wp = list(filter(lambda x: not x.is_intersection, last_wp))
            if len(filter_wp) == 0:
                continue
            edge_id = self.road_id_to_edge[str(filter_wp[0].road_id) + '_' + str(filter_wp[0].section_id) + '_' + str(filter_wp[0].lane_id)]
            junction_lane.incoming_edge_id = edge_id

            if(next_edge not in self.roadgraph.Edges[edge_id].next_edge_connect):
                self.roadgraph.Edges[edge_id].next_edge_connect[next_edge] = []
            self.roadgraph.Edges[edge_id].next_edge_connect[next_edge].append(junction_lane.id)
            
            for wp_i in junction_list:
                edge_search_key = str(wp_i.road_id) + '_' + str(wp_i.section_id) + '_' + str(wp_i.lane_id)
                self.road_id_to_junction[edge_search_key] = junction_id
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
import random

def random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    rgb_code = '#%02x%02x%02x' % (r, g, b)
    # return carla.Color(r=r, g=g, b=b)
    return rgb_code

if __name__ == '__main__':
    # client = carla.Client('localhost', 2000)
    # client.set_timeout(2.0)
    # client.load_world('Town06')
    # world = client.get_world()
    # 创建一个新的观察者
    # spectator = world.get_spectator()
    # initial_location = carla.Location(x=2000.0, y=0.0, z=1000.0)  # 例如，x, y, z坐标
    # initial_rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)  # 默认旋转

    # # 设置观察者的位置和方向
    # spectator.set_transform(carla.Transform(initial_location, initial_rotation))
    import time
    start = time.time()
    roadgraph = RoadGraph()
    road_info_get = RoadInfoGet(roadgraph)
    print(time.time() - start)
    # print(road_info_get.road_id_to_edge)
    for item in roadgraph.Edges.items():
        # print(item[0], item[1])
        road_info_get.world.debug.draw_string(item[1].last_segment[0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100)

    # test
    spawn_points = road_info_get.map.get_spawn_points()
    origin = carla.Location(spawn_points[100].location)
    destination = carla.Location(spawn_points[87].location)  

    grp = GlobalRoutePlanner(road_info_get.map, 0.5)
    route = grp.trace_route(origin, destination)
    route_edge_list = road_info_get.get_route_edge(route)
    print(route_edge_list)
    # wp_in_junction = route[47]
    # junction = wp_in_junction[0].get_junction()
    # wp_tuple = junction.get_waypoints(carla.LaneType.Driving)
    # for wp_2 in wp_tuple:
    #     world.debug.draw_line(wp_2[0].transform.location, wp_2[1].transform.location, color=carla.Color(r=0, g=255, b=0), life_time=100)
        # world.debug.draw_point(wp_2[1].transform.location, color=carla.Color(r=0, g=255, b=0), life_time=100)

    for wp in route:
        road_info_get.world.debug.draw_point(wp[0].transform.location, color=carla.Color(r=0, g=0, b=255), life_time=50, size=0.1)
    
    # for section in roadgraph.Sections.values():
    #     road_info_get.world.debug.draw_point(section.location, color=carla.Color(r=255, g=0, b=0), life_time=100)
        # for lane_id in section.lanes.values():
        #     lane = roadgraph.NormalLane_Dict[lane_id]
        #     road_info_get.world.debug.draw_string(lane.wp_list[-1].transform.location, lane_id, color=carla.Color(r=0, g=0, b=255), life_time=100)
    available_dict = roadgraph.get_available_lanes(route_edge_list)

    ''' 画出路网'''
    # import matplotlib.pyplot as plt
    # from matplotlib.patches import Polygon
    # fig, ax = plt.subplots(figsize=(10, 10))
    # for edge in roadgraph.Edges.values():
    #     edge_color = random_color()
    #     for section_id in edge.section_list:
    #         section_box = []
    #         section = roadgraph.Sections[section_id]
    #         color = random_color()
    #         for lane_id in section.lanes.values():
    #             lane = roadgraph.NormalLane_Dict[lane_id]
    #             ax.plot([wp.transform.location.x for wp in lane.wp_list], [wp.transform.location.y for wp in lane.wp_list], color=color)
    #         left_lane_id = section.lanes[min(section.lanes.keys())]
    #         right_lane_id = section.lanes[max(section.lanes.keys())]
    #         left_lane = roadgraph.NormalLane_Dict[left_lane_id]
    #         right_lane = roadgraph.NormalLane_Dict[right_lane_id]
    #         section_box.extend([wp_i.transform.location.x, wp_i.transform.location.y] for wp_i in left_lane.wp_list)
    #         section_box.reverse()
    #         section_box.extend([wp_i.transform.location.x, wp_i.transform.location.y] for wp_i in right_lane.wp_list)
    #         ax.add_patch(Polygon(xy=section_box, fill=True, alpha = 0.4, color='red'))
    #     for next_edge, junction_list in edge.next_edge_connect.items():
    #         color = random_color()
    #         for junction_id in junction_list:
    #             junction_lane = roadgraph.Junction_Dict[junction_id]
    #             # junction_box = []
    #             for wp in junction_lane.start_waypoint.next_until_lane_end(RESOLUTION):
    #                 ax.plot([wp.transform.location.x], [wp.transform.location.y], color = color, marker='o', markersize=1)

    # plt.show()


    ''' 画出available lane'''
    for edge_id, available_lanes in available_dict.items():
        # color = random_color()
        # if edge_id !=0:
        #     continue
        for section_id, lanes in available_lanes['available_lane'].items():
            for lane in lanes:
                if lane in available_lanes['change_lane']:
                    continue
                # for wp in lane.wp_list:
                #     road_info_get.world.debug.draw_point(wp.transform.location, color=carla.Color(r=0, g=255, b=0), life_time=10, size=0.05)
                road_info_get.world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=0, g=255, b=0), thickness=1.0, life_time=100)
        for junction_lane in available_lanes['junction_lane']:
            # draw_list = junction_lane.start_waypoint.next_until_lane_end(RESOLUTION)
            road_info_get.world.debug.draw_line(junction_lane.start_waypoint.transform.location, junction_lane.end_waypoint.transform.location, color=carla.Color(r=0, g=0, b=255), thickness=1.0, life_time=100)
        for lane in available_lanes['change_lane']:
            road_info_get.world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=255, g=0, b=0), thickness=1.0, life_time=100)
    # road_info_get.world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, thickness=0.5, color=color, life_time=10)
