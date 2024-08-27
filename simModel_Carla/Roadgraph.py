"""
Roadgraph is a data structure that represents the road network of the map. It is composed of edges, sections, lanes and junction_lanes.
"""

import carla
from typing import Tuple
import sqlite3
from dataclasses import dataclass, field
from abc import ABC
from typing import Dict, Union, Set, List
import logging
from Network_Structure import Edge, Section, NormalLane, JunctionLane
from simModel_Carla.DataQueue import (ERD,JLRD,LRD,RGRD)
from queue import Queue
from datetime import datetime
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
    dataQue=Queue()


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

    def exportRenderData(self,ego,vehINAoI,outOfAoI):
        roadgraphRenderData = RGRD()

        for eid,edge in self.Edges.items():
            lane_count=0
            for section_id in edge.section_list:
                section=self.Sections[section_id]
                for _,lane_id in section.lanes.items():
                    lane=self.NormalLane_Dict[lane_id]

                    try:
                        roadgraphRenderData.lanes[lane.id] = LRD(
                            lane.id, lane.left_bound, lane.right_bound)
                    except AttributeError:
                        lane.getPlotElem()
                        roadgraphRenderData.lanes[lane.id] = LRD(
                            lane.id, lane.left_bound, lane.right_bound)
                        
                    lane_count+=1
            roadgraphRenderData.edges[eid] = ERD(eid, lane_count)

        #TODO:需要读取交通信号灯，并将其和junctionlane关联起来
        for jid,junction_lane in self.Junction_Dict.items():
            tls=junction_lane.currTlState
            try:
                roadgraphRenderData.junction_lanes[jid] = JLRD(
                    jid, junction_lane.center_line, tls
                )
            except AttributeError:
                junction_lane.getPlotElem()
                roadgraphRenderData.junction_lanes[jid] = JLRD(
                    jid, junction_lane.center_line, tls
                )

        # export vehicles' information using dict.
        VRDDict = {
            'egoCar': [ego.exportVRD(), ],
            'carInAoI': [av.exportVRD() for av in vehINAoI.values()],
            'outOfAoI': [sv.exportVRD() for sv in outOfAoI.values()]
        }

        return roadgraphRenderData, VRDDict


    def get_available_next_lane(self, lane_id: str,available_lanes: set):
        """获取当前和当前lane连接的下一个available lane

        Args:
            roadgraph (RoadGraph): 路网信息
            lane_id (str): current lane id

        Returns:
            AbstractLane: 下一个available lane
        """
        lane = self.get_lane_by_id(lane_id)
        if isinstance(lane, NormalLane):
            # 直接查找相连的lane
            if lane.next_lane:
                next_lane_id = self.WP2Lane[lane.next_lane]
                if next_lane_id in available_lanes:
                    return self.get_lane_by_id(next_lane_id)
            # 查找相连的junction lane
            else:
                next_junction_list = self.get_Normal2Junction(lane.id)
                for next_lane_i in next_junction_list:
                    if next_lane_i in available_lanes:
                        return self.get_lane_by_id(next_lane_i)

        # 如果是junction lane，则直接查找和他相连的lane
        elif isinstance(lane, JunctionLane):
            next_lane_id = self.WP2Lane[lane.next_lane]
            if next_lane_id in available_lanes:
                return self.get_lane_by_id(next_lane_id)
        return None

    def get_traffic_light(self, world:carla.World):
        carla_junction_ids = []
        for key, junction_lane in self.Junction_Dict.items():
            if junction_lane.wp_list[0].junction_id not in carla_junction_ids:
                carla_junction_ids.append(junction_lane.wp_list[0].junction_id)

        traffic_lights:Dict[str,carla.TrafficLight]={}
        for j_id in carla_junction_ids:
            tls=world.get_traffic_lights_in_junction(j_id)
            for tl in tls:
                wp_list=tl.get_affected_lane_waypoints()
                for wp in wp_list:
                    wp_tuple=(wp.road_id,wp.section_id,wp.lane_id)
                    lane_id=self.WP2Lane[wp_tuple]
                    if lane_id in self.Junction_Dict.keys():
                        if lane_id not in traffic_lights.keys():
                            traffic_lights[lane_id]=tl
                    elif lane_id in self.NormalLane_Dict.keys():
                        if lane_id in self.Normal2Junction.keys():
                            junction_lane_ids=self.Normal2Junction[lane_id]
                            for junction_lane_id in junction_lane_ids:
                                if junction_lane_id in self.Junction_Dict.keys():
                                    if junction_lane_id not in traffic_lights.keys():
                                        traffic_lights[junction_lane_id] = tl
                    else:
                        continue

        for jid,tl in traffic_lights.items():
            junction_lane=self.Junction_Dict[jid]
            junction_lane.traffic_light=tl


    def getDataQue(self):
        #junction lane
        for jid,junction_lane in self.Junction_Dict.items():
            self.dataQue.put((
            'junctionLaneINFO', (
                jid, round(junction_lane.width,2), float(15.0), round(junction_lane.length,2), 0,#TODO:id,width,speedlimit,length,tlsindex
                '', '', 'pedestrian tram rail_urban rail rail_electric rail_fast ship'
            ), 'INSERT'
             ))

        #normal lane
        for lid,normal_lane in self.NormalLane_Dict.items():
            rawShape=''
            for idx,wp in enumerate(normal_lane.wp_list):
                rawShape+=str(wp.transform.location.x)+','+str(wp.transform.location.y)
                if idx !=len(normal_lane.wp_list)-1:
                    rawShape+=' ' 
            
            self.dataQue.put((
                'laneINFO', (
                    lid, rawShape, round(normal_lane.width,2), float(30.0), normal_lane.affiliated_section.affliated_edge.id, 
                    round(normal_lane.length,2), 'driving', '', 'pedestrian tram rail_urban rail rail_electric rail_fast ship'
                ), 'INSERT'
            ))
        
        #edge
        for eid,edge in self.Edges.items():
            lanes=[]
            for sid in edge.section_list:
                cs=self.Sections[sid]
                for lane_id in cs.lanes.values():
                    lanes.append(lane_id)
            for jid,junction_lane in self.Junction_Dict.items():
                idlist=junction_lane.id.split('-')
                for i,id in enumerate(idlist):
                    if eid == id and i==0:
                        toNode = junction_lane.node_id
                    elif eid == id and i==1:
                        fromNode = junction_lane.node_id
            self.dataQue.put((
                    'edgeINFO', (eid,len(lanes), str(fromNode), str(toNode)), 'INSERT'
                ))
        
        #connection
        viewed_junction=set()
        for jid,junction_lane in self.Junction_Dict.items():
            fromLaneID=self.WP2Lane[junction_lane.previous_lane]
            toLaneID=self.WP2Lane[junction_lane.next_lane]
            self.dataQue.put((
            'connectionINFO', (
                fromLaneID, toLaneID, 's', junction_lane.id#TODO cant get correct direction
            ), 'INSERT'))

            if junction_lane.node_id not in viewed_junction:
                self.dataQue.put((
                                'junctionINFO', (junction_lane.node_id, ''), 'INSERT'#TODO:cant find shape
                            ))
                viewed_junction.add(junction_lane.node_id)
                
        # self.dataQue.put((
        #         'geohashINFO',
        #         (ghx, ghy, ghEdges, ghJunctions), 'INSERT'
        #     ))

    def insertCommit(self,dataBase):
        
        conn = sqlite3.connect(dataBase, check_same_thread=False)
        cur = conn.cursor()
        commitCnt = 0
        while not self.dataQue.empty():
            tableName, data, process = self.dataQue.get()
            sql = '{} INTO {} VALUES '.format(process, tableName) + \
                '(' + '?,'*(len(data)-1) + '?' + ')'
            try:
                cur.execute(sql, data)
            except sqlite3.OperationalError as e:
                print(sql, data)
            commitCnt += 1
            if commitCnt == 10000:
                conn.commit()
                commitCnt = 0
        conn.commit()
        cur.close()
        conn.close()

        print('[green bold]Network information commited at {}.[/green bold]'.format(
            datetime.now().strftime('%H:%M:%S.%f')[:-3]))