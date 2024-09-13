"""
Get road information from carla map
"""
import sys

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from Roadgraph import RoadGraph
from Network_Structure import Edge, Section, NormalLane, JunctionLane
from typing import Tuple
from typing import List

RESOLUTION = 0.5

class RoadInfoGet:
    def __init__(self, roadgraph: RoadGraph, topology: List[Tuple[carla.Waypoint, carla.Waypoint]]):
        """Get road information from carla map

        Args:
            roadgraph (RoadGraph): roadgraph
            topology (List[Tuple[carla.Waypoint, carla.Waypoint]]): carla map topology. result of carla_map.get_topology()
        """
        self.topology = topology
        self.road_id_to_edge = {} # road_id-section_id-lane_id: edge_id
        self.road_id_to_junction = {} # road_id-section_id-lane_id: junction_lane_id

        self.roadgraph = roadgraph  # 用于更新路网信息

        self.build_edge()

    def build_edge(self):
        """根据topology建立edge和section
        
        1. 采样所有的waypoint，构建edge和section
        2. 根据edge的last_segment找到junction_lane，获得连接关系
        """
        segment_start_wp = [segment[0] for segment in self.topology]

        # -- 采样所有的waypoint，构建edge和section -- #
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
        
        self.update_lane()
    
    def update_lane(self):
        """update relationship in lane
        """
        for nid,normal_lane in self.roadgraph.NormalLane_Dict.items():
            if normal_lane.next_lane:
                normal_lane.next_lane_id=self.roadgraph.WP2Lane[normal_lane.next_lane]
            if normal_lane.previous_lane:
                normal_lane.previous_lane_id=self.roadgraph.WP2Lane[normal_lane.previous_lane]
        for jid,junction_lane in self.roadgraph.Junction_Dict.items():
            if junction_lane.previous_lane:
                junction_lane.previous_lane_id=self.roadgraph.WP2Lane[junction_lane.previous_lane]
            if junction_lane.next_lane:
                junction_lane.next_lane_id=self.roadgraph.WP2Lane[junction_lane.next_lane]
                

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

        # ------- 根据最后一个wp找到exit该edge的所有lane的waypoint -------- #
        # 根据lane_id 进行划分，引用xodr的解释：需要使用中心车道对OpenDRIVE中的车道进行定义和描述。中心车道没有宽度，并被用作车道编号的参考，自身的车道编号为0。对其他车道的编号以中心车道为出发点：车道编号向右呈降序，也就是朝负t方向；向左呈升序，也就是朝正t方向。

        lane_id = wp.lane_id
        lwp = wp.get_left_lane()
        while lwp and lwp.lane_id * lane_id > 0 and lwp.lane_type == carla.LaneType.Driving:
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
            # 根据wp_list创建一个section， wp_list是每个section的最后一排waypoint
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

        
        if len(lane.wp_list)>1:
            lane.get_spline2D()
        else:
            #In case of the lane that only has one wp,choose smaller Resolution to go over again
            test=lane.wp_list[-1].next_until_lane_end(RESOLUTION/5)
            end_wp=test[-1]
            cp=end_wp.previous(RESOLUTION/5)[0]
            tmp_wp_list=[end_wp]
            while cp.road_id==end_wp.road_id and cp.section_id==end_wp.section_id:
                tmp_wp_list.append(cp)
                cp=cp.previous(RESOLUTION/5)[0]
            lane.wp_list = tmp_wp_list[::-1]
            assert len(lane.wp_list)>1

            lane.get_spline2D()
            print('debuging')
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
        junction = wp.get_junction() # 获取wp所在的junction
        wp_tuple = junction.get_waypoints(carla.LaneType.Driving) # 获取junction上的每段[start_wp, end_wp]
        for i in range(len(wp_tuple)):
            (start_wp, _) = wp_tuple[i]

            # 如果start_wp的previous是normal_lane，说明start_wp是junction_lane的起点，构建junction_lane
            last_wp = start_wp.previous(RESOLUTION)
            filter_wp = list(filter(lambda x: not x.is_intersection, last_wp))
            if len(filter_wp) == 0:
                continue
            junction_lane = JunctionLane(width=start_wp.lane_width)
            junction_lane.node_id=start_wp.junction_id

            # 获取junction_lane的waypoint，可以得到从start_wp开始直到遇到normal_lane的所有waypoint
            junction_lane.wp_list = [start_wp] + start_wp.next_until_lane_end(RESOLUTION)
            
            # 获取junction_lane的incoming_edge_id和outgoing_edge_id
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
            if type(junction_lane.outgoing_edge_id)==int or junction_lane.incoming_edge_id==int:
                #current code does not consider the case where one junction is connected directly to another junction
                return
            junction_lane.id = junction_lane.incoming_edge_id + '-' + junction_lane.outgoing_edge_id + '-' + str(i)

            # 增加edge的connect信息
            if(next_edge not in self.roadgraph.Edges[edge_id].next_edge_connect):
                self.roadgraph.Edges[edge_id].next_edge_connect[next_edge] = []
            self.roadgraph.Edges[edge_id].next_edge_connect[next_edge].append(junction_lane.id)
            
            for wp_i in junction_lane.wp_list:
                edge_search_key = str(wp_i.road_id) + '_' + str(wp_i.section_id) + '_' + str(wp_i.lane_id)
                self.road_id_to_junction[edge_search_key] = junction_lane.id
                self.roadgraph.WP2Lane[(wp_i.road_id, wp_i.section_id, wp_i.lane_id)] = junction_lane.id

            junction_lane.get_spline2D()
            # update roadgraph
            self.roadgraph.Junction_Dict[junction_lane.id] = junction_lane
            previous_lane_id = self.roadgraph.WP2Lane[junction_lane.previous_lane]
            if previous_lane_id not in self.roadgraph.Normal2Junction:
                self.roadgraph.Normal2Junction[previous_lane_id] = [junction_lane.id]
            else:
                self.roadgraph.Normal2Junction[previous_lane_id].append(junction_lane.id)
            
if __name__ == '__main__':
    # -------------------- example ------------------- #
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    # client.load_world('Town06')
    world = client.get_world()
    carla_map = world.get_map()
    topology = carla_map.get_topology()
    

    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)

    # test_point=carla_map.get_waypoint_xodr(652,4,35.36)
    # previous_test_point=test_point.previous(0.5)[0]
    # test_wp_list=previous_test_point.next_until_lane_end(0.1)
    # world.debug.draw_string(test_point.transform.location, str('A'), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)
    # world.debug.draw_string(previous_test_point.transform.location,'B', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)
    # test_point=carla_map.get_waypoint_xodr(64,-5,121)
    # world.debug.draw_string(test_point.transform.location, str('A'), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)

    
    roadgraph = RoadGraph()
    road_info_get = RoadInfoGet(roadgraph, topology) # 用RoadInfoGet去更新roadgraph

    from draw import draw_roadgraph
    draw_roadgraph(roadgraph=roadgraph)

    for item in roadgraph.Edges.items():
        world.debug.draw_string(item[1].last_segment[0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=10000)

    # -------------------- test ------------------- #
    spawn_points = carla_map.get_spawn_points() # 选出适合放置车辆的位置
    origin = carla.Location(spawn_points[20].location)
    destination = carla.Location(spawn_points[120].location)  

    grp = GlobalRoutePlanner(carla_map, 0.5)
    route = grp.trace_route(origin, destination)
    route_edge_list = roadgraph.get_route_edge(route)
    print(route_edge_list)

    for wp in route:
        world.debug.draw_point(wp[0].transform.location, color=carla.Color(r=0, g=0, b=255), life_time=50, size=0.1)
    
    available_dict = roadgraph.get_all_available_lanes(route_edge_list, route[-1][0])

    # # ----------------- 在carla中画出available lane ----------------- #
    # for edge_id, available_lanes in available_dict.items():
    #     for section_id, lanes in available_lanes['available_lane'].items():
    #         for lane in lanes:
    #             # if lane in available_lanes['change_lane']:
    #             #     continue
    #             world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=0, g=255, b=0), thickness=1.0, life_time=100)
    #     for junction_lane in available_lanes['junction_lane']:
    #         world.debug.draw_line(junction_lane.start_wp.transform.location+carla.Location(z=1), junction_lane.end_wp.transform.location+carla.Location(z=1), color=carla.Color(r=0, g=0, b=255), thickness=1.0, life_time=100)
    #     for section_id, lanes in available_lanes['change_lane'].items():
    #         for lane in lanes:
    #             world.debug.draw_line(lane.wp_list[0].transform.location, lane.wp_list[-1].transform.location, color=carla.Color(r=255, g=0, b=0), thickness=1.0, life_time=100)
    
    for junction_lane in list(roadgraph.Junction_Dict.values()):
        world.debug.draw_line(junction_lane.start_wp.transform.location+carla.Location(z=1), junction_lane.end_wp.transform.location+carla.Location(z=1), color=carla.Color(r=0, g=0, b=255), thickness=1.0, life_time=100)
    # for junction_lane in list(roadgraph.NormalLane_Dict.values()):
    #     world.debug.draw_line(junction_lane.start_wp.transform.location+carla.Location(z=1), junction_lane.end_wp.transform.location+carla.Location(z=1), color=carla.Color(r=255, g=0, b=255), thickness=1.0, life_time=100)