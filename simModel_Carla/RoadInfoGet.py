"""
    road_id_to_edge: dict, find the edge id by (road_id, section_id, lane_id)
    edge_id: dict, {
            'edge_id': {
                road_list: {road_id:{section_id:{lane_id: (enter_waypoint, exit_waypoint)}...}...}
                last_segment: [[road_id, sectoin_id, lane_id]]
                connect: {edge_id: [[road_id, section_id, lane_id]]}
                lane之间的联通关系？ lane_change
            }...
        }
    
    当前版本的bug: 对于只出现一次的waypoint可能会导致其他waypoint之前存储的edge_id被覆盖，从而出现闲置的edge_id

"""

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from trafficManager.common.vehicle import Behaviour
from typing import Tuple

class RoadInfoGet:
    def __init__(self, world):
        self.world = world
        self.map = world.get_map()
        self.road_id_to_edge = {}
        self.edge_id = {}
        self.resolution = 0.5
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

            cur_edge_id = len(self.edge_id)
            
            self.create_edge(cur_edge_id, wp)
            self.get_wp_to_edge(cur_edge_id)
        
        self.get_edge_connect()
    
    def get_wp_to_edge(self, edge_id):
        """根据edge_id中存储的last_segment中的waypoint向previous方向遍历，找到该edge上的所有waypoint
        在road_id_to_edge中存储waypoint对应的edge_id

        Args:
            edge_id (int): edge的序号
        """
        for wp in self.edge_id[edge_id]['last_segment']:
            while not wp.is_intersection:
                edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
                self.road_id_to_edge[edge_search_key] = edge_id
                self.edge_id[edge_id]['road_list'].add(edge_search_key) # TODO: road_list有必要存储吗？
                for previous_wp in wp.previous(self.resolution):
                    if not previous_wp.is_intersection:
                        wp = previous_wp
                        break
                wp = previous_wp
                

    def create_edge(self, edge_id, wp):
        self.edge_id[edge_id] = {}
        self.edge_id[edge_id]['road_list'] = set()
        self.edge_id[edge_id]['connect'] = {}
        self.edge_id[edge_id]['last_segment'] = []

        # 找到这个wp所在的edge的最后一个waypoint
        while not wp.next(self.resolution)[0].is_intersection:
            wp = wp.next(self.resolution)[0]
        
        """根据最后一个wp找到exit该edge的所有lane的waypoint"""
        # 根据lane_id 进行划分，引用xodr的解释：需要使用中心车道对OpenDRIVE中的车道进行定义和描述。中心车道没有宽度，并被用作车道编号的参考，自身的车道编号为0。对其他车道的编号以中心车道为出发点：车道编号向右呈降序，也就是朝负t方向；向左呈升序，也就是朝正t方向。
        lane_id = wp.lane_id
        lwp = wp.get_left_lane()
        while lwp and lwp.lane_id * lane_id > 0 and lwp.lane_type == carla.LaneType.Driving:
            self.edge_id[edge_id]['last_segment'].append(lwp)
            lwp = lwp.get_left_lane()

        self.edge_id[edge_id]['last_segment'].append(wp)

        rwp = wp.get_right_lane()
        while rwp and rwp.lane_id * lane_id > 0 and rwp.lane_type == carla.LaneType.Driving:
            self.edge_id[edge_id]['last_segment'].append(rwp)
            rwp = rwp.get_right_lane()
            
        return
            
    def get_edge_connect(self):
        """找到每个edge的连接关系，并给出通过哪个waypoint可以连接到下一个edge
        """
        for edge_id in self.edge_id:
            for wp in self.edge_id[edge_id]['last_segment']:
                # 对于每一个edge的最后一个waypoint，找到其下一个路口的waypoint，遍历所有可能
                for next_wp in wp.next(self.resolution):
                    if next_wp.is_intersection:
                        # 找到wp所在lane的最后一个waypoint，从而找到它连接的edge
                        next_wp_in_normal = next_wp.next_until_lane_end(self.resolution)[-1].next(self.resolution)[0]
                        edge_search_key = str(next_wp_in_normal.road_id) + '_' + str(next_wp_in_normal.section_id) + '_' + str(next_wp_in_normal.lane_id)
                        connect_edge_id = self.road_id_to_edge[edge_search_key]
                        # TODO:这里的记录对象应该进一步细化？或者是不是应该记录location，然后直接通过方向判断换道？
                        if connect_edge_id not in self.edge_id[edge_id]['connect']:
                            self.edge_id[edge_id]['connect'][connect_edge_id] = [(wp.road_id, wp.section_id, wp.lane_id)]
                        else:
                            self.edge_id[edge_id]['connect'][connect_edge_id].append((wp.road_id, wp.section_id, wp.lane_id))

    def get_route_edge(self, route) -> set:
        route_edge_list = set()
        for wp in route:
            edge_search_key = str(wp[0].road_id) + '_' + str(wp[0].section_id) + '_' + str(wp[0].lane_id)
            if edge_search_key in self.road_id_to_edge:
                route_edge_list.add(self.road_id_to_edge[edge_search_key])
        
        return route_edge_list
            

    def next_available_action(self, wp) -> Tuple[Behaviour]:

        return Behaviour.LCL

if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    road_info_get = RoadInfoGet(world)
    # print(road_info_get.road_id_to_edge)
    for item in road_info_get.edge_id.items():
        # print(item[0], item[1])
        world.debug.draw_string(item[1]['last_segment'][0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100)

    # test
    spawn_points = road_info_get.map.get_spawn_points()
    origin = carla.Location(spawn_points[18].location)
    destination = carla.Location(spawn_points[10].location)  

    grp = GlobalRoutePlanner(road_info_get.map, 0.5)
    route = grp.trace_route(origin, destination)
    route_edge_list = road_info_get.get_route_edge(route)
    print(route_edge_list)
