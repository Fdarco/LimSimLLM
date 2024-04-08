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

"""

import carla
import numpy as np
import math

class RoadInfoGet:
    def __init__(self, world):
        self.world = world
        self.map = world.get_map()
        self.road_id_to_edge = {}
        self.edge_id = {}
        self.resolution = 0.5
        self.get_road_info()
        

    def get_road_info(self):
        # get road id to edge id
        segment_start_wp = [segment[0] for segment in self.map.get_topology()]
        for wp in segment_start_wp:
            if wp.is_intersection or wp.lane_type != carla.LaneType.Driving:
                continue

            # road_id = wp.road_id
            add = False
            edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
            for key in self.road_id_to_edge.keys():
                if key == edge_search_key:
                    add = True
                    break
            if add:
                continue


            cur_edge_id = len(self.edge_id)
            # 将该road_id所在的edge的所有lane的enter_waypoint和exit_waypoint存储
            self.edge_id[cur_edge_id] = {}
            self.edge_id[cur_edge_id]['road_list'] = set()
            self.edge_id[cur_edge_id]['connect'] = {}
            self.edge_id[cur_edge_id]['last_segment'] = []

            # 从当前位置向前找，直到找到下一个intersection
            while not wp.next(self.resolution)[0].is_intersection:
                wp = wp.next(self.resolution)[0]
            
            # 获取last_segment，注意不能一直get_left_lane或get_right_lane，因为有可能是非Driving的lane，并且会找到对向edge上的lane
            # 根据lane_id 进行划分，引用xodr的解释：需要使用中心车道对OpenDRIVE中的车道进行定义和描述。中心车道没有宽度，并被用作车道编号的参考，自身的车道编号为0。对其他车道的编号以中心车道为出发点：车道编号向右呈降序，也就是朝负t方向；向左呈升序，也就是朝正t方向。
            lane_id = wp.lane_id
            lwp = wp.get_left_lane()
            while lwp and lwp.lane_id * lane_id > 0 and lwp.lane_type == carla.LaneType.Driving:
                self.edge_id[cur_edge_id]['last_segment'].append(lwp)
                lwp = lwp.get_left_lane()

            self.edge_id[cur_edge_id]['last_segment'].append(wp)

            rwp = wp.get_right_lane()
            while rwp and rwp.lane_id * lane_id > 0 and rwp.lane_type == carla.LaneType.Driving:
                self.edge_id[cur_edge_id]['last_segment'].append(rwp)
                rwp = rwp.get_right_lane()
            
            # 从last_segment开始往后找，直到找到下一个intersection
            for wp in self.edge_id[cur_edge_id]['last_segment']:
                while not wp.is_intersection:
                    edge_search_key = str(wp.road_id) + '_' + str(wp.section_id) + '_' + str(wp.lane_id)
                    self.road_id_to_edge[edge_search_key] = cur_edge_id
                    self.edge_id[cur_edge_id]['road_list'].add(edge_search_key)
                    for previous_wp in wp.previous(self.resolution):
                        if not previous_wp.is_intersection:
                            wp = previous_wp
                            break
                    wp = previous_wp
                    # road_id = wp.road_id
                    
                    # TODO: road_list有必要存储吗？

        # 获取connect
        for edge_id in self.edge_id:
            for wp in self.edge_id[edge_id]['last_segment']:
                for next_wp in wp.next(self.resolution):
                    if next_wp.is_intersection:
                        next_wp_in_normal = next_wp.next_until_lane_end(self.resolution)[-1].next(self.resolution)[0]
                        edge_search_key = str(next_wp_in_normal.road_id) + '_' + str(next_wp_in_normal.section_id) + '_' + str(next_wp_in_normal.lane_id)
                        # road_id = next_wp_in_normal.road_id
                        connect_edge_id = self.road_id_to_edge[edge_search_key]
                        # TODO:这里的记录对象应该进一步细化？或者是不是应该记录location，然后直接通过方向判断换道？
                        if connect_edge_id not in self.edge_id[edge_id]['connect']:
                            self.edge_id[edge_id]['connect'][connect_edge_id] = [(wp.road_id, wp.section_id, wp.lane_id)]
                        else:
                            self.edge_id[edge_id]['connect'][connect_edge_id].append((wp.road_id, wp.section_id, wp.lane_id))


if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    road_info_get = RoadInfoGet(world)
    # print(road_info_get.road_id_to_edge)
    for item in road_info_get.edge_id.items():
        # print(item[0], item[1])
        world.debug.draw_string(item[1]['last_segment'][0].transform.location, str(item[0]), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100)
