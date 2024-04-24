from RoadInfoGet import RoadInfoGet
import carla


# 每个vehicle自行维护自己的路线信息
class Vehicle:
    def __init__(self, route):
        self.route = route

    
    # def get_available_next_lane(self, cur_wp: carla.Waypoint, available_lanes: dict) -> AbstractLane:
    #     lane = self.get_lane_by_id(lane_id)
    #     if isinstance(lane, NormalLane):
    #         for next_lane_i in lane.next_lanes.values():
    #             if next_lane_i[0] in available_lanes:
    #                 return self.get_lane_by_id(next_lane_i[0])
    #     elif isinstance(lane, JunctionLane):
    #         if lane.next_lane_id in available_lanes:
    #             return self.get_lane_by_id(lane.next_lane_id)
    #     return None