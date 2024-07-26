# from utils.roadgraph import AbstractLane, JunctionLane, NormalLane
import utils.roadgraph as rd

from typing import Union

from simModel_Carla.Roadgraph import RoadGraph
# from simModel_Carla.Network_Structure import NormalLane,JunctionLane,Edge,Section,AbstractLane
import Network_Structure as cl


class carlaRoadGraphWrapper(rd.RoadGraph):
    def __init__(self,roadgraph:RoadGraph):
        self.roadgraph=roadgraph

    def get_lane_by_id(self, lane_id: str):
        lane=self.roadgraph.get_lane_by_id(lane_id)
        if isinstance(lane,cl.NormalLane):
            normal_lane=carlaNormalLaneWrapper(lane)
            #next_lane:{to_normal_lane_id:(via_juction_lane_id,’direction’}
            if lane.id in self.roadgraph.Normal2Junction.keys():
                junction_lane_list= self.roadgraph.Normal2Junction[lane.id]
                for junction_lane_id in junction_lane_list:
                    next_lane_id=self.roadgraph.WP2Lane[self.roadgraph.Junction_Dict[junction_lane_id].next_lane]
                    normal_lane.next_lanes[next_lane_id]=(junction_lane_id,'fake')

            return normal_lane

        elif isinstance(lane,cl.JunctionLane):
            junction_lane=carlaJunctionLaneWrapper(lane)
            junction_lane.next_lane_id=self.roadgraph.WP2Lane[junction_lane.lane.next_lane]
            return junction_lane
        else:
            return None

    def get_available_next_lane(self, lane_id: str, available_lanes) -> rd.AbstractLane:
        #modified from simModel_Carla.Vehicle.get_available_next_lane
        lane = self.roadgraph.get_lane_by_id(lane_id)
        if isinstance(lane, cl.NormalLane):
            # 直接查找相连的lane
            if lane.next_lane:
                next_lane_id = self.roadgraph.WP2Lane[lane.next_lane]
                if next_lane_id in available_lanes:
                    return self.get_lane_by_id(next_lane_id)
            # 查找相连的junction lane
            else:
                next_junction_list = self.roadgraph.get_Normal2Junction(lane.id)
                for next_lane_i in next_junction_list:
                    if next_lane_i in available_lanes:
                        return self.get_lane_by_id(next_lane_i)

        # 如果是junction lane，则直接查找和他相连的lane
        elif isinstance(lane, cl.JunctionLane):
            next_lane_id = self.roadgraph.WP2Lane[lane.next_lane]
            if next_lane_id in available_lanes:
                return self.get_lane_by_id(next_lane_id)
        return None

    def get_next_lane(self, lane_id: str) -> Union[rd.NormalLane, rd.JunctionLane]:
        lane = self.roadgraph.get_lane_by_id(lane_id)
        if isinstance(lane, cl.NormalLane):
            if lane.next_lane:
                next_lane = self.get_lane_by_id(self.roadgraph.WP2Lane[lane.next_lane])
                return next_lane
            else:
                return None
        elif isinstance(lane, cl.JunctionLane):
            if lane.next_lane:
                next_lane = self.get_lane_by_id(self.roadgraph.WP2Lane[lane.next_lane])
                return next_lane
            else:
                return None
        return None


class carlaNormalLaneWrapper(rd.NormalLane):
    def __init__(self,lane:cl.NormalLane):
        self.lane=lane
        self.id=lane.id
        self.course_spline=lane.course_spline
        self.next_lanes= {}
        self.width=lane.width
        self.affiliated_edge=carlaEdgeWrapper(self.lane.affiliated_section.affliated_edge.id)
        self.speed_limit=30#TODO:how to get and what unit
    def left_lane(self) -> str:
        return self.lane.left_lane
    def right_lane(self) -> str:
        return self.lane.right_lane
    @property
    def spline_length(self):
        return self.lane.length

class carlaJunctionLaneWrapper(rd.JunctionLane):
    def __init__(self,lane:cl.JunctionLane):
        self.lane=lane
        self.id=lane.id
        self.width=lane.width
        self.course_spline=lane.course_spline
        self.next_lane_id=''
        self.affJunc=lane.id
        self.speed_limit=30#TODO:how to get and what unit
        self.currTlState=lane.currTlState

    @property
    def spline_length(self):
        return self.lane.length

class carlaEdgeWrapper(rd.Edge):
    def __init__(self,id):
        self.id=id

    def __eq__(self, other):
        return self.id==other.id