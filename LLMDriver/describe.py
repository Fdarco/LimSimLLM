import time

from trafficManager.common.observation import Observation
from trafficManager.common.vehicle import Behaviour, Vehicle, VehicleType
from trafficManager.decision_maker.abstract_decision_maker import EgoDecision, MultiDecision
from trafficManager.planner.abstract_planner import AbstractEgoPlanner
from trafficManager.predictor.abstract_predictor import Prediction

import logger, logging
import trafficManager.planner.trajectory_generator as traj_generator
from utils.obstacles import DynamicObstacle, ObsType, Rectangle
from utils.roadgraph import JunctionLane, NormalLane, RoadGraph, AbstractLane
from utils.trajectory import State, Trajectory

from trafficManager.planner.frenet_optimal_planner import frenet_optimal_planner
from trafficManager.common import cost

from typing import List, Tuple, Optional, Union, Dict
from datetime import datetime
import math
import os
import numpy as np
import json
import matplotlib.pyplot as plt

ACTIONS_ALL = {
    Behaviour.LCL: 'LANE_LEFT',
    Behaviour.IDLE: 'IDLE',
    Behaviour.LCR: 'LANE_RIGHT',
    Behaviour.AC: 'FASTER',
    Behaviour.DC: 'SLOWER'
}

ACTIONS_DESCRIPTION = {
    Behaviour.LCL: 'Turn-left - change lane to the left of the current lane',
    Behaviour.IDLE: 'IDLE - remain in the current lane with current speed',
    Behaviour.LCR: 'Turn-right - change lane to the right of the current lane',
    Behaviour.AC: 'Acceleration - accelerate the vehicle',
    Behaviour.DC: 'Deceleration - decelerate the vehicle'
}

# 明确需要哪些信息
class EnvScenario:
    def __init__(
            self, config
    ) -> None:
        
        self.SV: List[Vehicle] = []
        self.ego_vehicle = None
        self.config = config
        self.current_lane = None

        self.ego_prediction = None

        self.decision = None

        self.last_lane = 0

        with open(config["DESCRIBE_JSON"], 'r', encoding="utf-8") as f:
            self.des_json = json.load(f)

        self.logger = logger.setup_app_level_logger(logger_name = "prompt", file_name="prompt_debug.log")
        self.logging = logging.getLogger("prompt").getChild(__name__)

        # TODO：数据库操作待增加
        # if database:
        #     self.database = database
        # else:
        #     self.database = datetime.strftime(
        #         datetime.now(), '%Y-%m-%d_%H-%M-%S'
        #     ) + '.db'

        # if os.path.exists(self.database):
        #     os.remove(self.database)

        # self.dbBridge = DBBridge(self.database, env)

        # self.dbBridge.createTable()
        # self.dbBridge.insertSimINFO(envType, seed)
        # self.dbBridge.insertNetwork()

    def getLaneInfo(self, roadgraph: RoadGraph) -> str:
        """获取当前和下一车道的信息，包括车道的长度，车道的宽度，车道的类型等，如果包括junction lane，还需要包括交通灯信息

        Args:
            roadgraph (RoadGraph): 路网信息

        Returns:
            str: 对当前和下一车道的描述
        """
        if isinstance(self.current_lane, NormalLane):
            edge = self.current_lane.affiliated_edge
            edge_num = 0
            for lane_id in edge.lanes:
                lane = roadgraph.get_lane_by_id(lane_id)
                if lane.width > self.config["vehicle"]["car"]["width"]:
                    edge_num += 1
            # edge_num = edge.available_edge_num()
            left_lane_num = 1
            left_lane_id = self.current_lane.left_lane()
            
            while left_lane_id != None:
                # print("left_lane_id is ", left_lane_id)
                left_lane = roadgraph.get_lane_by_id(left_lane_id)
                if left_lane.width > self.config["vehicle"]["car"]["width"]:
                    left_lane_num += 1
                left_lane_id = left_lane.left_lane()
            # print("edge_id is ", edge.id)
            # print("left_lane_num: ", left_lane_num)
            current_lane_description = self.des_json["basic_description"]["current_lane_scenario_description"]["normal_lane"].format(edge_num = edge_num, left_lane_num = left_lane_num, lane_length = round(self.current_lane.course_spline.s[-1], 3))
        else:
            current_lane_description = self.des_json["basic_description"]["current_lane_scenario_description"]["junction_lane"]

        current_lane_description += self.des_json["basic_description"]["traffic_info_description"].format(speed_limit = self.current_lane.speed_limit)

        next_lane = roadgraph.get_available_next_lane(self.current_lane.id, self.ego_vehicle.available_lanes)
        if next_lane != None:
            if isinstance(next_lane, NormalLane):
                edge = next_lane.affiliated_edge
                edge_num = 0
                for lane_id in edge.lanes:
                    lane = roadgraph.get_lane_by_id(lane_id)
                    if lane.width > self.config["vehicle"]["car"]["width"]:
                        edge_num += 1
                # edge_num = edge.available_edge_num()
                next_lane_description = self.des_json["basic_description"]["next_lane_scenario_description"]["normal_lane"].format(edge_num = edge_num)
            else:
                if next_lane.tlLogic == "actuated":
                    tl_state = "with"
                else:
                    tl_state = "without"
                dis_stop_line = round(self.current_lane.spline_length - self.ego_vehicle.current_state.s, 3)
                # print("next_junction_id is ", next_lane.id)
                # print("current_lane.spline_length: ", self.current_lane.spline_length)
                # print("ego_vehicle.current_state.s: ", self.ego_vehicle.current_state.s)
                next_lane_description = self.des_json["basic_description"]["next_lane_scenario_description"]["junction_lane"].format(tl_state = tl_state, dis_stop_line = dis_stop_line)

                if tl_state == "with":
                    next_lane_description += self.des_json["basic_description"]["traffic_light_description"].format(curr_tl_state = next_lane.currTlState, next_tl_state = next_lane.nextTlState, switch_time = next_lane.switchTime)
        else:
            next_lane_description = ""
        
        return current_lane_description + next_lane_description

    def getEgoInfo(self) -> str:
        """获取自车的信息，包括自车的位置，自车的速度，自车的加速度等

        Returns:
            str: 对于自车的描述
        """
        ego_position_x = round(self.ego_vehicle.current_state.x, 3)
        ego_position_y = round(self.ego_vehicle.current_state.y, 3)
        ego_position = "'("+ str(ego_position_x) + "," + str(ego_position_y) +")'"
        ego_lane_position = round(self.ego_vehicle.current_state.s, 3)
        ego_speed = round(self.ego_vehicle.current_state.vel, 3)
        ego_acceleration = round(self.ego_vehicle.current_state.acc, 3)
        ego_description = self.des_json["basic_description"]["ego_state_description"].format(ego_position = ego_position, ego_lane_position = ego_lane_position, ego_speed = ego_speed, ego_acceleration = ego_acceleration)
        return ego_description + "\n"

    def getEmergencyInfo(self, roadgraph: RoadGraph) -> str:
        """对于紧急情况的描述，包括交通灯的状态，是否需要换道等

        Args:
            roadgraph (RoadGraph): 路网信息

        Returns:
            str: 对于紧急情况的描述
        """
        emergency_description = ""
        # 1. 距离交叉路口停止线小于20米，进行提示
        next_lane = roadgraph.get_available_next_lane(self.current_lane.id, self.ego_vehicle.available_lanes)
        if isinstance(next_lane, JunctionLane) and next_lane.tlLogic == "actuated" and self.current_lane.spline_length - self.ego_vehicle.current_state.s < 20:
            emergency_description = self.des_json["import_description"]["traffic_notice"] + "\n"
        # 2. 需要换道，进行提示
        if isinstance(self.current_lane, NormalLane) and self.current_lane.id not in self.ego_vehicle.available_lanes:
            direction = None # 0表示左转，1表示右转
            index = 0 # 用于记录需要换道的lane相对于当前lane的位置
            # find left available lanes
            lane = self.current_lane
            while lane.left_lane() is not None:
                lane_id = lane.left_lane()
                index += 1
                if lane_id in self.ego_vehicle.available_lanes:
                    direction = 0
                    self.logging.info(
                        f"Ego vehicle choose to change Left lane")
                    break
                lane = roadgraph.get_lane_by_id(lane_id)
            if direction == None:
                # find right available lanes
                lane = self.current_lane
                index = 0
                while lane.right_lane() is not None:
                    lane_id = lane.right_lane()
                    index += 1
                    if lane_id in self.ego_vehicle.available_lanes:
                        direction = 1
                        self.logging.info(
                            f"Ego vehicle choose to change Right lane"
                        )
                        break
                    lane = roadgraph.get_lane_by_id(lane_id)
            if direction == None:
                # can not reach to available lanes
                self.logging.error(
                    f"Vehicle {self.id} cannot change to available lanes, "
                    f"current lane {self.lane_id}, available lanes {self.available_lanes}"
                )
            else:
                emergency_description += self.des_json["import_description"]["change_lane_notice"].format(direction = "right" if direction else "left", index = index, dis_stop_line = round(self.current_lane.spline_length - self.ego_vehicle.current_state.s, 3), dis_change_lane = 10) + "\n"
        return emergency_description

    def getLastDecisionDescription(self, current_decision_time: float, last_decision_time: float) -> str:
        """获取上一次决策的描述，包括上一次决策的时间，上一次决策的动作，上一次决策的结果

        Args:
            current_decision_time (float): 当前决策的时间
            last_decision_time (float): 上一次决策的时间

        Returns:
            str: 上一次决策的描述
        """
        if self.decision != None and current_decision_time - last_decision_time < 5:
            last_decision_description = self.des_json["basic_description"]["last_decision_description"]["basic"].format(delta_time = current_decision_time - last_decision_time, decision = self.decision)
            # TODO: 增加在换道时候的描述
            if self.decision == Behaviour.LCL or self.decision == Behaviour.LCR:
                print("current lane is ", self.current_lane.id)
                print("last lane is ", self.last_lane.id)
                if self.last_lane.id != self.current_lane.id:
                    last_decision_description += self.des_json["basic_description"]["last_decision_description"]["changed_lane"].format(direction = "right" if self.decision == Behaviour.LCR else "left")
                else:
                    last_decision_description += self.des_json["basic_description"]["last_decision_description"]["changing_lane"].format(direction = "right" if self.decision == Behaviour.LCR else "left")

            return last_decision_description
        else:
            return ""

    def availableActionsDescription(self) -> str:
        """获取可用的action的描述

        Returns:
            str: 可用action的描述
        """
        avaliableActionDescription = 'Your available actions are: \n'
        availableActions = self.get_available_actions()
        for action in availableActions:
            avaliableActionDescription += ACTIONS_DESCRIPTION[action] + ' Action_id: ' + str(
                action.value) + '\n'
        return avaliableActionDescription
    
    def get_available_actions(self) -> List[int]:
        """获取可用的action

        Returns:
            List[int]: 可用的action序号
        """
        if isinstance(self.current_lane, JunctionLane) or (isinstance(self.current_lane, NormalLane) and self.ego_vehicle.current_state.s > self.current_lane.course_spline.s[-1] - 10):
            return [Behaviour.IDLE, Behaviour.AC, Behaviour.DC]
        else:
            return [Behaviour.IDLE, Behaviour.LCL, Behaviour.LCR, Behaviour.AC, Behaviour.DC]

    def getSVRelativeState(self, sv: Vehicle) -> str:
        """获取SV相对于ego的位置状态，包括ahead, behind

        Args:
            sv (Vehicle): 附近车辆

        Returns:
            str: 位置信息描述
        """
        #TODO:目前不清楚获取的lane的s信息是怎么表述的，和车辆行驶方向是否有关
        relativePosition = sv.current_state.s - self.ego_vehicle.current_state.s
        if relativePosition >= 0:
            return 'ahead'
        else:
            return 'behind'

    def getSVInfo(self, prediction: Prediction, roadgraph: RoadGraph) -> str:
        """获取AoI内的车辆信息，需要区分在normal lane和junction lane上的描述

        Args:
            prediction (Prediction): 车辆在未来一段时间内的预测结果

        Returns:
            str: 对AoI内的车辆进行描述
        """
        # 如果在normal lane上，只需要对AoI同方向的车辆进行描述; 如果在junction lane上，需要对所有AoI在junction上的车辆进行描述
        # 对AOI区域内的车辆进行描述，只需要描述会发生碰撞的车辆+{如果在junction lane上，只需描述同个lane的车辆信息}+{如果在normal lane上，描述当前lane以及两边的车辆信息}
        svDescription = "There are other vehicles driving around you, and below is their basic information:\n"
        is_sv = False
        # 这里需要在交叉路口前面一定距离的时候就同时考虑junction lane上的同方向车辆
        if isinstance(self.current_lane, NormalLane):
            for sv in self.SV:
                sv_lane = roadgraph.get_lane_by_id(sv.lane_id)
                if isinstance(sv_lane, NormalLane) and sv_lane.affiliated_edge.id == self.current_lane.affiliated_edge.id:
                    self.SV.remove(sv)
                    same_edge_sv = self.describeSVNormalLane(sv)
                    if same_edge_sv != "":
                        is_sv = True
                        svDescription += same_edge_sv
        
        if isinstance(self.current_lane, JunctionLane):
        # 描述同一个junction lane上的车辆
            for sv in self.SV:
                if sv.lane_id == self.current_lane.id:
                    self.SV.remove(sv)
                    is_sv = True
                    svDescription += self.describeSVNormalLane(sv)

        # 描述下一个normal lane的车辆
            next_lane = roadgraph.get_available_next_lane(self.current_lane.id, self.ego_vehicle.available_lanes)
            if next_lane != None:
                for sv in self.SV:
                    if sv.lane_id == next_lane.id:
                        self.SV.remove(sv)
                        is_sv = True
                        svDescription += self.describeSVNormalLane(sv, next_lane = True)

        if isinstance(self.current_lane, NormalLane) and self.ego_vehicle.current_state.s > self.current_lane.course_spline.s[-1] - 10:
        # 描述下一个junction lane上的车辆
            next_lane = roadgraph.get_available_next_lane(self.current_lane.id, self.ego_vehicle.available_lanes)
            if next_lane != None:
                for sv in self.SV:
                    if sv.lane_id == next_lane.id:
                        self.SV.remove(sv)
                        is_sv = True
                        svDescription += self.describeSVNormalLane(sv, next_lane = True)

        # 描述在AOI区域内可能发生碰撞的车辆
        for sv in self.SV:
            is_sv = True
            collision_des = self.describeSVInAOI(sv, prediction.get(sv, None))
            if collision_des != "":
                svDescription += collision_des
                is_sv = True

        if not is_sv:
            svDescription = 'There are no other vehicles driving near you, so you can drive completely according to your own ideas.\n'
        return svDescription
    
    def describeSVNormalLane(self, vehicle: Vehicle, next_lane: bool = False) -> str:
        """当 ego 在 StraightLane 上时，车道信息是重要的，需要处理车道信息
        首先判断车辆和 ego 是否在同一条 lane 上行驶，如果是，那么只需要判断车辆和 ego 的相对位置
        如果不是，那么需要判断车辆和 ego 的相对位置，以及车辆和 ego 的 lane 的相对位置

        Args:
            vehicle (Vehicle): sv车辆

        Returns:
            str: 在normal lane上的车辆的描述
        """
        # TODO:考虑周围车辆可能的换道信息
        # TODO:考虑normal lane很短，前面立马就是junction lane的情况，不能只考虑normal lane上的车
        if next_lane:
            lane_relative_position = "same lane as you"
            relative_position = "ahead"
            distance = round(self.current_lane.spline_length - self.ego_vehicle.current_state.s + vehicle.current_state.s, 3)

        else:
            if vehicle.lane_id == self.current_lane.id:
                # 车辆和 ego 在同一条 lane 上行驶
                lane_relative_position = "same lane as you"
            elif vehicle.lane_id == self.current_lane.left_lane():
                # 车辆和 ego 在左侧 lane 上行驶
                lane_relative_position = "lane to your left"
            elif vehicle.lane_id == self.current_lane.right_lane():
                # 车辆和 ego 在右侧 lane 上行驶
                lane_relative_position = "lane to your right"
            else:
                return ''
        
            relative_position = self.getSVRelativeState(vehicle)

            distance = round(abs(vehicle.current_state.s - self.ego_vehicle.current_state.s), 3)

        sv_position = '('+ str(round(vehicle.current_state.x, 3)) + "," + str(round(vehicle.current_state.y, 3)) +')'

            
        sv_normal_des = self.des_json["basic_description"]["surrond_vehicle_on_normal_description"].format(\
                                                                    sv_id = vehicle.id, \
                                                                    lane_relative_position = lane_relative_position, \
                                                                    relative_position = relative_position, \
                                                                    sv_speed = round(vehicle.current_state.vel, 3), \
                                                                    sv_acceleration = round(vehicle.current_state.acc, 3), \
                                                                    sv_lane_position = round(vehicle.current_state.s, 3), \
                                                                    sv_position = sv_position,\
                                                                    distance = distance)

        return sv_normal_des + "\n"

    def describeSVInAOI(self, vehicle: Vehicle, prediction_state: List[State]) -> [str, bool]:
        """当进行交差路口的描述时，需要对信息进行相应转换
        首先判断未来轨迹是否会相交
        如果相交，则需要判断相交的时间和位置，计算出来给到llm

        Args:
            vehicle (Vehicle): 在交叉路口的车辆
            prediction_state (List[State]): 该车辆在未来一段时间内的预测轨迹

        Returns:
            str: 在交叉路口的车辆的描述
        """

        # 计算轨迹交点
        if prediction_state == None or self.ego_prediction == None:
            self.logging.info("the prediction state of vehicle {} is None".format(vehicle.id))
            return ""
        else:
            [ego_time, ego_s, sv_time, sv_s] = self.trajectory_overlap(vehicle, self.ego_prediction, prediction_state)
            if ego_s != None:
                sv_junction_des = self.des_json["basic_description"]["surrond_vehicle_on_junction_description"].format(sv_id = vehicle.id, \
                                                                    sv_speed = round(vehicle.current_state.vel, 3), \
                                                                    sv_acc = round(vehicle.current_state.acc), \
                                                                    ego_s = round(ego_s, 3), \
                                                                    ego_time = round(ego_time, 3), \
                                                                    sv_s = round(sv_s, 3), \
                                                                    sv_time = round(sv_time, 3))
            
            else:
                return ""
        
        return sv_junction_des + "\n"

    def trajectory_overlap(self, sv:Vehicle, ego_traj: List[State], sv_traj: List[State]):
        """判断两条轨迹是否有重叠

        Args:
            ego_traj (List[State]): 自车轨迹
            sv_traj (List[State]): SV轨迹

        Returns:
            ego_time: 自车到达冲突点的时间
            ego_s: 冲突点离自车的距离
            sv_time: SV到达冲突点的时间
            sv_s: 冲突点离SV的距离
        """
        # 计算两条轨迹上每个点之间的距离
        ego_time, ego_s, sv_time, sv_s = None, None, None, None

        ego_xy = np.array([[state.x, state.y] for state in ego_traj]) # 维度为 m x 2

        sv_xy = np.array([[state.x, state.y] for state in sv_traj]) # 维度为 n x 2

        m, _ = ego_xy.shape
        n, _ = sv_xy.shape
        arr1_power = np.power(ego_xy, 2)
        arr1_power_sum = arr1_power[:, 0] + arr1_power[:, 1]
        arr1_power_sum = np.tile(arr1_power_sum, (n, 1))
        arr1_power_sum = arr1_power_sum.T
        arr2_power = np.power(sv_xy, 2)
        arr2_power_sum = arr2_power[:, 0] + arr2_power[:, 1]
        arr2_power_sum = np.tile(arr2_power_sum, (m, 1))
        dis = arr1_power_sum + arr2_power_sum - (2 * np.dot(ego_xy, sv_xy.T))
        dis = np.sqrt(dis) # 输出数组的维度为 m x n
        dis_statis = np.argwhere(dis < 0.5)

        if dis_statis.size != 0:
            ego_min_index, sv_min_index = dis_statis[0]

            # 如果在ego的第一个点和sv的第一个点就发生碰撞，说明是位于ego后面的车辆，不需要考虑
            if ego_min_index == 0:
                pass
            else:
                ego_s = ego_traj[ego_min_index].s - self.ego_vehicle.current_state.s
                sv_s = sv_traj[sv_min_index].s - sv.current_state.s
                ego_time = ego_traj[ego_min_index].t if ego_min_index != 0 else 0
                sv_time = sv_traj[sv_min_index].t if sv_min_index != 0 else 0

            # self.logging.info("ego collide with sv at {}".format(ego_min_index, sv_min_index, ego_time, ego_s, sv_time, sv_s))
        
        # plt.plot([state.x for state in ego_traj], [state.y for state in ego_traj], 'r*')
        # plt.plot([state.x for state in sv_traj], [state.y for state in sv_traj], 'b*')
        # plt.show()
        
        return [ego_time, ego_s, sv_time, sv_s]

    def describe(self, 
                 observation:Observation, 
                 roadgraph: RoadGraph, 
                 prediction: Prediction, 
                 current_decision_time, 
                 last_decision_time) -> List[str]:
        
        # step1: 获取ego和AoI内的车辆信息
        self.SV = []
        for vehicle in observation.vehicles:
            if vehicle.vtype == VehicleType.EGO:
                self.ego_vehicle = vehicle
            if vehicle.vtype == VehicleType.IN_AOI:
                self.SV.append(vehicle)

        # 获取上一次的lane
        self.last_lane = self.current_lane

        self.current_lane = roadgraph.get_lane_by_id(self.ego_vehicle.lane_id)
        self.ego_prediction = prediction.results.get(self.ego_vehicle, None)

        egoDecription = self.getEgoInfo()
        SVDescription = self.getSVInfo(prediction.results, roadgraph)

        # step2: 获取lane信息
        laneDescription = self.getLaneInfo(roadgraph)

        # step3: 决策连续性描述
        lastDecisionDescription = self.getLastDecisionDescription(current_decision_time, last_decision_time)

        # step4: 获取紧急情况描述
        emergencyDescription = self.getEmergencyInfo(roadgraph)

        # step5: 获取可用的action
        availableActionsDescription = self.availableActionsDescription()

        scenario_description = laneDescription + egoDecription + SVDescription + lastDecisionDescription + emergencyDescription

        self.logging.info("scenario_description is {}".format(scenario_description))
        self.logging.info("driving_intensions is {}".format(self.des_json["intension"]["normal"] + "\n"))
        self.logging.info("available_actions is {}".format(availableActionsDescription))

        return [scenario_description, availableActionsDescription, self.des_json["intension"]["normal"] + "\n"]

    # TODO: 数据库操作待增加
    # def promptsCommit(
    #     self, decisionFrame: int, vectorID: str, done: bool,
    #     description: str, fewshots: str, thoughtsAndAction: str
    # ):
    #     self.dbBridge.insertPrompts(
    #         decisionFrame, vectorID, done, description,
    #         fewshots, thoughtsAndAction
    #     )
