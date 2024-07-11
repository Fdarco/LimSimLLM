import time

from trafficManager.common.observation import Observation
from trafficManager.common.vehicle import Behaviour
from vehicle import Vehicle
from trafficManager.planner.abstract_planner import AbstractEgoPlanner
from trafficManager.predictor.abstract_predictor import Prediction

import logger, logging
from Network_Structure import JunctionLane, NormalLane, AbstractLane
from Roadgraph import RoadGraph
from utils.trajectory import State, Trajectory
from simInfo.CustomExceptions import NoPathFoundError

import numpy as np
import math
import os

from trafficManager.planner.frenet_optimal_planner import frenet_optimal_planner
from trafficManager.common import cost

import yaml


def load_config(config_file_path: str) -> dict:
    """Loads the configuration file from the given path.
    
    Args:
        config_file_path (str): Path to the configuration file.
    
    Returns:
        dict: The configuration dictionary.
    """
    try:
        with open(config_file_path, "r", encoding="utf-8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    except FileNotFoundError:
        with open(os.path.join('..',config_file_path),"r", encoding="utf-8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    return config

decision_logger = logger.setup_app_level_logger(logger_name="ego_planner", file_name="llm_decision.log")
logging = logging.getLogger("ego_planner").getChild(__name__)

class LLMEgoPlanner:
    def __init__(self):
        self.config = load_config("trafficManager/config.yaml")

    def simple_state_machine(self, ego_veh: Vehicle, roadgraph: RoadGraph, current_lane: AbstractLane) -> Behaviour:
        """Simple state machine for ego vehicle

        Args:
            ego_veh (Vehicle): ego vehicle
            roadgraph (RoadGraph): road graph

        Returns:
            Behaviour: behaviour
        """
        ego_veh.behaviour = Behaviour.IDLE

        if not current_lane.id in ego_veh.next_available_lanes:
            # 说明应该换道了
            while current_lane.left_lane:
                left_lane = roadgraph.get_lane_by_id(current_lane.left_lane)
                if left_lane.id in ego_veh.next_available_lanes:
                    ego_veh.behaviour = Behaviour.LCL
                    break
                current_lane = left_lane
            else:
                while current_lane.right_lane:
                    right_lane = roadgraph.get_lane_by_id(current_lane.right_lane)
                    if right_lane.id in ego_veh.next_available_lanes:
                        ego_veh.behaviour = Behaviour.LCR
                        break
                    current_lane = right_lane

            # 说明当前section上已经没有可行驶的lane了
            if ego_veh.behaviour == Behaviour.IDLE:
                # TODO:raise NoPathFoundError("There is no available lane for the vehicle to drive!")
                pass
        return ego_veh.behaviour
    
    def plan(self,
             ego_veh: Vehicle,
             roadgraph: RoadGraph,
             config,
             T ) -> Trajectory:
        logging.info("============= planning at {} s =============".format(T))
        """planning for ego vehicle

        Raises:
            NoPathFoundError: _description_

        Returns:
            _type_: _description_
        """
        config = self.config
        start = time.time()
        current_lane = roadgraph.get_lane_by_id(ego_veh.lane_id)
        obs_list = []

        # vehicle_id = ego_veh.id
        # Process static obstacle
        # for obs in observation.obstacles:
        #     obs_list.append(obs)

        # Process dynamic_obstacles
        # for predict_veh, prediction in prediction.results.items():
        #     if predict_veh.id == vehicle_id:
        #         continue

        #     shape = Rectangle(predict_veh.length, predict_veh.width)
        #     current_state = State(x=prediction[0].x,
        #                           y=prediction[0].y,
        #                           s=prediction[0].s,
        #                           d=prediction[0].d,
        #                           yaw=prediction[0].yaw,
        #                           vel=prediction[0].vel)
        #     dynamic_obs = DynamicObstacle(obstacle_id=predict_veh.id,
        #                                   shape=shape,
        #                                   obstacle_type=ObsType.CAR,
        #                                   current_state=current_state,
        #                                   lane_id=predict_veh.lane_id)
        #     for i in range(1, len(prediction)):
        #         state = State(x=prediction[i].x,
        #                       y=prediction[i].y,
        #                       s=prediction[i].s,
        #                       d=prediction[i].d,
        #                       yaw=prediction[i].yaw,
        #                       vel=prediction[i].vel)
        #         dynamic_obs.future_trajectory.states.append(state)

        #     obs_list.append(dynamic_obs)

        # state machine
        ego_veh.behaviour = self.simple_state_machine(ego_veh, roadgraph, current_lane)

        # 对于很短的lane，可能会规划不出来
        next_lane = ego_veh.get_available_next_lane(
            roadgraph, current_lane.id)
        
        # 针对lane length很短的section，可以允许车辆沿着当前lane继续行驶
        if next_lane == None:
            if current_lane.next_lane:
                next_lane_id = roadgraph.WP2Lane[current_lane.next_lane]
                next_lane = roadgraph.get_lane_by_id(next_lane_id)

        # 防止lane过短，导致规划不出来
        if next_lane != None:
            lanes = [current_lane, next_lane]
            if current_lane.length - ego_veh.state.s + next_lane.length < 20:#why 20?
                # 找下一个lane
                next_lane = ego_veh.get_available_next_lane(
                    roadgraph, next_lane.id)
                if next_lane != None:
                    lanes.append(next_lane)
        else:
            lanes = [current_lane]

        if ego_veh.behaviour == Behaviour.IDLE:
            path = self.acdc_trajectory_generator(ego_veh, lanes, config, ego_veh.behaviour)
        elif ego_veh.behaviour == Behaviour.AC:
            # Accelerate
            path = self.acdc_trajectory_generator(ego_veh, lanes, config, ego_veh.behaviour)
        elif ego_veh.behaviour == Behaviour.DC:
            # Decelerate
            path = self.acdc_trajectory_generator(ego_veh, lanes, config, ego_veh.behaviour)
        elif ego_veh.behaviour == Behaviour.LCL:
            # Turn Left, make sure the left lane has enough length
            left_lane = roadgraph.get_lane_by_id(current_lane.left_lane)
            if left_lane.length - ego_veh.state.s < 10:
                # 如果长度太短，拼接left_lane和下一个lane
                next_lane_id=left_lane.next_lane
                if next_lane_id:
                    next_lane = roadgraph.get_lane_by_id(roadgraph.WP2Lane[next_lane_id])
                    left_lane.wp_list.extend(next_lane.wp_list)
                    left_lane.get_spline2D()
            path = self.lanechange_trajectory_generator(
                ego_veh,
                left_lane,
                obs_list,
                config
            )
        elif ego_veh.behaviour == Behaviour.LCR:
            # Turn Right
            right_lane = roadgraph.get_lane_by_id(current_lane.right_lane)
            if right_lane.length - ego_veh.state.s < 10:
                # 拼接left_lane和下一个lane
                next_lane = roadgraph.get_lane_by_id(roadgraph.WP2Lane[right_lane.next_lane])
                if next_lane:
                    right_lane.wp_list.extend(next_lane.wp_list)
                    right_lane.get_spline2D()
            path = self.lanechange_trajectory_generator(
                ego_veh,
                right_lane,
                obs_list,
                config
            )
        else:
            logging.error(
                "Vehicle {} has unknown behaviour {}".format(
                    ego_veh.id, ego_veh.behaviour)
            )
        logging.debug(
            "Vehicle {} Total planning time: {}".format(
                ego_veh.id, time.time() - start)
        )

        return path

    def acdc_trajectory_generator(self, 
        vehicle: Vehicle, 
        lanes: AbstractLane, 
        config, 
        behaviour:Behaviour
    ) -> Trajectory:
        
        course_t = config["MIN_T"]  # Sample course time
        dt = config["DT"]  # time tick
        current_state = vehicle.state
        
        if behaviour == Behaviour.AC:
            if current_state.acc < 0:
                target_acc = config["NORMAL_ACC_DEFAULT"]
            else:
                target_acc = current_state.acc + config["NORMAL_ACC_DEFAULT"]
            if target_acc > config["ACC_MAX"]:
                target_acc = config["ACC_MAX"]

        elif behaviour == Behaviour.DC:
            if current_state.acc > 0:
                target_acc = -config["NORMAL_DCC_DEFAULT"]
            else:
                target_acc = current_state.acc -config["NORMAL_DCC_DEFAULT"]
            if target_acc < config["ACC_MIN"]:
                target_acc = config["ACC_MIN"]
            # When the speed decreases to 0, it does not continue to decelerate and stops at the same place
            if current_state.vel + target_acc*course_t < 0:
                course_t = (0.1 - current_state.vel)/target_acc

        elif behaviour == Behaviour.IDLE:
            target_acc = 0

        target_v = current_state.vel + target_acc*course_t
        if behaviour == Behaviour.IDLE:
            target_v = 5.0
            
        target_s = current_state.s + (current_state.vel + target_v)* course_t * 0.5
        if target_s - current_state.s < vehicle.length:
            target_state = State(s=target_s, s_d=target_v, d=current_state.d)
        else:
            target_state = State(s=target_s, s_d=target_v, d=0)

        # judge stopping the car
        if (round(current_state.vel, 1) <= 0.1 and
            (target_s - current_state.s) <= vehicle.length):  # already stopped, keep it
            logging.debug(f"Vehicle {vehicle.id} Already stopped")
            path = Trajectory()
            for t in np.arange(0, config["MIN_T"], dt):
                path.states.append(
                    State(t=t, s=current_state.s, d=current_state.d, yaw=None))
            path.frenet_to_cartesian(lanes, vehicle.state)
            path.cost = (
                cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
                dt + cost.guidance(path, config["weights"]) * dt +
                cost.acc(path, config["weights"]) * dt +
                cost.jerk(path, config["weights"]) * dt)
        else:
            path = frenet_optimal_planner.calc_spec_path(current_state,
                                                        target_state, course_t, dt
                                                        )
            
            if course_t != config["MIN_T"]:
                logging.debug(f"course_t is {course_t}, dt is {dt}, path length is {len(path.states)}, yaw is {path.states[-1].yaw}, current yaw is {current_state.yaw}")
                for t in np.arange(path.states[-1].t + dt, config["MIN_T"], dt):
                    path.states.append(
                        State(t=t, s=target_state.s, d=target_state.d, yaw=None))
            path.frenet_to_cartesian(lanes, current_state)
            path.cost = (
                cost.smoothness(path, lanes[0].course_spline, config["weights"]) *
                dt + cost.guidance(path, config["weights"]) * dt +
                cost.acc(path, config["weights"]) * dt +
                cost.jerk(path, config["weights"]) * dt)
        return path


    def lanechange_trajectory_generator(self,
        vehicle: Vehicle,
        target_lane: AbstractLane,
        obs_list,
        config,
    ) -> Trajectory:
        
        state_in_target_lane = vehicle.get_state_in_lane(target_lane)
        target_vel = vehicle.state.vel
        dt = config["DT"]
        s_sample = config["S_SAMPLE"]
        n_s_sample = config["N_S_SAMPLE"]

        sample_t = [config["MIN_T"] / 1.5]  # Sample course time
        vel_min = max(state_in_target_lane.vel - 2.0, 0)
        vel_max = min(target_vel + s_sample * n_s_sample * 1.01, 13.89)
        vel_max = max(vel_max, 5.0)
        sample_s = np.empty(0)
        for t in sample_t:
            sample_s = np.append(
                sample_s,
                np.arange(
                    state_in_target_lane.s + t * vel_min,
                    state_in_target_lane.s + t * vel_max,
                    s_sample,
                ),
            )

        # Step 2: Calculate Paths
        best_path = None
        best_cost = math.inf
        for t in sample_t:
            for s in sample_s:
                s_d = vehicle.state.vel
                target_state = State(t=t, s=s, d=0, s_d=s_d)
                path = frenet_optimal_planner.calc_spec_path(
                    state_in_target_lane, target_state, target_state.t, dt)
                if not path.states:
                    continue
                path.frenet_to_cartesian(target_lane, vehicle.state)
                path.cost = (
                    cost.smoothness(path, target_lane.course_spline,
                                    config["weights"]) * dt +
                    cost.vel_diff(path, target_vel, config["weights"]) * dt +
                    cost.guidance(path, config["weights"]) * dt +
                    cost.acc(path, config["weights"]) * dt +
                    cost.jerk(path, config["weights"]) * dt +
                    cost.obs(vehicle, path, obs_list, config) +
                    cost.changelane(config["weights"]))
                if not path.is_nonholonomic():
                    continue
                if path.cost < best_cost:
                    best_cost = path.cost
                    best_path = path
        if best_path is not None:
            logging.debug(f"Vehicle {vehicle.id} found a lane change path with cost: {best_cost}")
            return best_path
        else:
            raise NoPathFoundError("Can't generate the trajectory!")
