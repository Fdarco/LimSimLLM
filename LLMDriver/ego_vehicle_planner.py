import time

from trafficManager.common.observation import Observation
from trafficManager.common.vehicle import Behaviour, Vehicle
from trafficManager.decision_maker.abstract_decision_maker import EgoDecision, MultiDecision
from trafficManager.planner.abstract_planner import AbstractEgoPlanner
from trafficManager.predictor.abstract_predictor import Prediction

import logger
import trafficManager.planner.trajectory_generator as traj_generator
from utils.obstacles import DynamicObstacle, ObsType, Rectangle
from utils.roadgraph import JunctionLane, NormalLane, RoadGraph, AbstractLane
from utils.trajectory import State, Trajectory

import numpy as np
import math

from trafficManager.planner.frenet_optimal_planner import frenet_optimal_planner
from trafficManager.common import cost
logging = logger.get_logger(__name__)

class LLMEgoPlanner(AbstractEgoPlanner):
    def plan(self,
             ego_veh: Vehicle,
             observation: Observation,
             roadgraph: RoadGraph,
             prediction: Prediction,
             T,
             config,
             ego_decision: MultiDecision = None) -> Trajectory:
        
        # self.update_state(ego_veh, roadgraph)

        vehicle_id = ego_veh.id
        start = time.time()
        current_lane = roadgraph.get_lane_by_id(ego_veh.lane_id)

        obs_list = []
        # Process static obstacle
        for obs in observation.obstacles:
            obs_list.append(obs)

        # Process dynamic_obstacles
        for predict_veh, prediction in prediction.results.items():
            if predict_veh.id == vehicle_id:
                continue

            shape = Rectangle(predict_veh.length, predict_veh.width)
            current_state = State(x=prediction[0].x,
                                  y=prediction[0].y,
                                  s=prediction[0].s,
                                  d=prediction[0].d,
                                  yaw=prediction[0].yaw,
                                  vel=prediction[0].vel)
            dynamic_obs = DynamicObstacle(obstacle_id=predict_veh.id,
                                          shape=shape,
                                          obstacle_type=ObsType.CAR,
                                          current_state=current_state,
                                          lane_id=predict_veh.lane_id)
            for i in range(1, len(prediction)):
                state = State(x=prediction[i].x,
                              y=prediction[i].y,
                              s=prediction[i].s,
                              d=prediction[i].d,
                              yaw=prediction[i].yaw,
                              vel=prediction[i].vel)
                dynamic_obs.future_trajectory.states.append(state)

            obs_list.append(dynamic_obs)

        """
        Predict for current vehicle
        """
        next_lane = roadgraph.get_available_next_lane(
            current_lane.id, ego_veh.available_lanes)
        lanes = [current_lane, next_lane] if next_lane != None else [
            current_lane]

        if ego_veh.behaviour == Behaviour.IDLE:
            path = self.acdc_trajectory_generator(ego_veh, lanes, config, ego_veh.behaviour)
        elif ego_veh.behaviour == Behaviour.AC:
            # Accelerate
            path = self.acdc_trajectory_generator(ego_veh, lanes, config, ego_veh.behaviour)
        elif ego_veh.behaviour == Behaviour.DC:
            # Decelerate
            path = self.acdc_trajectory_generator(ego_veh, lanes, config, ego_veh.behaviour)
        elif ego_veh.behaviour == Behaviour.LCL:
            # Turn Left
            left_lane = roadgraph.get_lane_by_id(current_lane.left_lane())
            path = self.lanechange_trajectory_generator(
                ego_veh,
                left_lane,
                obs_list,
                config,
                T,
            )
        elif ego_veh.behaviour == Behaviour.LCR:
            # Turn Right
            right_lane = roadgraph.get_lane_by_id(current_lane.right_lane())
            path = self.lanechange_trajectory_generator(
                ego_veh,
                right_lane,
                obs_list,
                config,
                T,
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
        current_state = vehicle.current_state
        
        # TODO: 加减速需要替换为对加速度的修改，而不是给定加速度进行计算
        if behaviour == Behaviour.AC:
            target_acc = config["ACC_DEFAULT"]
            target_s = current_state.s + current_state.vel * course_t + 0.5* target_acc * (course_t**2)
            target_state = State(s=target_s, s_d=current_state.s_d + target_acc*course_t, d=0)
        elif behaviour == Behaviour.DC:
            target_acc = -config["ACC_DEFAULT"]
            target_s = current_state.s + current_state.vel * course_t + 0.5* target_acc * (course_t**2)
            target_state = State(s=target_s, s_d=current_state.s_d + target_acc*course_t, d=0)
        elif behaviour == Behaviour.IDLE:
            target_s = current_state.s + current_state.vel * course_t
            target_state = State(s=target_s, s_d=current_state.s_d, d=0)

        path = frenet_optimal_planner.calc_spec_path(current_state,
                                                     target_state, course_t, dt
                                                     )
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
        T,
    ) -> Trajectory:
        
        state_in_target_lane = vehicle.get_state_in_lane(target_lane)
        target_vel = vehicle.target_speed
        dt = config["DT"]
        d_t_sample = config["D_T_S"] / 3.6
        n_s_d_sample = config["N_D_S_SAMPLE"]
        s_sample = config["S_SAMPLE"]
        n_s_sample = config["N_S_SAMPLE"]

        sample_t = [config["MIN_T"] / 1.5]  # Sample course time
        sample_vel = np.linspace(
            max(1e-9, state_in_target_lane.vel - d_t_sample * n_s_d_sample),
            min(state_in_target_lane.vel + d_t_sample * n_s_d_sample,
                target_lane.speed_limit), 5)
        vel = min(state_in_target_lane.vel, target_vel)
        sample_s = np.empty(0)
        for t in sample_t:
            sample_s = np.append(
                sample_s,
                np.arange(
                    state_in_target_lane.s + t * (max(5.0, vel)),
                    state_in_target_lane.s + t *
                    (target_vel + s_sample * n_s_sample * 1.01),
                    s_sample,
                ),
            )

        # Step 2: Calculate Paths
        best_path = None
        best_cost = math.inf
        for t in sample_t:
            for s in sample_s:
                for s_d in sample_vel:
                    target_state = State(t=t, s=s, d=0, s_d=s_d)
                    path = frenet_optimal_planner.calc_spec_path(
                        state_in_target_lane, target_state, target_state.t, dt)
                    if not path.states:
                        continue
                    path.frenet_to_cartesian(target_lane, vehicle.current_state)
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
        # 不再对路径的cost进行判断
        # TODO: 只将其作为参考留下来，后续可以通过cost来对LLM进行训练
        if best_path is not None:
            logging.debug(f"Vehicle {vehicle.id} found a lane change path with cost: {best_cost}")
            return path

        else:
            # TODO:当轨迹无法生成时，结束仿真
            return False
    
    def update_state(self, vehicle: Vehicle, roadgraph: RoadGraph) -> None:
        """Update the behaviour of a vehicle.

        Args:
            roadgraph (RoadGraph): The roadgraph containing the lanes the vehicle is traveling on.
        """
        current_lane = roadgraph.get_lane_by_id(vehicle.lane_id)

        if vehicle.current_state.s > current_lane.course_spline.s[-1] - 0.2:
            if isinstance(current_lane, NormalLane):
                next_lane = roadgraph.get_available_next_lane(
                    current_lane.id, vehicle.available_lanes)
                vehicle.lane_id = next_lane.id
                vehicle.current_state = vehicle.get_state_in_lane(next_lane)
                current_lane = next_lane
            elif isinstance(current_lane, JunctionLane):
                next_lane_id = current_lane.next_lane_id
                next_lane = roadgraph.get_lane_by_id(next_lane_id)
                vehicle.lane_id = next_lane.id
                vehicle.current_state = vehicle.get_state_in_lane(next_lane)
                current_lane = next_lane