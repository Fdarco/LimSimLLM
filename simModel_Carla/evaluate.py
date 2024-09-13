from simInfo.Evaluation import Hyper_Parameter,Score_List,Decision_Score
from Replay import ReplayModel
from simModel.CarFactory import Vehicle, egoCar
from utils.trajectory import Trajectory, State,Rectangle,RecCollide
from Roadgraph import RoadGraph
from Network_Structure import JunctionLane

import sqlite3
import logger,logging
from typing import List
import numpy as np

class Score:
    def __init__(self) -> None:
        """define each score item
        """
        pass
    def score(self)->float:
        """Calculate the score for each frame.

        Returns:
            float: The score for each frame decision.
        """
        raise NotImplementedError
    
class Evaluator:
    def __init__(self,database:str,timeStep:float) -> None:

        self.final_score: float = 0.0 # based on the whole decision in one route
        self.current_time = timeStep
        # odometer
        self.driving_mile = 0.0
        self.complete_percentage: float = 0.0
        
        
        self.logger = logger.setup_app_level_logger(logger_name="Evaluation", file_name="closeloopEvaluation_result.log")
        self.logging = logging.getLogger("Evaluation").getChild(__name__)
        self.createTable(database)
        
        self.deltaT=1#0.1
    
    @property
    def score(self):
        raise NotImplementedError
    def createTable(self,database) ->None:
        """create each score item table head in db 

        Args:
            database (_type_): db file path

        """
        raise NotImplementedError  
    def Evalutate(self,model:ReplayModel)->None:
        raise NotImplementedError
    def SaveResultinDB(self,model:ReplayModel)->None:
        conn = sqlite3.connect(model.dataBase)
        cur = conn.cursor()
        # add result data
        cur.execute(
            """UPDATE resultINFO SET total_score = {},complete_percentage={}, where egoID = {};""".format
            (
                self.final_score, self.complete_percentage,model.egoID
            )
        )

        conn.commit()
        conn.close()    
    def SaveDatainDB(self, model: ReplayModel, score:Score) -> None:
       raise NotImplementedError

class Decision_Evaluator(Evaluator):
    def __init__(self, database: str, timeStep: float) -> None:
        super().__init__(database, timeStep)
        self.decision_score = Score_List() 
        self.hyper_parameter = Hyper_Parameter()
        self.ttc_score = []
        self.red_junctionlane_record = []
        
        
    def createTable(self, database) -> None:
        conn = sqlite3.connect(database)
        cur = conn.cursor()
        cur.execute("DROP TABLE IF EXISTS evaluationINFO")
        conn.commit()
        cur.execute(
            """CREATE TABLE IF NOT EXISTS evaluationINFO(
                frame REAL PRIMARY KEY,
                traffic_light_score REAL,
                comfort_score REAL,
                efficiency_score REAL,
                speed_limit_score REAL,
                collision_score REAL,
                decision_score REAL,
                caution TEXT
            );"""
        )
        conn.commit()
        conn.close()
        
    def SaveDatainDB(self, model: ReplayModel, decision_score: Decision_Score) -> None:
        conn = sqlite3.connect(model.dataBase)
        cur = conn.cursor()
        # add evaluation data
        cur.execute(
            """INSERT INTO evaluationINFO (
                frame, traffic_light_score, comfort_score, efficiency_score, speed_limit_score, collision_score, decision_score, caution
                ) VALUES (?,?,?,?,?,?,?,?);""",
            (
                model.timeStep, decision_score.red_light, decision_score.comfort, decision_score.efficiency, decision_score.speed_limit, decision_score.safety, decision_score.score(self.hyper_parameter), self.current_reasoning
            )
        )
        conn.commit()
        conn.close()
    @property
    def score(self):
        return self.decision_score
    
    def Evaluate(self, model: ReplayModel) -> None:
        """Executes this function once per frame. It calculates the decision score and saves the data in the database.

        Args:
            model (ReplayModel): class containing current frame information
        """
        self.current_reasoning = ""
        
        # 1. calculate ttc: calculate the ego states and other car states in future 5s, take it as ttc(s)
        self.ttc_score.append(self.calculate_ttc(model) / self.hyper_parameter.TTC_THRESHOLD)
        
        # 2. calculate decision score each 10 frames
        if model.timeStep - self.current_time > 150 and model.timeStep % 10 == 0:
            current_decision_score = Decision_Score() # each decision's score
            self.Current_Decision_Score(model, current_decision_score)
            self.decision_score.append(current_decision_score)
            self.SaveDatainDB(model, current_decision_score)
            # update car mile
            self.CalculateDrivingMile(model)
        
        # 3. if the route is end, calculate the final score
        if model.tpEnd:
            if not self.getResult(model):
                self.decision_score.fail_result()
                self.logger.error("the result is failed")
                self.cal_route_length(model)
                self.CalculateDrivingMile(model)
                self.driving_mile += model.sr.ego.lanePos
                print(self.driving_mile, " ", self.route_length)
            else:
                self.route_length = self.driving_mile
                self.logging.info("the result is success!")

            self.SaveResultinDB(model)
            
            self.logger.info("your final score is {}".format(round(self.final_s, 3)))
            self.logger.info("your driving mile is {} m, the route length is {} m, the complete percentage is {}%".format(round(self.driving_mile, 3), round(self.route_length, 3), round(self.complete_p*100, 3)))
            self.logger.info("your driving score is {}".format(round(self.decision_score.eval_score(self.hyper_parameter), 3)))
            self.logger.info("your driving time is {} s".format((model.timeStep - self.current_time)/10))
            
        return

    def getResult(self, model: ReplayModel) -> bool:
        conn = sqlite3.connect(model.dataBase)
        cur = conn.cursor()
        cur.execute("SELECT result FROM resultINFO WHERE egoID = ?;", (model.sr.ego.id,))
        result = cur.fetchone()[0]
        conn.close()
        return result
    
    def calculate_ttc(self, model: ReplayModel) -> float:
        """calculate the ttc score, predict the future 5s trajectory, if the trajectory will collide, the time is ttc

        Args:
            model (ReplayModel)

        Returns:
            float: minimum ttc time
        """
        ttc_list = []

        # get ego prediction trajectory
        roadgraph= model.roadgraph
        ego_availablelanes = model.ego.next_available_lanes
        ego_trajectory = getSVTrajectory(model.sr.ego, roadgraph, ego_availablelanes, self.hyper_parameter.TTC_THRESHOLD)

        # get other vehicle prediction trajectory
        for _, value in model.vehINAoI.items():
            if value.id == model.egoID:
                continue
            veh_availablelanes = value.next_available_lanes
            vehicle_trajectory = getSVTrajectory(value, roadgraph, veh_availablelanes, self.hyper_parameter.TTC_THRESHOLD)

            # calculate ttc, if the (x, y) will collide, the time is ttc
            for index in range(0, min(len(ego_trajectory), len(vehicle_trajectory))):
                recA = Rectangle([ego_trajectory[index].x, ego_trajectory[index].y],
                                model.sr.ego.length, model.sr.ego.width, model.sr.ego.yaw)
                recB = Rectangle([vehicle_trajectory[index].x, vehicle_trajectory[index].y],
                                value.length, value.width, value.yaw)
                rc = RecCollide(recA, recB)
                if rc.isCollide():
                    ttc_list.append(ego_trajectory[index].t)
                    break
        return min(ttc_list) if len(ttc_list) > 0 else self.hyper_parameter.TTC_THRESHOLD
    
    def Current_Decision_Score(self, model: ReplayModel, decision_score: Decision_Score) -> Decision_Score:
        """Calculate the current decision score, include comfort, efficiency, speed limit, safety, red light

        Args:
            model (ReplayModel): class containing current frame information
            decision_score (Decision_Score): current decision score

        Returns:
            Decision_Score: current decision score
        """
        # 1. comfort -- evaluate comfortable of the driving
        decision_score.comfort = self.get_comfort_score(model.sr.ego, decision_score)
        
        # 2. pass red light check
        if self.PassRedLight(model):
            decision_score.red_light = 0.7
        else:
            decision_score.red_light = 1.0

        # 3. lower speed than other car's eval speed in the same edge in last 10 frame        
        lane_id = model.ego.laneID
        # if lane_id in model.roadgraph.Junction_Dict:
        #     speed_limit = model.rb.getJunctionLane(lane_id).speed_limit
        # else:
        #     speed_limit = model.rb.getLane(lane_id).speed_limit
        speed_limit=13.89#cant find it
        ego_history_speed = list(model.ego.speedQ)[-10::]
        
        ## 3.1 judge if wait for red light
        if self.judge_wait_traffic_light(model):
            decision_score.efficiency = 1.0
        ## 3.2 calculate the eval speed
        else:
            ego_history_speed = list(model.ego.speedQ)[-10::]
            ego_eval_speed = sum(ego_history_speed) /10
            decision_score.efficiency = 0.0
            vehicle_num = 0
            all_vehicle_eval_speed = 0.0
            # get next lane id
            try:
                ego_availablelanes = model.sr.ego.next_available_lanes
                next_lane = model.roadgraph.get_available_next_lane(model.sr.ego.laneID, ego_availablelanes)
            except Exception as e:
                next_lane = None
            for _, value in model.sr.vehINAoI.items():
                if value.id == model.sr.ego.id:
                    continue
                if value.laneID.split("-")[0] == model.sr.ego.laneID.split("-")[0] or (next_lane != None and value.laneID == next_lane.id):
                    vehicle_history_speed = list(value.speedQ)[-10::]
                    eval_speed = sum(vehicle_history_speed)/len(vehicle_history_speed)
                    all_vehicle_eval_speed += eval_speed
                    vehicle_num += 1
            # if there is no car in same edge, need to compare with speed limit
            if vehicle_num > 0 and all_vehicle_eval_speed > 0.1:
                compare_speed = min(all_vehicle_eval_speed / vehicle_num, speed_limit)
                decision_score.efficiency = ego_eval_speed / compare_speed
            elif vehicle_num == 0:
                decision_score.efficiency = ego_eval_speed / speed_limit
            else:
                decision_score.efficiency = 1.0
            decision_score.efficiency = min(1, decision_score.efficiency)
            if decision_score.efficiency < 0.6:
                self.current_reasoning += "your speed is so low that you have a low efficiency\n"

        # 4. exceed speed limit
        decision_score.speed_limit = 0.0
        ego_speed = np.array(ego_history_speed)
        ego_speed = np.where(ego_speed > speed_limit, ego_speed - speed_limit, 0)  
        if np.count_nonzero(ego_speed) > 0:
            decision_score.speed_limit = 0.9
            self.current_reasoning += "you exceed the speed limit\n"
        else:
            decision_score.speed_limit = 1.0

        # 5. ttc: calculate the ego states and other car states in future 5s, take it as ttc(s)
        decision_score.safety = min(self.ttc_score)
        if decision_score.safety <= 0.6:
            self.current_reasoning += "you have high risk of collision with other vehicles\n"
        self.ttc_score = []
        return decision_score

    def get_comfort_score(self, ego_vehicle: egoCar, decision_score: Decision_Score) -> float:
        """calculate the comfort score
        score = (acc_score + jerk_score + lateral_acc_score + lateral_jerk_score) / 4
        acc_score = k * (acc_eval - [acc_ref]) + b
        which   k = (b2 - b1) / (acc_ref.normal - acc_ref.cautious)
                b1 = 1.0
                b2 = 0.6

        Args:
            ego_vehicle (egoCar)
            decision_score (Decision_Score)

        Returns:
            float: comfort score
        """
        ego_history_acc = list(ego_vehicle.accelQ)[-11::]

        # 1. calculate longitudinal acc score
        ego_longitudinal_acc = sum(ego_history_acc[1::])/10
        decision_score.longitudinal_acc = self.hyper_parameter.calculate_acc_score(ego_longitudinal_acc, self.hyper_parameter.longitudinal_acc if ego_longitudinal_acc > 0 else self.hyper_parameter.longitudinal_dec)

        # 2. calculate longitudinal jerk score
        ego_longitudinal_jerk = np.mean((np.array(ego_history_acc[1::]) - np.array(ego_history_acc[0:-1]))/self.deltaT)
        decision_score.longitudinal_jerk = self.hyper_parameter.calculate_acc_score(ego_longitudinal_jerk, self.hyper_parameter.positive_jerk if ego_longitudinal_jerk > 0 else self.hyper_parameter.negative_jerk)
        
        # 3. calculate curvature k
        # ref from https://zhuanlan.zhihu.com/p/619658901
        ego_speed = list(ego_vehicle.speedQ)[-11::]
        ego_x = list(ego_vehicle.xQ)[-13::]
        ego_y = list(ego_vehicle.yQ)[-13::]
        ego_xdot = (np.array(ego_x[1::]) - np.array(ego_x[0:-1])) / self.deltaT
        ego_ydot = (np.array(ego_y[1::]) - np.array(ego_y[0:-1])) / self.deltaT
        ego_xdd = (ego_xdot[1::] - ego_xdot[0:-1]) / self.deltaT
        ego_ydd = (ego_ydot[1::] - ego_ydot[0:-1]) / self.deltaT
        try:
            k = (ego_xdot[1::]*ego_ydd - ego_ydot[1::]*ego_xdd) / (ego_xdot[1::]**2 + ego_ydot[1::]**2)**(3/2)
        except:
            k = 0

        # 4. calculate lateral acc score
        ego_histort_lateral_acc = np.abs(k) * np.array(ego_speed)**2
        ego_lateral_acc = np.mean(ego_histort_lateral_acc[1::])
        decision_score.lateral_acc = self.hyper_parameter.calculate_acc_score(ego_lateral_acc, self.hyper_parameter.lateral_acc if ego_lateral_acc > 0 else self.hyper_parameter.lateral_dec)

        # 5. calculate lateral jerk score
        ego_lateral_jerk = np.mean((ego_histort_lateral_acc[1::] - ego_histort_lateral_acc[0:-1])/self.deltaT)
        decision_score.lateral_jerk = self.hyper_parameter.calculate_acc_score(ego_lateral_jerk, self.hyper_parameter.positive_jerk if ego_lateral_jerk > 0 else self.hyper_parameter.negative_jerk)

        # 6. record if the score is 0
        if decision_score.longitudinal_acc == 0.0:
            self.current_reasoning += f"your longitudinal acc is too large, which is {ego_longitudinal_acc}\n"
        if decision_score.longitudinal_jerk == 0.0:
            self.current_reasoning += f"your longitudinal jerk is too large, which is {ego_longitudinal_jerk}\n"
        if decision_score.lateral_acc == 0.0:
            self.current_reasoning += f"your lateral acc is too large, which is {ego_lateral_acc}\n"
        if decision_score.lateral_jerk == 0.0:
            self.current_reasoning += f"your lateral jerk is too large, which is {ego_lateral_jerk}\n"
        return decision_score.comfort_score()
    
    def PassRedLight(self, model: ReplayModel) -> bool:
        # get the new 10 frames info, if the current lane is junction and last lane is normallane
        # judge the traffic light
        if len(model.ego.laneIDQ) < 11:
            return False
        if model.ego.laneID in model.roadgraph.Junction_Dict and model.sr.ego.laneIDQ[-11][0] not in model.roadgraph.Junction_Dict and model.ego.laneID not in self.red_junctionlane_record:
            current_lane = model.roadgraph.get_lane_by_id(model.ego.laneID)
            if current_lane.currTlState != None:
                if current_lane.currTlState == "r" or current_lane.currTlState == "R":
                    self.red_junctionlane_record.append(model.sr.ego.laneID)
                    self.current_reasoning += "you pass the red light\n"
                    return True
        return False
    
    def judge_wait_traffic_light(self, model: ReplayModel) -> bool:
        """If the next lane is a junctionlane, and ego is 15m away from the stop line and the speed is less than 0.5 m/s, it is judged that the vehicle is waiting for a red light.

        Args:
            model (ReplayModel)

        Returns:
            bool: True if wait for red light
        """
        if len(model.ego.laneIDQ) < 11:
            return False
        if model.ego.laneID not in model.roadgraph.Junction_Dict:
            ego_availablelanes = model.ego.next_available_lanes
            next_lane = model.roadgraph.get_available_next_lane(model.sr.ego.laneID, ego_availablelanes)
            if isinstance(next_lane, JunctionLane):
                if next_lane.currTlState == "r" or next_lane.currTlState == "R":
                    lane_length = model.rb.get_lane_by_id(model.sr.ego.laneID).length
                    if lane_length - model.sr.ego.lanePos <= self.hyper_parameter.stop_distance and model.sr.ego.speed <= self.hyper_parameter.judge_speed:
                        return True
        return False
    
    def CalculateDrivingMile(self, model: ReplayModel) -> None:
        # if the car just go to new edge, add the length of the last edge
        if model.sr.ego.laneIDQ[-11].split("-")[0] != model.sr.ego.laneIDQ[-1].split("-")[0]:
            if model.sr.ego.laneIDQ[-11][0] not in model.roadgraph.Junction_Dict:
                self.driving_mile += model.rb.get_lane_by_id(model.sr.ego.laneIDQ[-11]).length
            else:
                self.driving_mile += model.rb.get_lane_by_id(model.sr.ego.laneIDQ[-11]).length
        return

    def SaveResultinDB(self, model: ReplayModel) -> None:
        conn = sqlite3.connect(model.dataBase)
        cur = conn.cursor()
        # add result data
        cur.execute(
            """UPDATE resultINFO SET total_score = {},complete_percentage={}, drive_score={} where egoID = {};""".format
            (
                self.final_s, self.complete_percentage, self.decision_score.eval_score(self.hyper_parameter), model.sr.ego.id
            )
        )

        conn.commit()
        conn.close()
        
    @property
    def final_s(self) -> float:
        self.final_score = self.decision_score.eval_score(self.hyper_parameter) * self.complete_p
        return self.final_score
    
    @property
    def complete_p(self) -> float:
        self.complete_percentage = self.driving_mile / self.route_length
        return self.complete_percentage
    
def getSVTrajectory(vehicle: Vehicle, roadgraph: RoadGraph, available_lanes, T: float) -> List[State]:
    """Get the future 5s trajectory of vehicle

    Args:
        vehicle (Vehicle): predict object
        roadgraph (RoadGraph)
        available_lanes (_type_)
        T (float): predict duration (s)

    Returns:
        List[State]: state in future T times
    """
    # judge the vehicle lane and position
    prediction_trajectory = Trajectory()
    
    # if the vehicle's lane position is near the end of the lane, need the next lane to predict
    next_lane = roadgraph.get_available_next_lane(
        vehicle.laneID, available_lanes)
    
    current_lane = roadgraph.get_lane_by_id(vehicle.laneID)
    lanes = [current_lane, next_lane] if next_lane != None else [
        current_lane]
    if next_lane != None:
        next_next_lane = roadgraph.get_next_lane(next_lane.id)
        if next_next_lane != None:
            lanes.append(next_next_lane)
            next_3_lane = roadgraph.get_next_lane(next_next_lane.id)
            if next_3_lane != None:
                lanes.append(next_3_lane)
    
    for t in range(0, T*20, 1):
        t = t/20
        prediction_trajectory.states.append(State(s = vehicle.lanePos + vehicle.speed * t, t=t))
    prediction_trajectory.frenet_to_cartesian(lanes, State(s = vehicle.lanePos, s_d = vehicle.speed,laneID=vehicle.laneID, yaw=vehicle.yaw, x=vehicle.x, y=vehicle.y, vel=vehicle.speed, acc=vehicle.accel, t=0))
    
    return prediction_trajectory.states