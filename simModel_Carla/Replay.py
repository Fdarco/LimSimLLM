from __future__ import annotations
from datetime import datetime
from typing import Dict
import sqlite3
import dill
import os

from simModel.MovingScene import SceneReplay
from simModel.CarFactory import Vehicle, egoCar
from utils.trajectory import Trajectory, State
from Roadgraph import RoadGraph
import carla




class ReplayModel:
    def __init__(self, dataBase: str, startFrame: int = None) -> None:
        
        self.updateInterval=10
        
        #设置回放起始帧
        print(
            '[green bold]Model initialized at {}.[/green bold]'.format(
                datetime.now().strftime('%H:%M:%S.%f')[:-3]
            )
        )
        self.dataBase = dataBase
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()

        # minTimeStep
        cur.execute("""SELECT MAX(frame) FROM frameINFO;""")
        maxTimeStep = cur.fetchone()[0] - 200
        if maxTimeStep < 0:
            maxTimeStep = 0
        cur.execute("""SELECT MIN(frame) FROM frameINFO;""")
        minTimeStep = cur.fetchone()[0]
        if startFrame:
            if startFrame > maxTimeStep:
                print(
                    '[yellow]The start frame is too large, and is reassigned to[/yellow] %i.'
                    % maxTimeStep
                )
                self.timeStep = maxTimeStep
            elif startFrame < minTimeStep:
                print(
                    '[yellow]The start frame is too small, and is reassigned to[/yellow] %i.'
                    % minTimeStep
                )
                self.timeStep = minTimeStep
            else:
                self.timeStep = startFrame
        else:
            self.timeStep = minTimeStep
        
        #读取路网
        self.roadgraph = RoadGraph()
        self.check_and_load_mapcache()
        
        #初始化自车
        cur.execute("""SELECT * FROM simINFO;""")
        simINFO = cur.fetchone()
        _, egoID, netBoundary = simINFO
        if egoID:
            self.egoID = egoID
            self.ego = self.initVeh(egoID, self.timeStep)
        else:
            raise TypeError('Please select the appropriate database file.')

        cur.close()
        conn.close()

        # self.sr = SceneReplay(self.rb, self.ego)#不用sr了，还是在模型里面维护一个AOI
        self.vehINAoI:Dict[int,Vehicle]={}
        self.outOfAoI: Dict[int, Vehicle] = {}
        
        self.currVehicls:Dict[int,Vehicle]={}
        self.currVehicls[egoID]=self.ego

        self.tpEnd = 0

    @property
    def sr(self):
        return self
    @property
    def rb(self):
        return self.roadgraph
    
    def moveStep(self):
        if not self.tpEnd:
            self.timeStep += self.updateInterval
            self.getSce()
            self.updateTrafficLight()
        else:
            return
    def updateTrafficLight(self):
        class faketl:
            def __init__(self,tls) -> None:
                if tls=='g':
                    self.state=carla.TrafficLightState.Green
                elif tls=='r':
                    self.state=carla.TrafficLightState.Red
                elif tls=='y':
                    self.state=carla.TrafficLightState.Yellow
                else:
                    raise ValueError
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            """SELECT id, currPhase FROM trafficLightStates
            WHERE frame = {};""".format(
                self.timeStep
            )
        )
        tls_list = cur.fetchall()
        for tls in tls_list:
            jl=self.roadgraph.Junction_Dict[tls[0]]
            jl.traffic_light=faketl(tls[1])

        cur.close()
        conn.close()
    
    def getSce(self):
        nextFrameVehs = self.getNextFrameVehs()
        if self.ego.id in nextFrameVehs:
            for veh_id in nextFrameVehs:
                if veh_id not in self.currVehicls:
                    veh = self.initVeh(veh_id, self.timeStep)
                    self.updateVeh(veh)
                    self.currVehicls[veh_id]=veh
                else:
                    self.updateVeh(self.currVehicls[veh_id])
        else:
            if not self.tpEnd:
                print('[green]The ego car has reached the destination.[/green]')
                self.tpEnd = 1
        destroyed_vid=[vid for vid in self.currVehicls.keys() if vid not in nextFrameVehs]
        for vid in destroyed_vid:
            del self.currVehicls[vid]
    
    def check_and_load_mapcache(self):
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute("""SELECT * FROM simINFO;""")
        simINFO = cur.fetchone()
        _,egoID,map_name=simINFO
        
        map_cache_path=os.path.join(os.getcwd(),'simModel_Carla','map_cache', f"{map_name}.pkl")
        
        if os.path.exists(map_cache_path):
            with open(map_cache_path, 'rb') as f:
                self.roadgraph = dill.load(f)
            #transform wp_list of lane from dict to carla.Waypoint, because waypoint cant be pickled
            print("Loaded roadgraph from cache.")
        else:
            raise FileNotFoundError
        
        cur.close()
        conn.close()
    
    def dbTrajectory(self, vehid: str, currFrame: int) -> Trajectory:
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            """SELECT frame, x, y, yaw, speed, accel, laneID, lanePos, routeIdx, vtag FROM frameINFO
            WHERE vid = "{}" AND frame >= {} AND frame < {};""".format(
                vehid, currFrame, currFrame + 5*self.updateInterval
            )
        )
        frameData = cur.fetchall()
        if frameData:
            # if the trajectory is segmented in time, only the
            # data of the first segment will be taken.
            validSeq = [frameData[0]]
            for i in range(len(frameData) - 1):
                if frameData[i + 1][0] - frameData[i][0] == self.updateInterval:
                    validSeq.append(frameData[i + 1])

            tState = []
            for vs in validSeq:
                state = State(
                    x=vs[1],
                    y=vs[2],
                    yaw=vs[3],
                    vel=vs[4],
                    acc=vs[5],
                    laneID=vs[6],
                    s=vs[7],
                    routeIdx=vs[8],
                )
                tState.append(state)
            dbTrajectory = Trajectory(states=tState)
        else:
            return

        cur.close()
        conn.close()
        return dbTrajectory,validSeq[0][9]
    
    def initVeh(self, vid: str, currFrame: int) -> Vehicle:
        dbTrajectory,vtag = self.dbTrajectory(vid, currFrame)
        if vid == self.egoID:
            veh = egoCar(vid)
        else:
            veh = Vehicle(vid)
        veh.dbTrajectory = dbTrajectory

        vType = self.dbVType(vid)
        length, width, maxAccel, maxDecel, maxSpeed, vTypeID, routes = vType
        veh.length = length
        veh.width = width
        veh.maxAccel = maxAccel
        veh.maxDecel = maxDecel
        veh.maxSpeed = maxSpeed
        veh.vTypeID = vTypeID
        veh.routes = routes.split(' ')

        return veh

    def dbVType(self, vid: str):
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            """SELECT length, width,
            maxAccel, maxDecel, maxSpeed, vTypeID, routes FROM vehicleINFO
            WHERE vid = '%s';"""
            % vid
        )

        vType = cur.fetchall()

        cur.close()
        conn.close()
        return vType[0]

    
    def getNextFrameVehs(self):
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            """SELECT DISTINCT vid FROM frameINFO
            WHERE frame = {};""".format(
                self.timeStep
            )
        )
        nextFrameVehs = cur.fetchall()
        nextFrameVehs = [query[0] for query in nextFrameVehs]
        cur.close()
        conn.close()
        return nextFrameVehs

    def updateVeh(self, veh: Vehicle):
        self.setDBTrajectory(veh)
        self.getAvailableLanes(veh,self.timeStep)
        if veh.dbTrajectory and veh.dbTrajectory.xQueue:
            (x, y, yaw, speed, accel, laneID, lanePos,
             routeIdx) = veh.dbTrajectory.pop_last_state_r()
            veh.xQ.append(x)
            veh.yQ.append(y)
            veh.yawQ.append(yaw)
            veh.speedQ.append(speed)
            veh.accelQ.append(accel)
            # if veh.dbTrajectory.laneIDQueue:
            veh.laneIDQ.append(laneID)
            veh.lanePosQ.append(lanePos)
            veh.routeIdxQ.append(routeIdx)
            
            if veh.vtag =='AoI':
                if veh.id in self.vehINAoI:
                    pass
                else:
                    self.vehINAoI[veh.id]=veh
                    if veh.id in self.outOfAoI:
                        del self.outOfAoI[veh.id]
            elif veh.vtag=='outOfAoI':
                if veh.id in self.outOfAoI:
                    pass
                else:
                    self.outOfAoI[veh.id]=veh
                    if veh.id in self.vehINAoI:
                        del self.vehINAoI[veh.id]
                
    def getAvailableLanes(self,veh:Vehicle,currFrame:int)->set:
        conn = sqlite3.connect(self.dataBase)
        cur = conn.cursor()
        cur.execute(
            """SELECT frame,nextAvailableLanes FROM frameINFO
            WHERE vid = "{}" AND frame >= {} AND frame < {};""".format(
                veh.id, currFrame, currFrame + 5*self.updateInterval
            )
        )
        frameData = cur.fetchall()
        if frameData:
            # if the trajectory is segmented in time, only the
            # data of the first segment will be taken.
            validSeq = [frameData[0]]
            for vs in validSeq:
                next_available_lanes=vs[1]
        else:
            return

        cur.close()
        conn.close()
        veh.next_available_lanes=set(next_available_lanes.split(','))
    
    def setDBTrajectory(self, veh: Vehicle | egoCar):
        dbTrajectory,vtag = self.dbTrajectory(veh.id, self.timeStep)
        if dbTrajectory:
            veh.dbTrajectory = dbTrajectory
        veh.vtag=vtag#记录AOI情况


    def exportRenderData(self):
        try:
            roadgraphRenderData, VRDDict = self.sr.exportRenderData()
            return roadgraphRenderData, VRDDict
        except TypeError:
            return None, None

