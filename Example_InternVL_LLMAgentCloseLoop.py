import os
import textwrap
import time
from rich import print
from typing import List, Tuple
from datetime import datetime
from simInfo.Memory import DrivingMemory
from simInfo.EnvDescriptor import EnvDescription

from simInfo.CustomExceptions import (
    CollisionChecker, CollisionException, 
    record_result, LaneChangeException, 
    BrainDeadlockException, TimeOutException
)
from simModel.Model import Model
from simModel.MPGUI import GUI
from trafficManager.traffic_manager import TrafficManager

from simModel.DataQueue import QuestionAndAnswer

import logger, logging
from trafficManager.common.vehicle import Behaviour

import requests
from urllib3.exceptions import NameResolutionError, MaxRetryError
from socket import gaierror
from functools import wraps
import re

decision_logger = logger.setup_app_level_logger(
    logger_name="LLMAgent", file_name="internvl_decision.log")
LLM_logger = logging.getLogger("LLMAgent").getChild(__name__)

def retry_on_exception(exceptions, delay=1, max_retries=5):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            retries = 0
            while retries < max_retries:
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    print(f"Caught exception: {e}. Retrying after {delay} seconds...")
                    time.sleep(delay)
                    retries += 1
            return func(*args, **kwargs)
        return wrapper
    return decorator


class LLMAgent:
    def __init__(
        self, use_memory: bool = True, delimiter: str = "####"
    ) -> None:
        """
        Initialize the LLMAgent_closeloop object.

        Args:
            use_memory (bool, optional): Flag indicating whether to use memory. Defaults to True.
            delimiter (str, optional): Delimiter string. Defaults to "####".
        """

        """ 
        For azure user, if you want to use azure key in this project, you need to check if your azure has deployed embedding model. You need to set your environment as follow and modify the code in file "simInfo/Memory.py" and "simInfo/Reflection.py":
        export OPENAI_API_TYPE="azure"
        export OPENAI_API_KEY='your azure key'
        export OPENAI_API_BASE="your azure node"
        export OPENAI_API_VERSION="your api version"
        export EMBEDDING_MODEL="your embedding model"
        """
        db_path = os.path.dirname(os.path.abspath(__file__)) + "/db/" + "memory_library/" # the path for the memory database
        self.agent_memory = DrivingMemory(db_path=db_path)
        self.few_shot_num = 3

        self.use_memory = use_memory
        self.delimiter = delimiter

        self.llm_source = {
            "prompt_tokens": 0,
            "completion_tokens": 0,
            "total_tokens": 0,
            "cost": 0
        }

    def send_request(
        self,
        descriptions: list = None
    ):
        """
        A function that sends a POST request to a service with a question, image, and history.
        
        Parameters:
            service (str): The service URL to send the request to.
            question (str): The question to be sent in the request, Not Null.
            image_path (str): The path to the image file to be included in the request, Not Null.
            history (Optional[List]): A list of historical data to be included in the request, optional.
            
        Returns:
            dict: A dictionary containing the JSON response data from the service.
            ```JSON
            # Example JSON response
            {
                'response': 'The answer is 42',
                'errors': ''
                'history': []
            }
            ```
        """
        service = 'http://127.0.0.1:8000/api'

        scenario_description = descriptions[0]
        available_actions = descriptions[1]
        driving_intensions = descriptions[2]

        with open(os.path.dirname(os.path.abspath(__file__)) + "/simInfo/system_message_internvl.txt.txt", "r") as f:
            system_message = f.read()

        human_message = textwrap.dedent(f"""\
        # Here is the current scenario:
        ## Driving scenario description:
        {scenario_description}

        ## Navigation instruction:
        {driving_intensions}

        ## Available actions:
        {available_actions}

        ## 
        Remember to follow the format instructions and think more steps.
        You can stop reasoning once you have a valid action to take. 
        """)

        # Prepare the data to be sent as JSON
        data = {
            'question': system_message+human_message,
            'image': None,
            'history': ''
        }

        # Send a POST request to the Flask app
        response = requests.post(service, json=data)

        # Parse the JSON response
        response_data = response.json()

        return response_data
    
    @retry_on_exception(
        (ConnectionError,NameResolutionError, MaxRetryError, gaierror)
    )
    def makeDecision(self, descriptions):
        """
        A function that makes a decision based on a prompt, measures the time it takes to make the decision, and returns various relevant data including the behavior, the decision message, prompt tokens, completion tokens, total tokens, and the time cost.
        """
        start = time.time()
        response = self.send_request(descriptions)
        
        ans = response['response']
        print(ans)
        end = time.time()
        timeCost = end - start
        decision = ans.split('## Decision\n')[-1]
        behavior = None
        if decision:
            pattern = r"[38412]"
            result_list = re.findall(pattern, decision)
            if len(result_list) > 0:
                result = int(result_list[0])
            else:
                raise ValueError
            behavior = Behaviour(result)

        else:
            raise ValueError('GPT-4V did not return a valid decision')

        return (
            behavior, ans, timeCost)

if __name__ == "__main__":
    ego_id = "50"
    sumo_gui = False
    sumo_cfg_file = './networkFiles/CarlaTown06/Town06.sumocfg'
    sumo_net_file = "./networkFiles/CarlaTown06/Town06.net.xml"
    sumo_rou_file = "./networkFiles/CarlaTown06/carlavtypes.rou.xml,networkFiles/CarlaTown06/Town06.rou.xml"
    carla_host = '127.0.0.1'
    carla_port = 2000
    step_length = 0.1
    tls_manager = 'sumo'
    sync_vehicle_color = True
    sync_vehicle_lights = True

    stringTimestamp = datetime.strftime(datetime.now(), '%Y-%m-%d_%H-%M-%S')
    database = 'results_InternVL/' + stringTimestamp + '.db'

    # step1. init LLM_Driver
    model = Model(
        egoID=ego_id, netFile=sumo_net_file, rouFile=sumo_rou_file,
        cfgFile=sumo_cfg_file, dataBase=database, SUMOGUI=sumo_gui,
        CARLACosim=False, carla_host=carla_host, carla_port=carla_port
    )
    planner = TrafficManager(model)
    agent = LLMAgent(use_memory=False)
    descriptor = EnvDescription()
    collision_checker = CollisionChecker()
    model.start()

    gui = GUI(model)
    gui.start()

    action_list = []
    total_start_time = time.time()
    # step2. run the model, use LLM to make decision per second. PS: the unit of timeStep is 0.1s
    try:
        while not model.tpEnd:
            model.moveStep()
            collision_checker.CollisionCheck(model)
            if model.timeStep % 10 == 0:
                roadgraph, vehicles = model.exportSce()
                if model.tpStart and roadgraph:
                    LLM_logger.info(f"--------------- timestep is {model.timeStep} ---------------")
                    # 2.1 get current description based on rules
                    navInfo = descriptor.getNavigationInfo(roadgraph, vehicles)
                    actionInfo = descriptor.getAvailableActionsInfo(roadgraph, vehicles)
                    envInfo = descriptor.getEnvPrompt(roadgraph, vehicles)

                    start_time = time.time()
                    # 2.2 make decision by LLM
                    ego_behaviour, response, timeCost = agent.makeDecision([envInfo, actionInfo, navInfo])
                    descriptor.decision = ego_behaviour # update the last decision

                    # 2.3 put the QA into the memory, the cost can not get in this version
                    # TODO: get the cost from the LLM, add the InternVL's API
                    current_QA = QuestionAndAnswer(envInfo, navInfo, actionInfo, '', response, 0, 0, 0, time.time()-start_time, ego_behaviour)

                    model.putQA(current_QA)
                    trajectories = planner.plan(
                        model.timeStep * 0.1, roadgraph, vehicles, Behaviour(ego_behaviour), other_plan=False
                    )
                    action_list.append(ego_behaviour)
                    if len(action_list) > 10:
                        last_10_actions = action_list[-10::]
                        last_10_actions.sort()
                        if last_10_actions[0] == last_10_actions[-1] and model.ms.ego.speed <= 0.1:
                            raise BrainDeadlockException()
                    if len(action_list) > 80:
                        raise TimeOutException()
                    model.setTrajectories(trajectories)
                else:
                    model.ego.exitControlMode()

            model.updateVeh()

    # step3. record the result
    except (
        CollisionException, LaneChangeException, 
        BrainDeadlockException, TimeOutException
        ) as e:
        record_result(model, total_start_time, False, str(e))
        model.dbBridge.commitData()
    except Exception as e:
        model.dbBridge.commitData()
        raise e
    else:
        record_result(model, total_start_time, True, None)
    finally:
        model.destroy()
        gui.terminate()
        gui.join()

    