from datetime import datetime
import re
import time

import cv2
import os
import base64
import numpy as np
import requests
from rich import print
from functools import wraps
from urllib3.exceptions import NameResolutionError, MaxRetryError
from socket import gaierror
from requests.exceptions import ConnectionError

import logger, logging
from simInfo.EnvDescriptor import EnvDescription
from trafficManager.traffic_manager import TrafficManager, LaneChangeException
from simModel.Model import Model
from simModel.MPGUI import GUI

from simModel.DataQueue import QuestionAndAnswer

from trafficManager.common.vehicle import Behaviour
from simInfo.CustomExceptions import (
    CollisionException, LaneChangeException, 
    CollisionChecker, record_result,
    BrainDeadlockException, TimeOutException
)

from PIL import Image, ImageOps
from typing import List, Optional
import io

decision_logger = logger.setup_app_level_logger(
    logger_name="LLMAgent", file_name="InternVL_decision.log")
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

    
def NPImageEncode(npimage: np.ndarray) -> str:
    """
    Encode a numpy array image to base64 format.

    Args:
        npimage (np.ndarray): The numpy array image to be encoded.

    Returns:
        str: The base64 encoded string representation of the input image.
    """
    _, buffer = cv2.imencode('.png', npimage)
    npimage_base64 = base64.b64encode(buffer).decode('utf-8')
    return npimage_base64

# ----------------------------------------------------------------------

# Define a driver agent based on GPT-4V

# ----------------------------------------------------------------------
class VLMAgent:
    def __init__(self, max_tokens: int = 4000) -> None:
        self.api_key = os.environ.get('OPENAI_API_KEY')
        self.max_tokens = max_tokens
        self.content = []
        self.index = 0


    def send_request(
        self,
        image_list: List[np.ndarray],
        question: str,
        history: Optional[List]
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
        # Open the image file in binary mode, convert it to base64 and decode it to ASCII
        image_list = [ImageOps.expand(image, border=5, fill='black') for image in image_list]
        # 按照最小的高度进行拼接
        min_height = min([image.size[1] for image in image_list])
        image_list = [image.convert('RGB') for image in image_list]
        image_list = [image.resize((int(image.size[0] * min_height/ image.size[1]), min_height), Image.LANCZOS) for image in image_list]
        
        image = Image.new('RGB', (sum([image.size[0] for image in image_list]), min_height))
        cur_width = 0
        for i in range(0, len(image_list)):
            image.paste(image_list[i], (cur_width, 0))
            cur_width += image_list[i].size[0]

        # image.save(f"results_interVL/interVL_image/{self.index}.jpg")
        # self.index += 1
        buffered = io.BytesIO()
        image.save(buffered, format="JPEG")
        image_encoded = base64.b64encode(buffered.getvalue()).decode('ascii')

        # Prepare the data to be sent as JSON
        data = {
            'question': question,
            'image': image_encoded,
            'history': history
        }

        # Send a POST request to the Flask app
        response = requests.post(service, json=data)

        # Parse the JSON response
        response_data = response.json()

        return response_data

    def addImageBase64(self, image_base64: str):
        """
        Adds an image encoded in base64 to the prompt content list.

        Args:
            image_base64 (str): The base64 encoded string of the image.

        Returns:
            None
        """
        imagePrompt = {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{image_base64}",
                "detail": "low"
            }
        }
        self.content.append(imagePrompt)

    def addTextPrompt(self, textPrompt: str):
        textPrompt = {
            "type": "text",
            "text": textPrompt
        }
        self.content.append(textPrompt)

    def request(self):
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }

        payload = {
            "model": "gpt-4-vision-preview",
            "temperature": 0.5,
            "messages": [
                {
                    "role": "user",
                    "content": self.content
                }
            ],
            "max_tokens": self.max_tokens
        }
        response = requests.post(
            "https://api.openai.com/v1/chat/completions",
            headers=headers,
            json=payload
        )
        self.content = []

        return response.json()

    def str2behavior(self, decision: str) -> Behaviour:
        """
        Convert a string decision to a Behaviour enum.

        Args:
            decision (str): The decision string to be converted to Behaviour.
            
        Returns:
            Behaviour: The corresponding Behaviour enum for the given decision string.
            
        Raises:
            NotImplementedError: If the decision string is not recognized.
        """
        if decision.upper() == 'IDLE':
            return Behaviour.IDLE
        elif decision.capitalize() == 'Acceleration':
            return Behaviour.AC
        elif decision.capitalize() == 'Deceleration':
            return Behaviour.DC
        elif decision.capitalize() == 'Turn-right':
            return Behaviour.LCR
        elif decision.capitalize() == 'Turn-left':
            return Behaviour.LCL
        else:
            errorStr = f'The decision `{decision}` is not implemented yet!'
        raise NotImplementedError(errorStr)


    @retry_on_exception(
        (ConnectionError,NameResolutionError, MaxRetryError, gaierror)
    )
    def makeDecision(self, image_list, question, history):
        """
        A function that makes a decision based on a prompt, measures the time it takes to make the decision, and returns various relevant data including the behavior, the decision message, prompt tokens, completion tokens, total tokens, and the time cost.
        """
        start = time.time()
        response = self.send_request(image_list, question, history)
        
        ans = response['response']
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
    

if __name__ == '__main__':
    with open(os.path.dirname(os.path.abspath(__file__)) + "/simInfo/example_QA/system_v3.txt", "r") as f:
        SYSTEM_PROMPT = f.read()

    # Simulation settings
    ego_id = '50'
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
    database = 'results_interVL/' + stringTimestamp + '.db'

    # init simulation
    model = Model(
        egoID=ego_id, netFile=sumo_net_file, rouFile=sumo_rou_file,
        cfgFile=sumo_cfg_file, dataBase=database, SUMOGUI=sumo_gui,
        CARLACosim=True, carla_host=carla_host, carla_port=carla_port
    )
    planner = TrafficManager(model)
    descriptor = EnvDescription()
    collision_checker = CollisionChecker()
    model.start()

    gui = GUI(model)
    gui.start()

    # init GPT-4V-based driver agent
    InternVL = VLMAgent()

    # example QA
    with open(os.path.dirname(os.path.abspath(__file__)) + "/results_interVL/example_QA/example_QA.txt", "r") as f:
        QA = f.read()
        example_message = QA.split("======")[0]
        example_answer = QA.split("======")[1]
    example_QA = f"## Here is an example question and answer\n\n{example_message}\n\n{example_answer}\n\n"

    # close loop simulation
    total_start_time = time.time()
    try:
        while not model.tpEnd:
            model.moveStep()
            collision_checker.CollisionCheck(model)
            if model.timeStep % 10 == 0:
                roadgraph, vehicles = model.exportSce()
                if model.tpStart and roadgraph:
                    # get the text prompt: available actions, navigation and ego state
                    actionInfo = descriptor.getAvailableActionsInfo(roadgraph, vehicles)
                    naviInfo = descriptor.getNavigationInfo(roadgraph, vehicles)
                    egoInfo = descriptor.getEgoInfo(vehicles)
                    currentLaneInfo = descriptor.getCurrentLaneInfo(roadgraph, vehicles)
                    TotalInfo = '## Available actions\n\n' + actionInfo + '\n\n' + '## Navigation information\n\n' + currentLaneInfo + egoInfo + naviInfo
                    # get the image prompt: the left front, front, and right front
                    images = model.getCARLAImage(1, 1)
                    front_img = images[-1].CAM_FRONT
                    front_left_img = images[-1].CAM_FRONT_LEFT
                    front_right_img = images[-1].CAM_FRONT_RIGHT
                    PILImage = [Image.fromarray(img) for img in [front_left_img, front_img, front_right_img]]
                    if isinstance(front_img, np.ndarray):

                        question = SYSTEM_PROMPT+example_QA+"# Here is the current scenario\n"+'The next images are stitched together images taken by the vehicle\'s left front camera, front camera, and right front camera, with a black line segment separating them.\n'+f'\nThe current frame information is:\n{TotalInfo}\n'+'Now, please tell me your answer. Please think step by step and make sure it is right.'

                        # print("================================")
                        # print(question)
                        # print("------")
                        
                        # get the decision made by the driver agent
                        (
                            behaviour, ans, 
                            timecost
                        ) = InternVL.makeDecision(PILImage, question, None)
                        # put the decision into the database
                        model.putQA(
                            QuestionAndAnswer(
                                currentLaneInfo+egoInfo, naviInfo, actionInfo, '', 
                                ans, 0, 0, 0, 
                                timecost, int(behaviour)
                            )
                        )
                        # plan trajectories according to the decision made by driver agent
                        trajectories = planner.plan(
                            model.timeStep * 0.1, roadgraph, vehicles, Behaviour(behaviour), other_plan=False
                        )
                        model.setTrajectories(trajectories)
                else:
                    model.ego.exitControlMode()

            model.updateVeh()
    except (
        CollisionException, LaneChangeException, 
        BrainDeadlockException, TimeOutException
        ) as e:
        # record the failed decision made by the driver agent
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