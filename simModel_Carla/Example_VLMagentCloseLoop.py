from utils.load_config import load_config
from ego_vehicle_planning import LLMEgoPlanner
from simModel_Carla.MPGUI import GUI
from simModel_Carla.Model import Model
from trafficManager.traffic_manager_carla import TrafficManager
import carla
import time
from math import sqrt
import random
from dataclasses import field
from datetime import datetime
import re
import time
from trafficManager.common.vehicle import Behaviour
from urllib3.exceptions import NameResolutionError, MaxRetryError
from socket import gaierror
from functools import wraps
from simModel_Carla.DataQueue import (
    CameraImages, ImageQueue, QAQueue, QuestionAndAnswer, RenderQueue
)

import cv2
import os
import base64
import numpy as np
import requests


from leaderboard_util import initDataProvider,route_transform,setup_sensors

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

class VLMAgent:
    def __init__(self, max_tokens: int = 4000) -> None:
        self.api_key = 'sk-vYEbj0AB8Y6kJpUT557c391dD4234727B0Bf1a46A5CeBf9f'#os.environ.get('OPENAI_API_KEY')
        self.max_tokens = max_tokens
        self.content = []

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
            "model": "gpt-4o-2024-08-06",
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
            "https://api.key77qiqi.cn/v1/chat/completions",
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
    def makeDecision(self):
        """
        A function that makes a decision based on a prompt, measures the time it takes to make the decision, and returns various relevant data including the behavior, the decision message, prompt tokens, completion tokens, total tokens, and the time cost.
        """
        start = time.time()
        response = self.request()
        print(response)
        ans = response['choices'][0]['message']['content']
        prompt_tokens = response['usage']['prompt_tokens']
        completion_tokens = response['usage']['completion_tokens']
        total_tokens = response['usage']['total_tokens']
        end = time.time()
        timeCost = end - start
        match = re.search(r'## Decision\n(.*)', ans)
        behavior = None
        if match:
            decision = match.group(1)
            behavior = self.str2behavior(decision)
        else:
            raise ValueError('GPT-4V did not return a valid decision')
        return (
            behavior, ans, prompt_tokens, 
            completion_tokens, total_tokens, timeCost)
    

SYSTEM_PROMPT = """
You are GPT-4V(ision), a large multi-modal model trained by OpenAI. Now you act as a mature driving assistant, who can give accurate and correct advice for human driver in complex urban driving scenarios. You'll receive some images from the onboard camera. You'll need to make driving inferences and decisions based on the information in the images. At each decision frame, you receive navigation information and a collection of actions. You will perform scene description, and reasoning based on the navigation information and the front-view image. Eventually you will select the appropriate action output from the action set.
Make sure that all of your reasoning is output in the `## Reasoning` section, and in the `## Decision` section you should only output the name of the action, e.g. `AC`, `IDLE` etc.

Your answer should follow this format:
## Description
Your description of the front-view image.
## Reasoning
reasoning based on the navigation information and the front-view image.
## Decision
one of the actions in the action set.(SHOULD BE exactly same and no other words!)
"""

if __name__=='__main__':
    from simInfo.EnvDescriptor import EnvDescription
    from carlaWrapper import carlaRoadGraphWrapper
    descriptor=EnvDescription()
    
    config_name='./simModel_Carla/exp_config/long_term_config.yaml'
    random.seed(112102)

    stringTimestamp = datetime.strftime(datetime.now(), '%Y-%m-%d_%H-%M-%S')    
    database = 'results/' + stringTimestamp + '.db'
    total_start_time = time.time()

    model:Model=Model(cfgFile=config_name,dataBase=database)

    planner = TrafficManager(model)

    model.start()
    model.runAutoPilot()


    gui = GUI(model)
    gui.start()

    # init GPT-4V-based driver agent
    gpt4v = VLMAgent()
    
    while not model.tpEnd:
        model.moveStep()
        if model.shouldUpdate():
            roadgraph, vehicles = model.exportSce()
            
            start_time=time.time()
            actionInfo = descriptor.getAvailableActionsInfo(carlaRoadGraphWrapper(roadgraph), vehicles)
            naviInfo = descriptor.getNavigationInfo(carlaRoadGraphWrapper(roadgraph), vehicles)
            currentLaneInfo = descriptor.getCurrentLaneInfo(carlaRoadGraphWrapper(roadgraph), vehicles)
            # envInfo = descriptor.getEnvPrompt(carlaRoadGraphWrapper(roadgraph), vehicles)
            egoInfo = descriptor.getEgoInfo(vehicles)
            
            TotalInfo = '## Available actions\n\n' + actionInfo + '\n\n' + '## Navigation information\n\n' + currentLaneInfo + egoInfo + naviInfo

            # get the image prompt: the left front, front, and right front
            images = model.getCARLAImage(1, 1)
            if not images:
                continue
            front_img = images[-1].CAM_FRONT
            front_left_img = images[-1].CAM_FRONT_LEFT
            front_right_img = images[-1].CAM_FRONT_RIGHT
            assert isinstance(front_img, np.ndarray)
            # wrap the prompt and pass it to the driver agent
            gpt4v.addTextPrompt(SYSTEM_PROMPT)
            gpt4v.addTextPrompt('The next three images are images captured by the left front, front, and right front cameras.\n')
            gpt4v.addImageBase64(NPImageEncode(front_left_img))
            gpt4v.addImageBase64(NPImageEncode(front_img))
            gpt4v.addImageBase64(NPImageEncode(front_right_img))
            gpt4v.addTextPrompt(f'\nThe current frame information is:\n{TotalInfo}')
            gpt4v.addTextPrompt('Now, please tell me your answer. Please think step by step and make sure it is right.')
            # get the decision made by the driver agent
            (
                behaviour, ans, 
                prompt_tokens, completion_tokens, 
                total_tokens, timecost
            ) = gpt4v.makeDecision()
            # put the decision into the database
            
            
            model.putQA(
                QuestionAndAnswer(
                    currentLaneInfo+egoInfo, naviInfo, actionInfo, '', 
                    ans, prompt_tokens, completion_tokens, total_tokens, 
                    timecost, int(behaviour)
                )
            )            

            try:
                trajectories = planner.plan(
                    model.timeStep * 0.1, roadgraph, vehicles, Behaviour(behaviour), other_plan=True
                )
            except:
                trajectories=dict()
            
            model.setTrajectories(trajectories)
            print(model.ego.lane_id)
            print(model.ego.behaviour)
            
            world=model.world
            for veh in model.vehicles:
                if veh.trajectory:
                    for state in veh.trajectory.states:
                        world.debug.draw_point(carla.Location(x=state.x,y=state.y,z=0.5),color=carla.Color(r=0, g=0, b=255), life_time=1, size=0.1)
        model.updateVeh()

    #according to collision sensor
    model.record_result(total_start_time, True, None)

    model.destroy()
    gui.terminate()
    gui.join()

