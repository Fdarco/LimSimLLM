import carla
from typing import Dict, Union, Set, List,Optional
import numpy as np
from simModel_Carla.DataQueue import CameraImages

class CAMActor:
    def __init__(
        self,world,name: str,
        transform: carla.Transform, fov: str,
        vehicle, max_size: int = 5
    ) -> None:
        """
        Initializes the CameraSensor object.

        Args:
            name (str): The name of the CameraSensor object.
            sync (SimulationSynchronization): The SimulationSynchronization object.
            transform (carla.Transform): The transform of the CameraSensor object.
            fov (str): The field of view of the CameraSensor object.
            vehicle: The vehicle to attach the CameraSensor object to.
            max_size (int, optional): The maximum size of the image list. Defaults to 5.

        Returns:
            None
        """
        self.name = name
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '1600')
        cam_bp.set_attribute('image_size_y', '900')
        cam_bp.set_attribute('fov', fov)
        # cam_bp.set_attribute('sensor_tick', '0.1')
        self.cam = world.spawn_actor(
            cam_bp, transform, attach_to=vehicle
        )
        self.cam.listen(self.process)
        self.max_size = max_size
        self.imageList = []

    def process(self, image):
        image_data = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        image_data = np.reshape(image_data, (image.height, image.width, 4))
        # image_data rgb
        image_data = image_data[:,:,[2,1,0,3]]
        self.addImage(image_data)

    def addImage(self, image_data):
        if len(self.imageList) >= self.max_size:
            self.imageList.pop(0)
        self.imageList.append(image_data)

    def getImage(self):
        return self.imageList[-1] if self.imageList else None

class nuScenesImageExtractor:
    def __init__(self):
        # configure camera
        self.CAM_FONRT_RECORD = False
        self.CAM_FRONT_RIGHT_RECORD = False
        self.CAM_FRONT_LEFT_RECORD = False
        self.CAM_BACK_LEFT_RECORD = False
        self.CAM_BACK_RECORD = False
        self.CAM_BACK_RIGHT_RECORD = False

        # nuScenes cameras settings
        self.cameraInfo = {
            'CAM_FRONT': {
                'transform': carla.Transform(
                    carla.Location(x=1.5, y=0, z=2),
                    carla.Rotation(yaw=0, pitch=0, roll=0)
                ),
                'fov': '70',
                'record': self.CAM_FONRT_RECORD,
            },
            'CAM_FRONT_RIGHT': {
                'transform': carla.Transform(
                    carla.Location(x=1.5, y=0.7, z=2),
                    carla.Rotation(yaw=55, pitch=0, roll=0)
                ),
                'fov': '70',
                'record': self.CAM_FRONT_RIGHT_RECORD
            },
            'CAM_FRONT_LEFT': {
                'transform': carla.Transform(
                    carla.Location(x=1.5, y=-0.7, z=2),
                    carla.Rotation(yaw=-55, pitch=0, roll=0)
                ),
                'fov': '70',
                'record': self.CAM_FRONT_LEFT_RECORD
            },
            'CAM_BACK_LEFT': {
                'transform': carla.Transform(
                    carla.Location(x=-0.7, y=0, z=2),
                    carla.Rotation(yaw=-110, pitch=0, roll=0)
                ),
                'fov': '70',
                'record': self.CAM_BACK_LEFT_RECORD
            },
            'CAM_BACK': {
                'transform': carla.Transform(
                    carla.Location(x=-1.5, y=0, z=2),
                    carla.Rotation(yaw=180, pitch=0, roll=0)
                ),
                'fov': '110',
                'record': self.CAM_BACK_RECORD
            },
            'CAM_BACK_RIGHT': {
                'transform': carla.Transform(
                    carla.Location(x=-0.7, y=0, z=2),
                    carla.Rotation(yaw=110, pitch=0, roll=0)
                ),
                'fov': '70',
                'record': self.CAM_BACK_RIGHT_RECORD
            }
        }

        self.cameras: Dict[str, CAMActor] = {}

    def allCamBeenSetted(self) -> bool:
        return self.CAM_FONRT_RECORD and self.CAM_FRONT_LEFT_RECORD \
            and self.CAM_FRONT_RIGHT_RECORD and self.CAM_BACK_LEFT_RECORD \
            and self.CAM_BACK_RECORD and self.CAM_BACK_RIGHT_RECORD

    def setCameras(self, vehicle,world) -> None:
        if self.allCamBeenSetted():
            return
        else:
            for k, v in self.cameraInfo.items():
                if not v['record']:
                    self.cameras[k] = CAMActor(
                        world,k, self, v['transform'], v['fov'], vehicle
                    )
                    v['record'] = True
                else:
                    continue

    def getCameraImages(self) -> Optional[CameraImages]:
        images = {}
        for k, v in self.cameras.items():
            pic = v.getImage()
            if isinstance(pic, np.ndarray):
                images[k] = pic
            else:
                return None
        return CameraImages(
            images['CAM_FRONT'], images['CAM_FRONT_RIGHT'], images['CAM_FRONT_LEFT'],
            images['CAM_BACK_LEFT'], images['CAM_BACK'], images['CAM_BACK_RIGHT']
        )
