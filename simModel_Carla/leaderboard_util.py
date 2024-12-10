from scenario_runner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from simModel_Carla.Model import Model
from Roadgraph import RoadGraph
from vehicle import Vehicle
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import RoadOption
import xml.etree.ElementTree as ET
import numpy as np
from scenario_runner.srunner.scenariomanager.timer import GameTime
from threading import Thread
from queue import Queue
from queue import Empty
import logging
import copy

import time

import math
import carla
def initDataProvider(model:Model):
        CarlaDataProvider.set_client(model.client)
        CarlaDataProvider.set_traffic_manager_port(model.tm_port)
        CarlaDataProvider.set_world(model.world)
        # CarlaDataProvider.set_random_seed(args.traffic_manager_seed)
        CarlaDataProvider._carla_actor_pool[model.ego.actor.id]=model.ego.actor
        CarlaDataProvider.register_actor(model.ego.actor,model.ego.actor.get_transform())

def route_transform(rd:RoadGraph,veh:Vehicle, hop_resolution=2.0):
    def on_same_lane(x,y):
        return x.road_id ==y.road_id and x.section_id == y.section_id and x.lane_id==y.lane_id
    #将route idx 转换为 wplist
    waypoints_trajectory=[veh.cur_wp.transform.location]
    for idx,rid in enumerate(veh.route):
        if idx!=len(veh.route)-1:#不是最后一位的话，找到与下一个edge连接的junctionlane的起始路点
            edge=rd.Edges[rid]
            junction_lane=edge.next_edge_connect[veh.route[idx+1]]
            j_wplist=rd.Junction_Dict[junction_lane[0]].wp_list
            waypoints_trajectory.append(rd.Junction_Dict[junction_lane[0]].wp_list[len(j_wplist)//2].transform.location)
        else:
            waypoints_trajectory.append(veh.end_waypoint.transform.location)

    grp = GlobalRoutePlanner(CarlaDataProvider.get_map(), hop_resolution)
    # Obtain route plan
    lat_ref, lon_ref = _get_latlon_ref(CarlaDataProvider.get_world())

    route = []
    gps_route = []
    count=0
    for i in range(len(waypoints_trajectory) - 1):

        waypoint = waypoints_trajectory[i]
        waypoint_next = waypoints_trajectory[i + 1]
        interpolated_trace = grp.trace_route(waypoint, waypoint_next)
        idx=0
        for wp, connection in interpolated_trace:
            route.append((wp.transform, connection))
            gps_coord = _location_to_gps(lat_ref, lon_ref, wp.transform.location)
            gps_route.append((gps_coord, connection))
            
            count+=1
            if idx!=len(interpolated_trace)-1:
                
                #对于全局路径中的变道部分，如果变道后的路点的s小于等于当前路点s+sample_resolution，则更改变道后的路点 
                #先判断两个路点的关系,是否是变道
                #还要特别处理连续变道的情况
                waypoint=wp
                waypoint_next = interpolated_trace[idx+1][0]
                left_wp=waypoint.get_left_lane() if waypoint.lane_change==carla.libcarla.LaneChange.Both or waypoint.lane_change==carla.libcarla.LaneChange.Left else None
                right_wp=waypoint.get_right_lane() if waypoint.lane_change==carla.libcarla.LaneChange.Both or waypoint.lane_change==carla.libcarla.LaneChange.Right else None
                
                search_count=0
                while search_count<=1:
                    if left_wp and on_same_lane(left_wp,waypoint_next):
                        check_change_lane_wp=left_wp
                    elif right_wp and on_same_lane(right_wp,waypoint_next):
                        check_change_lane_wp=right_wp
                    elif on_same_lane(waypoint,waypoint_next):
                        check_change_lane_wp=wp
                    else:
                        #如果后续路点在当前路点的前序道路上,删除
                        lane_end=waypoint_next.next_until_lane_end(10)[0].next(1)[0]
                        if on_same_lane(lane_end,waypoint):
                            del interpolated_trace[idx+1]#bug:i
                            print('del back')
                        check_change_lane_wp=None
                    
                    #如果变道后的路点纵向变化不高
                    if check_change_lane_wp and waypoint_next.s <check_change_lane_wp.s+1:
                        # breakpoint()
                        # print(f'found change wp:{count}')
                        interpolated_trace[idx+1]=(check_change_lane_wp.next(hop_resolution)[0],interpolated_trace[idx+1][1])
                        # del interpolated_trace[i+1]
                    search_count+=1
                    waypoint_next=waypoint_next.next_until_lane_end(10)[0].next(1)[0]
            idx+=1
    for idx,wp_tulpe in enumerate(route):
        p=idx+1
        while p<=len(route)-1:
            if route[p][0].location.distance(wp_tulpe[0].location)<=0.3:
                del route[p]
            p+=1           
    return gps_route, route


def _get_latlon_ref(world):
    """
    Convert from waypoints world coordinates to CARLA GPS coordinates
    :return: tuple with lat and lon coordinates
    """
    xodr = world.get_map().to_opendrive()
    tree = ET.ElementTree(ET.fromstring(xodr))

    # default reference
    lat_ref = 42.0
    lon_ref = 2.0

    for opendrive in tree.iter("OpenDRIVE"):
        for header in opendrive.iter("header"):
            for georef in header.iter("geoReference"):
                if georef.text:
                    str_list = georef.text.split(' ')
                    for item in str_list:
                        if '+lat_0' in item:
                            lat_ref = float(item.split('=')[1])
                        if '+lon_0' in item:
                            lon_ref = float(item.split('=')[1])
    return lat_ref, lon_ref

def _location_to_gps(lat_ref, lon_ref, location):
    """
    Convert from world coordinates to GPS coordinates
    :param lat_ref: latitude reference for the current map
    :param lon_ref: longitude reference for the current map
    :param location: location to translate
    :return: dictionary with lat, lon and height
    """

    EARTH_RADIUS_EQUA = 6378137.0   # pylint: disable=invalid-name
    scale = math.cos(lat_ref * math.pi / 180.0)
    mx = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
    my = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat_ref) * math.pi / 360.0))
    mx += location.x
    my -= location.y

    lon = mx * 180.0 / (math.pi * EARTH_RADIUS_EQUA * scale)
    lat = 360.0 * math.atan(math.exp(my / (EARTH_RADIUS_EQUA * scale))) / math.pi - 90.0
    z = location.z

    return {'lat': lat, 'lon': lon, 'z': z}

def setup_sensors(pdm,vehicle, debug_mode=False):
    """
    Create the sensors defined by the user and attach them to the ego-vehicle
    :param pdm : pdm class
    :param vehicle: ego vehicle
    :return:
    """
    bp_library = CarlaDataProvider.get_world().get_blueprint_library()
    for sensor_spec in pdm.sensors():
        # These are the sensors spawned on the carla world
        bp = bp_library.find(str(sensor_spec['type']))
        if sensor_spec['type'].startswith('sensor.camera'):
            bp.set_attribute('image_size_x', str(sensor_spec['width']))
            bp.set_attribute('image_size_y', str(sensor_spec['height']))
            bp.set_attribute('fov', str(sensor_spec['fov']))
            sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                roll=sensor_spec['roll'],
                                                yaw=sensor_spec['yaw'])
        elif sensor_spec['type'].startswith('sensor.lidar'):
            bp.set_attribute('range', str(sensor_spec['range']))
            bp.set_attribute('rotation_frequency', str(sensor_spec['rotation_frequency']))
            bp.set_attribute('channels', str(sensor_spec['channels']))
            bp.set_attribute('upper_fov', str(sensor_spec['upper_fov']))
            bp.set_attribute('lower_fov', str(sensor_spec['lower_fov']))
            bp.set_attribute('points_per_second', str(sensor_spec['points_per_second']))
            sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                z=sensor_spec['z'])
            sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                roll=sensor_spec['roll'],
                                                yaw=sensor_spec['yaw'])
        elif sensor_spec['type'].startswith('sensor.other.gnss'):
            sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                z=sensor_spec['z'])
            sensor_rotation = carla.Rotation()

        # create sensor
        sensor_transform = carla.Transform(sensor_location, sensor_rotation)
        sensor = CarlaDataProvider.get_world().spawn_actor(bp, sensor_transform, vehicle)
        # setup callback
        sensor.listen(CallBack(sensor_spec['id'], sensor, pdm.sensor_interface))
        # pdm._sensors_list.append(sensor)

    # # Tick once to spawn the sensors
    # CarlaDataProvider.get_world().tick()



def setup_sensors(pdm, vehicle):
    """
    Create the sensors defined by the user and attach them to the ego-vehicle
    :param vehicle: ego vehicle
    :return:
    """
    world = CarlaDataProvider.get_world()
    bp_library = world.get_blueprint_library()
    for sensor_spec in pdm.sensors():
        type_, id_, sensor_transform, attributes = preprocess_sensor_spec(sensor_spec)

        # These are the pseudosensors (not spawned)
        if type_ == 'sensor.opendrive_map':
            sensor = OpenDriveMapReader(vehicle, attributes['reading_frequency'])
        elif type_ == 'sensor.speedometer':
            sensor = SpeedometerReader(vehicle, attributes['reading_frequency'])

        # These are the sensors spawned on the carla world
        else:
            bp = bp_library.find(type_)
            for key, value in attributes.items():
                bp.set_attribute(str(key), str(value))
            sensor = CarlaDataProvider.get_world().spawn_actor(bp, sensor_transform, vehicle)

        # setup callback
        sensor.listen(CallBack(id_, type_, sensor, pdm.sensor_interface))#pdm的sensor_interface在基类的地方
        # self._sensors_list.append(sensor)
        # self.sensor_list_names.append([sensor_spec['id'], sensor])

    # Some sensors miss sending data during the first ticks, so tick several times and remove the data
    for _ in range(10):
        world.tick()

class CallBack(object):
    def __init__(self, tag, sensor_type, sensor, data_provider):
        self._tag = tag
        self._data_provider = data_provider

        self._data_provider.register_sensor(tag, sensor_type, sensor)

    def __call__(self, data):
        if isinstance(data, carla.libcarla.Image):
            self._parse_image_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.LidarMeasurement):
            self._parse_lidar_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.RadarMeasurement):
            self._parse_radar_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.GnssMeasurement):
            self._parse_gnss_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.IMUMeasurement):
            self._parse_imu_cb(data, self._tag)
        elif isinstance(data, GenericMeasurement):
            self._parse_pseudosensor(data, self._tag)
        else:
            logging.error('No callback method for this sensor.')

    # Parsing CARLA physical Sensors
    def _parse_image_cb(self, image, tag):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = copy.deepcopy(array)
        array = np.reshape(array, (image.height, image.width, 4))
        self._data_provider.update_sensor(tag, array, image.frame)

    def _parse_lidar_cb(self, lidar_data, tag):
        points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        self._data_provider.update_sensor(tag, points, lidar_data.frame)

    def _parse_radar_cb(self, radar_data, tag):
        # [depth, azimuth, altitute, velocity]
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        points = np.flip(points, 1)
        self._data_provider.update_sensor(tag, points, radar_data.frame)

    def _parse_gnss_cb(self, gnss_data, tag):
        array = np.array([gnss_data.latitude,
                          gnss_data.longitude,
                          gnss_data.altitude], dtype=np.float64)
        self._data_provider.update_sensor(tag, array, gnss_data.frame)

    def _parse_imu_cb(self, imu_data, tag):
        array = np.array([imu_data.accelerometer.x,
                          imu_data.accelerometer.y,
                          imu_data.accelerometer.z,
                          imu_data.gyroscope.x,
                          imu_data.gyroscope.y,
                          imu_data.gyroscope.z,
                          imu_data.compass,
                         ], dtype=np.float64)
        self._data_provider.update_sensor(tag, array, imu_data.frame)

    def _parse_pseudosensor(self, package, tag):
        self._data_provider.update_sensor(tag, package.data, package.frame)


class SensorConfigurationInvalid(Exception):
    """
    Exceptions thrown when the sensors used by the agent are not allowed for that specific submissions
    """

    def __init__(self, message):
        super(SensorConfigurationInvalid, self).__init__(message)


class SensorReceivedNoData(Exception):
    """
    Exceptions thrown when the sensors used by the agent take too long to receive data
    """

    def __init__(self, message):
        super(SensorReceivedNoData, self).__init__(message)


class SensorInterface(object):
    def __init__(self):
        self._sensors_objects = {}
        self._data_buffers = Queue()
        self._queue_timeout = 10

        # Only sensor that doesn't get the data on tick, needs special treatment
        self._opendrive_tag = None

    def register_sensor(self, tag, sensor_type, sensor):
        if tag in self._sensors_objects:
            raise SensorConfigurationInvalid("Duplicated sensor tag [{}]".format(tag))

        self._sensors_objects[tag] = sensor

        if sensor_type == 'sensor.opendrive_map': 
            self._opendrive_tag = tag

    def update_sensor(self, tag, data, frame):
        if tag not in self._sensors_objects:
            raise SensorConfigurationInvalid("The sensor with tag [{}] has not been created!".format(tag))

        self._data_buffers.put((tag, frame, data))

    def get_data(self, frame):
        """Read the queue to get the sensors data"""
        try:
            data_dict = {}
            while len(data_dict.keys()) < len(self._sensors_objects.keys()):
                # Don't wait for the opendrive sensor
                if self._opendrive_tag and self._opendrive_tag not in data_dict.keys() \
                        and len(self._sensors_objects.keys()) == len(data_dict.keys()) + 1:
                    break

                sensor_data = self._data_buffers.get(True, self._queue_timeout)
                if sensor_data[1] != frame:
                    continue
                data_dict[sensor_data[0]] = ((sensor_data[1], sensor_data[2]))

        except Empty:
            raise SensorReceivedNoData("A sensor took too long to send their data")

        return data_dict


def preprocess_sensor_spec(sensor_spec):
    type_ = sensor_spec["type"]
    id_ = sensor_spec["id"]
    attributes = {}

    if type_ == 'sensor.opendrive_map':
        attributes['reading_frequency'] = sensor_spec['reading_frequency']
        sensor_location = carla.Location()
        sensor_rotation = carla.Rotation()

    elif type_ == 'sensor.speedometer':
        delta_time = CarlaDataProvider.get_world().get_settings().fixed_delta_seconds
        attributes['reading_frequency'] = 1 / delta_time
        sensor_location = carla.Location()
        sensor_rotation = carla.Rotation()

    if type_.startswith('sensor.camera'):
        attributes['image_size_x'] = str(sensor_spec['width'])
        attributes['image_size_y'] = str(sensor_spec['height'])
        attributes['fov'] = str(sensor_spec['fov'])
        attributes['lens_circle_multiplier']=str(3.0)
        attributes['lens_circle_falloff']=str(3.0)
        attributes['chromatic_aberration_intensity']=str(0.5)
        attributes['chromatic_aberration_offset']=str(0)

        sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                            z=sensor_spec['z'])
        sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                            roll=sensor_spec['roll'],
                                            yaw=sensor_spec['yaw'])

    elif type_ == 'sensor.lidar.ray_cast':
        attributes['range'] = str(85)
        # if DATAGEN==1:
        # attributes['rotation_frequency'] = str(sensor_spec['rotation_frequency'])
        # attributes['points_per_second'] = str(sensor_spec['points_per_second'])
        # else:
        attributes['rotation_frequency'] = str(10)
        attributes['points_per_second'] = str(600000)
        attributes['channels'] = str(64)
        attributes['upper_fov'] = str(10)
        attributes['lower_fov'] = str(-30)
        attributes['atmosphere_attenuation_rate'] = str(0.004)
        attributes['dropoff_general_rate'] = str(0.45)
        attributes['dropoff_intensity_limit'] = str(0.8)
        attributes['dropoff_zero_intensity'] = str(0.4)

        sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                            z=sensor_spec['z'])
        sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                            roll=sensor_spec['roll'],
                                            yaw=sensor_spec['yaw'])

    elif type_ == 'sensor.other.radar':
        attributes['horizontal_fov'] = str(sensor_spec['horizontal_fov'])  # degrees
        attributes['vertical_fov'] = str(sensor_spec['vertical_fov'])  # degrees
        attributes['points_per_second'] = '1500'
        attributes['range'] = '100'  # meters

        sensor_location = carla.Location(x=sensor_spec['x'],
                                            y=sensor_spec['y'],
                                            z=sensor_spec['z'])
        sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                            roll=sensor_spec['roll'],
                                            yaw=sensor_spec['yaw'])

    elif type_ == 'sensor.other.gnss':
        # attributes['noise_alt_stddev'] = str(0.000005)
        # attributes['noise_lat_stddev'] = str(0.000005)
        # attributes['noise_lon_stddev'] = str(0.000005)
        attributes['noise_alt_bias'] = str(0.0)
        attributes['noise_lat_bias'] = str(0.0)
        attributes['noise_lon_bias'] = str(0.0)

        sensor_location = carla.Location(x=sensor_spec['x'],
                                            y=sensor_spec['y'],
                                            z=sensor_spec['z'])
        sensor_rotation = carla.Rotation()

    elif type_ == 'sensor.other.imu':
        attributes['noise_accel_stddev_x'] = str(0.001)
        attributes['noise_accel_stddev_y'] = str(0.001)
        attributes['noise_accel_stddev_z'] = str(0.015)
        attributes['noise_gyro_stddev_x'] = str(0.001)
        attributes['noise_gyro_stddev_y'] = str(0.001)
        attributes['noise_gyro_stddev_z'] = str(0.001)

        sensor_location = carla.Location(x=sensor_spec['x'],
                                            y=sensor_spec['y'],
                                            z=sensor_spec['z'])
        sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                            roll=sensor_spec['roll'],
                                            yaw=sensor_spec['yaw'])
    sensor_transform = carla.Transform(sensor_location, sensor_rotation)

    return type_, id_, sensor_transform, attributes

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.setDaemon(True)
        thread.start()

        return thread
    return wrapper

class GenericMeasurement(object):
    def __init__(self, data, frame):
        self.data = data
        self.frame = frame

class BaseReader(object):
    def __init__(self, vehicle, reading_frequency=1.0):
        self._vehicle = vehicle
        self._reading_frequency = reading_frequency
        self._callback = None
        self._run_ps = True
        self.run()

    def __call__(self):
        pass

    @threaded
    def run(self):
        first_time = True
        latest_time = GameTime.get_time()
        while self._run_ps:
            if self._callback is not None:
                current_time = GameTime.get_time()

                # Second part forces the sensors to send data at the first tick, regardless of frequency
                if current_time - latest_time > (1 / self._reading_frequency) \
                        or (first_time and GameTime.get_frame() != 0):
                    self._callback(GenericMeasurement(self.__call__(), GameTime.get_frame()))
                    latest_time = GameTime.get_time()
                    first_time = False

                else:
                    time.sleep(0.001)

    def listen(self, callback):
        # Tell that this function receives what the producer does.
        self._callback = callback

    def stop(self):
        self._run_ps = False

    def destroy(self):
        self._run_ps = False

class SpeedometerReader(BaseReader):
    """
    Sensor to measure the speed of the vehicle.
    """
    MAX_CONNECTION_ATTEMPTS = 10

    def _get_forward_speed(self, transform=None, velocity=None):
        """ Convert the vehicle transform directly to forward speed """
        if not velocity:
            velocity = self._vehicle.get_velocity()
        if not transform:
            transform = self._vehicle.get_transform()

        vel_np = np.array([velocity.x, velocity.y, velocity.z])
        pitch = np.deg2rad(transform.rotation.pitch)
        yaw = np.deg2rad(transform.rotation.yaw)
        orientation = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])
        speed = np.dot(vel_np, orientation)
        return speed

    def __call__(self):
        """ We convert the vehicle physics information into a convenient dictionary """

        # protect this access against timeout
        attempts = 0
        while attempts < self.MAX_CONNECTION_ATTEMPTS:
            try:
                velocity = self._vehicle.get_velocity()
                transform = self._vehicle.get_transform()
                break
            except Exception:
                attempts += 1
                time.sleep(0.2)
                continue

        return {'speed': self._get_forward_speed(transform=transform, velocity=velocity)}

class OpenDriveMapReader(BaseReader):
    def __call__(self):
        return {'opendrive': CarlaDataProvider.get_map().to_opendrive()}