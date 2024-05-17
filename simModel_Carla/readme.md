### USE
1. 打开carla并加载地图
2. 直接运行__main__.py，ego car可以按照给定的路线驾驶

### FIle
1. Network_Structure.py: 路网信息的基本组成 edge, section, lane
2. Roadgraph.py: 路网类型
3. RoadInfoGet.py: 从carla中获取路网
4. vehicle.py: 用于planning的vehicle object，存储vehicle的相关信息(可根据后续任务进行修改)
5. ego_vehicle_planning.py: ego car的AC DC IDLE LCL LCR动作规划
6. __main__.py: demo，包含了如何更新ego的lane_id信息


### TODO List
1. 从carla.actor中获取traffic light信息以及周围的交通参与者(actor)信息
2. 封装函数获取他车的信息流
3. 油门控制

### Thought List
1. 在waypoint的left_lane_marking和right_lane_marking中，说明了该车道两边的道路线标志，或许对于高级语义信息的理解有帮助。
   - 例如，对于黄虚线来说，虽然意味着该侧lane属于对向edge，但是在驾驶的时候可以掉头或者超车，这算是一种驾驶的高级信息