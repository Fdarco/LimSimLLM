1. 在waypoint的left_lane_marking和right_lane_marking中，说明了该车道两边的道路线标志，或许对于高级语义信息的理解有帮助。
   - 例如，对于黄虚线来说，虽然意味着该侧lane属于对向edge，但是在驾驶的时候可以掉头或者超车，这算是一种驾驶的高级信息

2. waypoint中的road_id是跨edge的，所以需要对edge的划分进行更详细的对象车道划分
3. 对于车辆的控制可以用carla自带的controller.py