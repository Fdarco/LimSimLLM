import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from Roadgraph import RoadGraph, RESOLUTION

# ------------------ draw function ------------------ #
import random

def random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    rgb_code = '#%02x%02x%02x' % (r, g, b)
    # return carla.Color(r=r, g=g, b=b)
    return rgb_code


def draw_roadgraph(roadgraph: RoadGraph):
    """绘制roadgraph

    Args:
        roadgraph (RoadGraph): 构建的edge section lane junction
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    for edge in roadgraph.Edges.values():
        edge_color = random_color()

        # ------------- draw the section ------------- #
        for section_id in edge.section_list:
            section_box = []
            section = roadgraph.Sections[section_id]
            color = random_color()

            # ------------- draw the lane ------------- #
            for lane_id in section.lanes.values():
                lane = roadgraph.NormalLane_Dict[lane_id]
                ax.plot([wp.transform.location.x for wp in lane.wp_list], [wp.transform.location.y for wp in lane.wp_list], color=color)

            # ------------- draw the section box ------------- #
            left_lane_id = section.lanes[min(section.lanes.keys())]
            right_lane_id = section.lanes[max(section.lanes.keys())]
            left_lane = roadgraph.NormalLane_Dict[left_lane_id]
            right_lane = roadgraph.NormalLane_Dict[right_lane_id]
            section_box.extend([wp_i.transform.location.x, wp_i.transform.location.y] for wp_i in left_lane.wp_list)
            section_box.reverse()
            section_box.extend([wp_i.transform.location.x, wp_i.transform.location.y] for wp_i in right_lane.wp_list)
            ax.add_patch(Polygon(xy=section_box, fill=True, alpha = 0.4, color='red'))

        # ------------- draw the junction ------------- #
        for next_edge, junction_list in edge.next_edge_connect.items():
            color = random_color()
            for junction_id in junction_list:
                junction_lane = roadgraph.Junction_Dict[junction_id]
                for wp in junction_lane.start_wp.next_until_lane_end(RESOLUTION):
                    ax.plot([wp.transform.location.x], [wp.transform.location.y], color = color, marker='o', markersize=1)

    plt.show()