import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

obstacles_file = "../data/world_obstacles.txt"
goal_file = "../data/goal.txt"
path_file = "../data/path.txt"

def initialize_marker(mid, mtype, color, alpha=1.0):
    """ Returns a new and initialized marker """
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.id = mid
    marker.ns = "nsl"
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    if mtype == 'strip':
        marker.type = Marker.LINE_STRIP
    else:
        marker.type = Marker.LINE_LIST
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    if color == 'red':
        marker.color.r = 1.0 # red
    elif color == 'blue':
        marker.color.b = 1.0 # blue
    else:
        marker.color.g = 1.0 # green
    marker.color.a = alpha # alpha

    return marker

def main():
    # Create ROS node
    rospy.init_node('marker_test', anonymous=False)

    # Publish markers to 'marker_test' topic
    pub_node = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    # Initialize MarkerArray
    marker_arr = MarkerArray()
    marker_cnt = 0

    # Marker for growing obstacles
    # marker1 = initialize_marker('red')

    from grow_obstacles import grow_obstacles
    obstacles, _ = grow_obstacles()
    for obstacle in obstacles:
        # Create a new marker for each obstacle
        marker = initialize_marker(marker_cnt, 'strip', 'red')
	marker_cnt += 1
        points = []
        for point in obstacle:
            pt = Point(point[0] * .01, point[1] * .01, 0)
	    # print(pt)
            points.append(pt)
	points.append(points[0]) # to make lines connect
	marker.points = points
        marker_arr.markers.append(marker)

    # Marker for visibility graph
    from gene_vgraph import gene_vgraph
    edges, _ = gene_vgraph(obstacles_file, goal_file)
    # Create a new marker for edges
    marker = initialize_marker(marker_cnt, 'list', 'blue', alpha=.3)
    marker_cnt += 1
    points = []
    for edge in edges:
        pt1 = Point(edge[0][0] * .01, edge[0][1] * .01, 0)
        pt2 = Point(edge[1][0] * .01, edge[1][1] * .01, 0)
        points.append(pt1)
        points.append(pt2)
    marker.points = points
    marker_arr.markers.append(marker)

    # Marker for shortest path
    from shortest_path import shortest_path
    path, _ = shortest_path(obstacles_file, goal_file)
    # Create a new marker for shortest path
    marker = initialize_marker(marker_cnt, 'strip', 'green')
    marker_cnt += 1
    points = []
    for point in path:
        pt = Point(point[0] * .01, point[1] * .01, 0)
        points.append(pt)
    marker.points = points
    marker_arr.markers.append(marker)

    # Create points
    # points = []
    # pt1 = Point(1, 1, 1)
    # pt2 = Point(2, 2, 2)
    # points.append(pt1)
    # points.append(pt2)

    # marker.points = points

    # marker_arr.markers.append(marker)

    # Draw markers
    while not rospy.is_shutdown():
        pub_node.publish(marker_arr)
        rospy.Rate(30).sleep()

if __name__ == '__main__':
    try:
        main()
    except None:
        rospy.loginfo('Failed')

