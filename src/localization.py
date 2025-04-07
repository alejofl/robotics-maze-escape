import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from custom_types import Map, Point, MapPoint


def get_map() -> OccupancyGrid:
    """
    Get the map from the map server.
    
    Returns:
        OccupancyGrid: one-dimensional array of the map
    """
    service = rospy.ServiceProxy("static_map", GetMap)
    return service().map


def deserialize_map(data: OccupancyGrid) -> Map:
    """
    Deserialize the map into a 2D array.
    
    Args:
        data (OccupancyGrid): one-dimensional array of the map
    
    Returns:
        Map: deserialized map object
    """
    coordinates = lambda i, j: (i * data.info.resolution + data.info.origin.position.x + data.info.resolution / 2,
                                j * data.info.resolution + data.info.origin.position.y + data.info.resolution / 2)
    
    walls = np.reshape(data.data, (data.info.height, data.info.width))
    map = np.zeros((data.info.height, data.info.width), dtype=Point)
    for y in range(data.info.height):
        for x in range(data.info.width):
            p = MapPoint(*coordinates(x, data.info.height - y), walls[x][y] == 100)
            map[y][x] = p

    return Map(
        width=data.info.width,
        height=data.info.height,
        resolution=data.info.resolution,
        map=map,
    )


def get_laser_scan(map_resolution: float) -> np.ndarray:
    """
    Get the laser scan data from the robot.
    
    Args:
        map_resolution (float): resolution of the map
    
    Returns:
        LaserScan: laser scan data
    """
    scan = rospy.wait_for_message("scan", LaserScan)
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment
    range_min = scan.range_min
    range_max = scan.range_max
    points = []

    for i, range in enumerate(scan.ranges):
        if range < range_min or range > range_max:
            continue

        angle = angle_min + i * angle_increment
        x = range * np.cos(angle)
        if x <= 0:
            x -= map_resolution / 2
        else:
            x += map_resolution / 2
        y = range * np.sin(angle)
        if y <= 0:
            y -= map_resolution / 2
        else:
            y += map_resolution / 2
        points.append(Point(x, y))

    return np.array(points)
