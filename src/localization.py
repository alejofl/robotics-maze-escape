import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from custom_types import Map, Point


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
            p = Point(*coordinates(x, data.info.height - y), walls[x][y] == 100)
            map[y][x] = p

    return Map(
        width=data.info.width,
        height=data.info.height,
        resolution=data.info.resolution,
        map=map,
    )
