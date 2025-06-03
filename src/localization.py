import numpy as np
import rospy
from typing import Tuple
from sklearn.neighbors import KNeighborsClassifier
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from custom_types import Map, Point, MapPoint, CostmapMapPoint
from scipy.ndimage import gaussian_filter


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
    coordinates = lambda i, j: (i * data.info.resolution + data.info.origin.position.x + (data.info.resolution / 2),
                                j * data.info.resolution + data.info.origin.position.y + (data.info.resolution / 2))
    
    walls = np.reshape(data.data, (data.info.height, data.info.width))
    convoluted_walls = gaussian_filter(walls, sigma=1.2)
    convoluted_walls = convoluted_walls / np.max(convoluted_walls)
    map = np.zeros((data.info.height, data.info.width), dtype=Point)
    costmap = np.zeros((data.info.height, data.info.width), dtype=Point)
    for y in range(data.info.height):
        for x in range(data.info.width):
            p = MapPoint(*coordinates(x, y), walls[y][x] == 100)
            c = CostmapMapPoint(*coordinates(x, y), convoluted_walls[y][x])
            map[x][y] = p
            costmap[x][y] = c

    return Map(
        width=data.info.width,
        height=data.info.height,
        origin=(data.info.origin.position.x, data.info.origin.position.y),
        resolution=data.info.resolution,
        map=map,
        costmap=costmap
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


def predict_robot_position(map: Map, laser_scan: np.ndarray) -> Tuple[Tuple[int, int], Point, np.ndarray]:
    """
    Predict the robot position based on the laser scan data, using K-Nearest Neighbors (kNN) algorithm.
    
    Args:
        map (Map): map object containing the map data
        laser_scan (np.ndarray): laser scan data
        
    Returns:
        Tuple[Tuple[int, int], Point]: predicted robot position in map indices and world coordinates
    """
    real_amount_of_walls = len(laser_scan)
    x_train = []
    y_train = []
    for point in map.map.flatten():
        x_train.append((point.x, point.y))
        y_train.append(1 if point.wall else 0)

    knn = KNeighborsClassifier(n_neighbors=1)
    knn.fit(x_train, y_train)

    possible_positions = []
    for i in np.arange(0, np.round(map.width * map.resolution), 1):
        for j in np.arange(0, np.round(map.height * map.resolution), 1):
            possible_positions.append((i, j))

    possible_laser_scans = []
    for x, y in possible_positions:
        new_laser_scan = np.array([(p.x + x, p.y + y) for p in laser_scan])
        possible_laser_scans.append(new_laser_scan)

    amount_of_walls = []
    for laser_scan in possible_laser_scans:
        amount_of_walls.append(np.sum(knn.predict(laser_scan)))

    position_candidate = None
    laser_scan_candidate = None
    min_distance = np.inf
    for position, laser_scan, amount in zip(possible_positions, possible_laser_scans, amount_of_walls):
        distance = np.abs(amount - real_amount_of_walls)
        if distance < min_distance:
            position_candidate = position
            laser_scan_candidate = laser_scan
            min_distance = distance

    x_idx = int(np.round(position_candidate[0] / map.resolution))
    y_idx = int(np.round(position_candidate[1] / map.resolution))
    if (map.origin[0]) < 0:
        x_idx += int(np.round(np.abs(map.origin[0]) / map.resolution))
    if (map.origin[1]) < 0:
        y_idx += int(np.round(np.abs(map.origin[1]) / map.resolution))

    return (x_idx, y_idx), Point(*position_candidate), np.array([Point(*p) for p in laser_scan_candidate])
