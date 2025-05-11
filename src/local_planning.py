
# DIRECTAMENTE COPIADO DEL COOKBOOK, HAY QUE ACORDARNOS DE MENCIONARLO
import rospy
import tf2_ros

from scipy.spatial.transform import Rotation as R

rospy.init_node("local_planner") # <- If not already initialised, create your node here
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)


def localiseRobot():
    """Localises the robot towards the 'map' coordinate frame. Returns pose in format (x,y,theta)"""
    while True:
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Robot localisation took longer than 1 sec")
            continue

    theta = R.from_quat([
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w]).as_euler("xyz")[2]
    
    return np.array([
        trans.transform.translation.x,
        trans.transform.translation.y,
        theta])
