import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

"""
!@brief pub_velocity             Robot velocity publisher.
"""
pub_velocity = rospy.Publisher("cmd_vel", Twist, queue_size=10)

"""
!@brief lidarPolarToPoints       converts point from polar to cartesian coordinate system
@param lidar_polar_data          data from lidar in polar coordinates
@return                          lidar data in cartesian coordinates
"""
def lidarPolarToPoints(lidar_polar_data: LaserScan):
    return []

"""
!@brief getNextDestination       Calculates the next robot goal in local coordinate system
                                 (it works like a carrot on the stick).
@param lidar_points              Points from lidar.
@return                          The destination coordinates.
"""
def getNextDestination(lidar_points):
    return 0, 0

"""
!@brief wallAtFront              Checks whether there is a wall in front of the robot.
@param lidar_points              Points from lidar.
@return                          True if the wall is present, false if the way is free.
"""
def wallAtFront(lidar_points) -> bool:
    return False

"""
!@brief PID                      Simple PID driver.
@param u                         Driver input signal.
@return                          Driver output signal.
"""
def PID(u: float) -> float:
    return 0.

"""
!@brief move                     Send the move forward command to the robot, with given
                                 additional rotation.
@param rotation                  Rotation speed, from -1 to 1.
"""
def move(rotation: float):
    pass

"""
!@brief stop                     Send the stop command to the robot.
"""
def stop():
    pass

"""
@brief lidarCallback             Process lidar data.
@param lidar_polar_data          ROS lidar message.
"""
def lidarCallback(lidar_polar_data: LaserScan):
    lidar_points = lidarPolarToPoints(lidar_polar_data)

    dest = getNextDestination(lidar_points)

    if True:
        pass
    else:
        pass

if __name__ == '__main__':
    rospy.init_node("wseiz_1")
    rate = rospy.Rate(10)
    sub_lidar = rospy.Subscriber("base_scan", LaserScan, lidarCallback, queue_size=10)

    while not rospy.is_shutdown():
        rate.sleep()
