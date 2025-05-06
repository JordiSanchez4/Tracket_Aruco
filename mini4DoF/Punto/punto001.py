#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped

def publish_point():
    rospy.init_node('point_publisher', anonymous=True)
    pub = rospy.Publisher('/point_marker', PointStamped, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "world"
        point.point.x = 2.39  # Cambia estas coordenadas según necesidad
        point.point.y = 2.96
        point.point.z = 1

        rospy.loginfo(f"Publicando punto: {point.point.x}, {point.point.y}, {point.point.z}")
        pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_point()
    except rospy.ROSInterruptException:
        pass
