#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped

def publish_static_transform():
    # Publicador de transformación base_link -> camera_link
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "camera_link"

    # Posición de la cámara con respecto al brazo (por ejemplo, 10 cm encima y 5 cm al frente)
    static_transformStamped.transform.translation.x = 0.05
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.10

    # Orientación: sin rotación
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)

def main():
    rospy.init_node('camera_point_publisher')

    # Asegura que el TF esté listo
    rospy.sleep(1.0)
    publish_static_transform()

    point_pub = rospy.Publisher('/camera_point', PointStamped, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "camera_link"

        # Coordenadas en el marco de la cámara (por ejemplo, algo frente a ella)
        point.point.x = 0.5
        point.point.y = 0.0
        point.point.z = 0.0

        point_pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
