import rospy
import tf2_ros
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler

def publish_tf():
    rospy.init_node('tf_and_points_publisher', anonymous=True)
    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        camera_transform = geometry_msgs.msg.TransformStamped()
        camera_transform.header.stamp = rospy.Time.now()
        camera_transform.header.frame_id = 'world'
        camera_transform.child_frame_id = 'camera'
        camera_transform.transform.translation.x = 0.360
        camera_transform.transform.translation.y = 0.00
        camera_transform.transform.translation.z = 0.410

        roll = -math.pi / 2
        pitch = 0
        yaw = 0

        quat = quaternion_from_euler(roll, pitch, yaw)
        camera_transform.transform.rotation.x = quat[0]
        camera_transform.transform.rotation.y = quat[1]
        camera_transform.transform.rotation.z = quat[2]
        camera_transform.transform.rotation.w = quat[3]

        tf_broadcaster.sendTransform(camera_transform)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_tf()
    except rospy.ROSInterruptException:
        pass
