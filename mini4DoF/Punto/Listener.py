#from controller import Supervisor
import rospy
from geometry_msgs.msg import PointStamped

class PointSubscriber:
    def __init__(self):
        self.supervisor = Supervisor()
        self.robot_node = self.supervisor.getFromDef("POINT_MARKER")

        rospy.init_node('webots_point_subscriber', anonymous=True)
        rospy.Subscriber("/point_marker", PointStamped, self.callback)
        self.run()

    def callback(self, msg):
        if self.robot_node:
            position = [msg.point.x, msg.point.y, msg.point.z]
            self.robot_node.getField("translation").setSFVec3f(position)
            rospy.loginfo(f"Actualizando punto en Webots: {position}")

    def run(self):
        while self.supervisor.step(int(self.supervisor.getBasicTimeStep())) != -1:
            rospy.spin()

if __name__ == "__main__":
    PointSubscriber()
