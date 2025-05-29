#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodo_robot");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hola desde brazo_robotico";

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}