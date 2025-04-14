
#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <std_msgs>
#include "visualization_msgs"

#include <sstream>





int main(int argc, char **argv){
    ros::init(argc, argv, "sample")

    ros::NodeHandle n;

    ros::Publisher sample_pub = n.advertise<std_msgs::String>("sampleMsg", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
       
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}

