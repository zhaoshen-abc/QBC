#include <iostream>
#include <math.h>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros_qt_demo/uav_data.h"
//traj save in WUN
ros::Time lastTime;
double deltaTime;
double pose[3];
bool isYUp;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        pub_ = n_.advertise<ros_qt_demo::uav_data>("uav_data", 1);
        //Topic you want to subscribe
        sub_ = n_.subscribe("/vrpn_client_node/QT/pose", 1, &SubscribeAndPublish::callback, this);

    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        //PUBLISHED_MESSAGE_TYPE output;
        //.... do something with the input and generate the output...
        ros_qt_demo::uav_data output;
        if(isYUp){
            output.pose.position.x=msg->pose.position.x;
            output.pose.position.z=msg->pose.position.y;
            output.pose.position.y=-msg->pose.position.z;
            output.pose.orientation.w=msg->pose.orientation.w;
            output.pose.orientation.x=msg->pose.orientation.x;
            output.pose.orientation.y=msg->pose.orientation.y;
            output.pose.orientation.z=msg->pose.orientation.z;

        }else{
            output.pose.position.x=msg->pose.position.x;
            output.pose.position.y=msg->pose.position.y;
            output.pose.position.z=msg->pose.position.z;
            output.pose.orientation.w=msg->pose.orientation.w;
            output.pose.orientation.x=msg->pose.orientation.z;
            output.pose.orientation.y=msg->pose.orientation.x;
            output.pose.orientation.z=msg->pose.orientation.y;
        }
        pub_.publish(output);
    }

private:
    ros::NodeHandle n_; 
    ros::Publisher pub_;
    ros::Subscriber sub_;

};//End of class SubscribeAndPublish

// void trans_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     deltaTime = (msg->header.stamp - lastTime).toSec();
//     double vel = sqrt(pow(msg->pose.position.x - pose[0], 2)
//                + pow(msg->pose.position.y - pose[1], 2)
//                + pow(msg->pose.position.z - pose[2], 2))/deltaTime;
//     optiVel << msg->header.stamp <<" "<< vel << std::endl;

// 	if (isZUp) {
//     	optiTraj << msg->header.stamp <<" "<< msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << " "<< msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << std::endl;
// 	} else {
//     	optiTraj << msg->header.stamp <<" "<< msg->pose.position.x << " " << msg->pose.position.z << " " << msg->pose.position.y << " "<< msg->pose.orientation.x << " " << msg->pose.orientation.z << " " << msg->pose.orientation.y << " " << msg->pose.orientation.w << std::endl;

// 	}
//      ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "translabel");
    ros::NodeHandle nh("~");

    nh.param<bool>("Y_up", isYUp, true);

    // ros::Subscriber optitrack_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose_unsigned", 50, trans_cb);
    SubscribeAndPublish SAPObject;
	ros::spin();

}
