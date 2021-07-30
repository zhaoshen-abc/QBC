#include "ros/ros.h"
#include "ros_qt_demo/MissionCommand.h"
#include "ros_qt_demo/FormationCommand.h"
#include "ros_qt_demo/ModeCommand.h"
#include "ros_qt_demo/PoseCommand.h"

// bool 返回值由于标志是否处理成功
bool mission_cb(ros_qt_demo::MissionCommand::Request& req,
          ros_qt_demo::MissionCommand::Response& resp){
    int num1 = req.state;

    ROS_INFO("服务器接收到的请求数据为:num1 = %d",num1);

    //逻辑处理
    if (num1 < 10)
    {
        resp.check = "less than 10";
    }
    else {
        resp.check = "greater than 10";
    }

    //如果没有异常，那么相加并将结果赋值给 resp
    return true;
}

bool formation_cb(ros_qt_demo::FormationCommand::Request& req,
          ros_qt_demo::FormationCommand::Response& resp){
    int num1 = req.formation;

    ROS_INFO("服务器接收到的请求数据为:num1 = %d",num1);

    //逻辑处理
    if (num1 < 10)
    {
        resp.check = "less than 10";
    }
    else {
        resp.check = "greater than 10";
    }
    //如果没有异常，那么相加并将结果赋值给 resp
    return true;
}

bool mode_cb(ros_qt_demo::ModeCommand::Request& req,
          ros_qt_demo::ModeCommand::Response& resp){
    int num1 = req.mode;

    ROS_INFO("服务器接收到的请求数据为:num1 = %d",num1);

    //逻辑处理
    if (num1 < 10)
    {
        resp.check = "less than 10";
    }
    else {
        resp.check = "greater than 10";
    }
    //如果没有异常，那么相加并将结果赋值给 resp
    return true;
}

bool pose_cb(ros_qt_demo::PoseCommand::Request& req,
          ros_qt_demo::PoseCommand::Response& resp){
    int num1 = req.waypoint_number;

    ROS_INFO("服务器接收到的请求数据为:num1 = %d",num1);

    //逻辑处理
    if (num1 < 10)
    {
        resp.check = "less than 10";
    }
    else {
        resp.check = "greater than 10";
    }
    //如果没有异常，那么相加并将结果赋值给 resp
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"Mission");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 服务 对象
    ros::ServiceServer mission_server = nh.advertiseService("/Mission_Service",mission_cb);
    ros::ServiceServer formation_server = nh.advertiseService("/Formation_Service",formation_cb);
    ros::ServiceServer mode_server = nh.advertiseService("/Mode_Service",mode_cb);
    ros::ServiceServer pose_server = nh.advertiseService("/Pose_Service",pose_cb);
    ROS_INFO("服务已经启动....");
    //     5.回调函数处理请求并产生响应
    //     6.由于请求有多个，需要调用 ros::spin()
    ros::spin();
    return 0;
}
