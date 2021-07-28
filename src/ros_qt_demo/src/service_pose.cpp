#include "ros/ros.h"
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_qt_demo/service_pose.hpp"
#include <cstdlib>

namespace class1_ros_qt_demo {

/*****************************************************************************
** Implementation
*****************************************************************************/

Service_Pose::Service_Pose(int argc, char** argv ,QStringListModel *logging_model_) :
    init_argc(argc),
    init_argv(argv),
    logging_model(logging_model_)
    {}

Service_Pose::~Service_Pose() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

//void Service_client::doMsg(const std_msgs::String::ConstPtr& msg_p){
//    //ROS_INFO("I hear:%s",msg_p->data.c_str());
//    //		std_msgs::String msg;
//    //		std::stringstream ss;
//    //		ss << "hello world " << count;
//    //		msg.data = ss.str();
//    //		chatter_publisher.publish(msg);
//    //		log(Info,std::string("I sent: ")+msg.data);
//    std_msgs::String msg;
//    std::stringstream ss;
//    ss << msg_p->data.c_str();
//    msg.data = ss.str();
//    log(Info,std::string("I hear: ")+msg.data);
//}

bool Service_Pose::init() {
    log(Info,std::string("Service init without param!"));
    ros::init(init_argc,init_argv,"class2_ros_qt_demo");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    mission_client = n.serviceClient<ros_qt_demo::MissionCommand>("/Mission_Service");
    formation_client = n.serviceClient<ros_qt_demo::FormationCommand>("/Formation_Service");
    start();
    return true;
}

bool Service_Pose::init(const std::string &master_url, const std::string &host_url,ros_qt_demo::MissionCommand mc) {
//    log(Info,std::string("Service init4!"));
    mc_=mc;
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"class2_ros_qt_demo");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    mission_client = n.serviceClient<ros_qt_demo::MissionCommand>("/Mission_Service");
    service_type=Mission;
    log(Info,std::string("Service start!"));
    start();
    return true;
}

bool Service_Pose::init(const std::string &master_url, const std::string &host_url,
                        ros_qt_demo::FormationCommand fc) {
//    log(Info,std::string("Service init5!"));
    fc_=fc;
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"class2_ros_qt_demo");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    formation_client = n.serviceClient<ros_qt_demo::FormationCommand>("/Formation_Service");
    service_type=Formation;
    start();
    return true;
}


bool Service_Pose::init(const std::string &master_url, const std::string &host_url,
                        ros_qt_demo::ModeCommand mdc) {
//    log(Info,std::string("Service init5!"));
    mdc_=mdc;
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"class2_ros_qt_demo");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    mode_client = n.serviceClient<ros_qt_demo::ModeCommand>("/Mode_Service");
    service_type=Mode;
    start();
    return true;
}


bool Service_Pose::init(const std::string &master_url, const std::string &host_url,
                        ros_qt_demo::PoseCommand pc) {
    log(Info,std::string("Service init5!"));
    pc_=pc;
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"class2_ros_qt_demo");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    pose_client = n.serviceClient<ros_qt_demo::PoseCommand>("/Pose_Service");
    service_type=Pose;
    start();
    return true;
}

void Service_Pose::run() {
    log(Info,std::string("Service run!"));
    switch(service_type){
        case Mission:{
            log(Info,std::string("service_type=Mission,waiting for server..."));
            ros::service::waitForService("/Mission_Service");
            // 6.发送请求,返回 bool 值，标记是否成功
            bool flag = mission_client.call(mc_);
            // 7.处理响应
            if (flag)
            {
                log(Info,std::string("response is:")+mc_.response.check);            }
            else
            {
                log(Error,std::string("request send fail...."));
            }
            break;
        }
        case Formation:{
            log(Info,std::string("service_type=Formation,waiting for server..."));
            ros::service::waitForService("/Formation_Service");
            // 6.发送请求,返回 bool 值，标记是否成功
            bool flag = formation_client.call(fc_);
            // 7.处理响应
            if (flag)
            {
                log(Info,std::string("response is:")+fc_.response.check);
            }
            else
            {
                log(Error,std::string("request send fail...."));
            }
            break;
        }
        case Mode:{
            log(Info,std::string("service_type=Mode,waiting for server..."));
            ros::service::waitForService("/Mode_Service");
            // 6.发送请求,返回 bool 值，标记是否成功
            bool flag = mode_client.call(mdc_);
            // 7.处理响应
            if (flag)
            {
                log(Info,std::string("response is:")+mdc_.response.check);
            }
            else
            {
                log(Error,std::string("request send fail...."));
            }
            break;
        }
        case Pose:{
            log(Info,std::string("service_type=Pose,waiting for server..."));
            ros::service::waitForService("/Pose_Service");
            // 6.发送请求,返回 bool 值，标记是否成功
            bool flag = pose_client.call(pc_);
            // 7.处理响应
            if (flag)
            {
                log(Info,std::string("response is:")+pc_.response.check);
            }
            else
            {
                log(Error,std::string("request send fail...."));
            }
            break;
        }
    }

//    ros::spin();
//	while ( ros::ok() ) {

//		std_msgs::String msg;
//		std::stringstream ss;
//		ss << "hello world " << count;
//		msg.data = ss.str();
//		chatter_publisher.publish(msg);
//		log(Info,std::string("I sent: ")+msg.data);
//		ros::spinOnce();
//		loop_rate.sleep();
//		++count;
//	}
//	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
//	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void Service_Pose::log( const LogLevel &level, const std::string &msg) {
    logging_model->insertRows(logging_model->rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Info) : {
                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model->setData(logging_model->index(logging_model->rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace class1_ros_qt_demo
