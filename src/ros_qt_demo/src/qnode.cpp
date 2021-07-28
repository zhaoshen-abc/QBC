/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_qt_demo/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_ros_qt_demo {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) :
    init_argc(argc),
    init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"class1_ros_qt_demo");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    subscriber = n.subscribe<ros_qt_demo::uav_data>("uav_data", 1000,  boost::bind(&QNode::doMsg,this,_1));
    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    ros::init(init_argc,init_argv,"class1_ros_qt_demo");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    subscriber = n.subscribe<ros_qt_demo::uav_data>("uav_data", 1000,  boost::bind(&QNode::doMsg,this,_1));
    start();
    log(Info,std::string("qnode init finish"));
	return true;
}

void QNode::doMsg(const ros_qt_demo::uav_data::ConstPtr& msg_p){
    //ROS_INFO("I hear:%s",msg_p->data.c_str());
    //		std_msgs::String msg;
    //		std::stringstream ss;
    //		ss << "hello world " << count;
    //		msg.data = ss.str();
    //		chatter_publisher.publish(msg);
    //		log(Info,std::string("I sent: ")+msg.data);
    msg.pose=msg_p->pose;
    msg.connected=msg_p->connected;
    msg.armed=msg_p->armed;
    msg.guided=msg_p->guided;
    msg.mode=msg_p->mode;
    msg.manual_input=msg_p->manual_input;
    msg.adhere=msg_p->adhere;

    log(Info,std::string("I hear: ")+msg.mode.c_str());
    emit this->test();
    emit this->getdata();
}

void QNode::run() {
    log(Info,std::string("qnode run"));
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
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

}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
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
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace class1_ros_qt_demo
