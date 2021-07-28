#ifndef class1_ros_qt_demo_SERVICE_Pose_H
#define class1_ros_qt_demo_SERVICE_Pose_H

//#endif // SERVICE_CLINET_H

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include "std_msgs/String.h"
#include <QThread>
#include <QStringListModel>
#include "ros_qt_demo/FormationCommand.h"
#include "ros_qt_demo/MissionCommand.h"
#include "ros_qt_demo/ModeCommand.h"
#include "ros_qt_demo/PoseCommand.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_ros_qt_demo {

/*****************************************************************************
** Class
*****************************************************************************/

class Service_Pose : public QThread {
    Q_OBJECT
public:
    Service_Pose(int argc, char** argv ,QStringListModel *logging_model_);
    virtual ~Service_Pose();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url,ros_qt_demo::MissionCommand mc);
    bool init(const std::string &master_url, const std::string &host_url,ros_qt_demo::FormationCommand fc);
    bool init(const std::string &master_url, const std::string &host_url,ros_qt_demo::ModeCommand mdc);
    bool init(const std::string &master_url, const std::string &host_url,ros_qt_demo::PoseCommand pc);
    void run();
    void doMsg(const std_msgs::String::ConstPtr& msg_p);

    /*********************
    ** Logging
    **********************/
    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };

    enum Service_type {
             Mission,
             Formation,
             Mode,
             Pose
     };

    void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
    ros::ServiceClient mission_client;
    ros::ServiceClient formation_client;
    ros::ServiceClient mode_client;
    ros::ServiceClient pose_client;
    QStringListModel *logging_model;
    Service_type service_type;
    ros_qt_demo::MissionCommand mc_;
    ros_qt_demo::FormationCommand fc_;
    ros_qt_demo::ModeCommand mdc_;
    ros_qt_demo::PoseCommand pc_;
};

}  // namespace class1_ros_qt_demo

#endif /* class1_ros_qt_demo_QNODE_HPP_ */
