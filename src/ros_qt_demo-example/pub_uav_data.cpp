#include "ros/ros.h"
#include "ros_qt_demo/uav_data.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"uav_data");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<ros_qt_demo::uav_data>("uav_data",1000);

    //4.组织被发布的消息，编写发布逻辑并发布消息
    ros_qt_demo::uav_data data;
    data.pose.position.x=72.35213;
    data.pose.position.y=134.51234;
    data.pose.position.z=2.45468;
    data.connected=false;
    data.armed=true;
    data.guided=true;
    data.mode="maual";
    data.manual_input=true;
    data.adhere={true,true,true,true};


    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(data);
        r.sleep();
        ros::spinOnce();
    }



    return 0;
}
