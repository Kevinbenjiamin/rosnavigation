#include "path_server/PathServer.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "path_server");
    ROS_INFO("path_server node initialized");
    //私有句柄，通常用于参数的调用
    ros::NodeHandle nh{"~"};
    PathServer pathServer{&nh};
    bool finished = false;
    ros::Rate loopRate{30};
    while(ros::ok()){
        finished = pathServer.update();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}