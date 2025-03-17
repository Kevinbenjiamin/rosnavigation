#include <iostream>

#include "pathpursuit/Pathpursuit.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "pathpursuit");
    ros::NodeHandle pn{"~"};

    PurePursuit ppsuit{&pn};
    ros::Rate loopRate = 40;
    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}