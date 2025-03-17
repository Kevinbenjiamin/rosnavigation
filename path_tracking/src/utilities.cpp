#include "path_tracking/utilities.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace utilities {

    //从四元数中提取偏航角（Yaw）
double getYaw(const geometry_msgs::Quaternion& quat){
    tf2::Quaternion q{quat.x, quat.y, quat.z, quat.w};
    tf2::Matrix3x3 mat{q}; //将四元数转换为旋转矩阵
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw); //小车导航中，ros系统通常使用四元数表示里程计orientation信息，但是很多时候需要rpy表示更加直观方便
    return yaw; //滚转角，俯仰角，偏航角
}

//计算路径中两个点之间的偏航角
double getPathYaw(unsigned int ind, const std::vector<geometry_msgs::PoseStamped>& path){
    return getYaw(path[ind].pose.position, path[ind+1].pose.position);
}

//计算两点之间的偏航角
double getYaw(const geometry_msgs::Point& start, const geometry_msgs::Point& end){
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    return atan2(dy, dx);
}

//找到路径中距离给定点最近的点
unsigned int closetPoint(const geometry_msgs::Point& p, const std::vector<geometry_msgs::PoseStamped>& path){
    double minDist = distSqrd(p, path[0].pose.position);
    unsigned int minInd = 0;
    for (unsigned int i = 1; i < path.size(); i++){
        double dist = distSqrd(p, path[i].pose.position);
        if (dist < minDist){
            minDist = dist;
            minInd = i;
        }
    }
    return minInd;
}

//for purepursuit
//找到路径中距离给定点最近的点
unsigned int ppclosetPoint(const geometry_msgs::Point& p, const std::vector<geometry_msgs::PoseStamped>& path){
    double minDist = distSqrd(p, path[0].pose.position);
    unsigned int minInd = 0;
    for (unsigned int i = 1; i < path.size(); i++){
        double dist = distSqrd(p, path[i].pose.position);
        if (dist < minDist){
            minDist = dist;
            minInd = i;
        }
    }
    return minInd+10;
}


//计算两点之间的距离平方
double distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) + (p2.z-p1.z)*(p2.z-p1.z); /// z or not???
}

//计算车辆当前位置与路径段（start 到 end）之间的横向误差。
double crossTrackError(const geometry_msgs::Point& vehicle, const geometry_msgs::Point& start, const geometry_msgs::Point& end){
    double pathYaw = getYaw(start, end);
    return cos(pathYaw)*(vehicle.y-start.y)-sin(pathYaw)*(vehicle.x-start.x);
}

//从里程计数据中提取车辆的速度
double velocity(const nav_msgs::Odometry& odom){
    return length(odom.twist.twist.linear);
}

//调用 length(const geometry_msgs::Vector3&) 函数计算线速度的大小
double length(const geometry_msgs::Vector3& vec){
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

//规范化角度到 [-π, π] 范围
double validAngle(double angle){
    if (angle > M_PI){
        angle-= 2*M_PI;
    } else if (angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

}