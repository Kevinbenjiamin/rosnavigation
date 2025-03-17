#include "path_server/PathServer.h"
#include "path_server/fileHandling.h"

#include <string>

//计算两点之间的距离平方
double distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) + (p2.z-p1.z)*(p2.z-p1.z);
}

//计算路径与里程计之间的距离平方
double distSqrd(const nav_msgs::Path& path, const nav_msgs::Odometry& odom){
    return distSqrd(path.poses.back().pose.position, odom.pose.pose.position);
}

//将里程计数据转换为 PoseStamped，用于将里程计数据添加到路径中
inline geometry_msgs::PoseStamped toPoseStamped(const nav_msgs::Odometry& odom){
    geometry_msgs::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    return ps;
}   

//根据传入的命令（command）初始化 PathServer
PathServer::PathServer(ros::NodeHandle* pn)
    : filePath{pn->param<std::string>("filePath", "myPath.txt")},//路径文件
    resolution{pn->param<double>("resolution", 1.0)} //分辨率
{
    std::string command = pn->param<std::string>("command", "not specified");
    std::string pathTopic = pn->param<std::string>("pathTopic", "path");
    std::string odomTopic = pn->param<std::string>("odomTopic", "odom");
    std::string odomFrame = pn->param<std::string>("odomFrame", "odom");

    ros::NodeHandle nh{};
    if (command == "load"){ //如果命令是 "load"，则从文件中读取路径并发布
        file::readPath(path, filePath);
        pathPub = nh.advertise<nav_msgs::Path>(pathTopic, 1);
        fp = &PathServer::loadUpdate;
        ROS_INFO("PathServer initialized with load command.");
    } else if (command == "save"){
        pathSub = nh.subscribe(pathTopic,1, &PathServer::pathCallback, this);
        fp = &PathServer::saveUpdate;
        ROS_INFO("PathServer initialized with save command.");
    } else if (command == "record"){
        fp = &PathServer::recordUpdate;
        path.header.frame_id = odomFrame;
        odomSub = nh.subscribe(odomTopic, 10, &PathServer::odomCallback, this);
        pathPub = nh.advertise<nav_msgs::Path>(pathTopic, 1);
        ROS_INFO("PathServer initialized with record command.");
    } else {
        ROS_ERROR_STREAM("invalid command: " << command << "\nValid commands are: load, save, record");
        throw std::runtime_error{"invalid command: " + command};
    }
}

//如果当前操作是 "record"，则在对象销毁时保存路径到文件
PathServer::~PathServer(){
    if (fp = &PathServer::recordUpdate){
        file::savePath(path, filePath);
    }
}

//当接收到路径数据时，将其保存到 path 中
void PathServer::pathCallback(const nav_msgs::Path::ConstPtr& msg){
    path = *msg;
}

//当接收到里程计数据时，如果路径为空或新点与最后一个点的距离超过 resolution，则将新点添加到路径中。
void PathServer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if (path.poses.size() == 0 || distSqrd(path, *msg) > resolution*resolution){
        path.poses.push_back(toPoseStamped(*msg));
    }
}

//通用更新：调用当前操作模式对应的更新函数（loadUpdate、saveUpdate 或 recordUpdate）。
bool PathServer::update(){
    return (this->*fp)();
}

//保存更新：如果路径不为空，则返回 true，表示路径已保存
bool PathServer::saveUpdate(){
    // return true if path has been set
    // this means that the path has been saved
    return path.poses.size() > 0;
}

//加载更新：发布加载的路径，并返回false
bool PathServer::loadUpdate(){
    path.header.stamp = ros::Time::now();
    pathPub.publish(path);
    return false;
}


//记录更新：每隔 saveInterval 秒保存一次路径，并发布当前路径
bool PathServer::recordUpdate(){
    constexpr unsigned int saveInterval = 15;
    static auto lastSave = ros::Time::now();
    ros::Duration elapsed = ros::Time::now() - lastSave;
    if (elapsed.toSec() >= saveInterval){
        file::savePath(path, filePath);
        lastSave = ros::Time::now();
    }
    path.header.stamp = ros::Time::now();
    pathPub.publish(path);
    return false;
}