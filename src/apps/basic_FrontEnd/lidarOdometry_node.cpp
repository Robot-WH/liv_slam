
#include <ros/ros.h>
#include <iostream>

// ros消息  
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// tf
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "registration.hpp"
#include "ros_utils.hpp"
#include "tic_toc.h"

  
using namespace std;

typedef pcl::PointXYZI PointT;
const int match_dis = 80;  

class lidarOdometry_node
{
private:
    /* data */
    ros::NodeHandle nh; 
    
    ros::Subscriber points_sub;
    ros::Publisher odom_pub;         // 发布/odom话题
    ros::Publisher pubLaserPath;     // 发布轨迹
    // tf发布
    tf::TransformBroadcaster odom_broadcaster;       // 发布 odom -> Lidar tf变换 
    tf::TransformBroadcaster keyframe_broadcaster;   // odom->KF

    // keyframe parameters 关键帧条件
    double keyframe_delta_trans;  // minimum distance between keyframes
    double keyframe_delta_angle;  //  
    double keyframe_delta_time;   //

    // registration validation by thresholding   非法匹配阈值
    bool transform_thresholding;  //
    double max_acceptable_trans;  //
    double max_acceptable_angle;

    // odometry calculation
    Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe  上一帧与参考关键帧的相对位姿
    Eigen::Matrix4f predict_trans;               // 预测位姿
    Eigen::Matrix4f keyframe_pose;               // keyframe pose      参考关键帧的位姿
    Eigen::Matrix4f odom;   
    nav_msgs::Path laserPath;                    // 记录轨迹  
    bool frist = true;
    ros::Time keyframe_stamp;                    // keyframe time
    pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud  用来匹配的参考关键帧
    // pcl::Registration<PointT, PointT>::Ptr registration; 
    boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> registration;      // 常用多线程NDT  
    std::string odom_frame_id;  
    std::string lidar_frame_id;                  

public:
    lidarOdometry_node(/* args */)
    {
        nh = ros::NodeHandle("~");
        // ROS topics    去除了畸变以及噪声的点云  
        points_sub = nh.subscribe("/processed_points", 100, &lidarOdometry_node::cloud_callback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 32); 
        // 发布轨迹   
        pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
        initialize_params(nh);
    }

    ~lidarOdometry_node(){};

    void initialize_params(ros::NodeHandle nh);
    Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);
    Eigen::Matrix4f matching_scan2scan(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);
    void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};


void lidarOdometry_node::initialize_params(ros::NodeHandle nh) {
    // 设置原点坐标系名字
    odom_frame_id = nh.param<std::string>("odom_frame_id", "/odom");
    lidar_frame_id = nh.param<std::string>("lidar_frame_id", "/lidar_odom");
    // 关键帧选取 
    keyframe_delta_trans = nh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = nh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = nh.param<double>("keyframe_delta_time", 1.0);
    // 运动求解失败   
    transform_thresholding = nh.param<bool>("transform_thresholding", false);
    max_acceptable_trans = nh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = nh.param<double>("max_acceptable_angle", 1.0);
    // 设置匹配方法   默认为ndt    
    registration = Set_NDTOMP_param(nh);
}

/**
 * @brief estimate the relative pose between an input cloud and a keyframe cloud   这个里程计需要重点优化预测值的给定
 * @param stamp  the timestamp of the input cloud
 * @param cloud  the input cloud
 * @return 当前帧在全局坐标系下的pose
 */
Eigen::Matrix4f lidarOdometry_node::matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    // 第一帧时
    if(!keyframe) {
        prev_trans.setIdentity();                  // 上一帧变换设置为单位阵
        predict_trans.setIdentity();               // 预测位姿
        keyframe_pose.setIdentity();               // 当前关键帧pose设为单位阵
        keyframe_stamp = stamp;
        keyframe = cloud;
        registration->setInputTarget(keyframe);    // 将关键帧设置为匹配对象
        return Eigen::Matrix4f::Identity();
    }
    /*
    // 匹配前去除畸变    
    pcl::PointCloud<PointT>::Ptr MatchCloud(new pcl::PointCloud<PointT>());
    for(auto&p:cloud->points)
    {
      // 只将近距离的点云参与匹配
      if((int)p.intensity<match_dis){
         MatchCloud->push_back(p);
      }
    }    */
    // 每次匹配都与参考关键帧进行匹配
    registration->setInputSource(cloud);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned, predict_trans);                              // 进行配准     predict_trans = setInputSource -> setInputTarget
    // 如果迭代没收敛   则该帧忽略  
    if(!registration->hasConverged()) {
        ROS_INFO("scan matching has not converged!!");
        cout<<"ignore this frame(" << stamp << ")"<<endl;
        return keyframe_pose * prev_trans;        // 返回上一帧位姿
    }
    else
    {
        float ndt_score = registration->getFitnessScore();   // 获得得分
        ROS_INFO_STREAM ( "front score: " << ndt_score);
        // 认为匹配不好 
        if(ndt_score>1)
        {
          //float resolution = 0.7*(ndt_score - 1) + 1.5;           // 根据匹配得分动态设置参数  
          //registration->setResolution(resolution);
          //ROS_INFO_STREAM("scan-scan resolution: "<<resolution);
        }
    }
    // 收敛的话   注意PCL中的T 是B -> W
    Eigen::Matrix4f trans = registration->getFinalTransformation();          // trans为当前帧->参考关键帧的变换矩阵
    // Twpi = Twpj * Tpjpi 
    Eigen::Matrix4f odom = keyframe_pose * trans;                            // 当前帧的全局pose    发布出去
    // Tb1w*Twb2 = Tb1b2
    Eigen::Matrix4f delta_motion = prev_trans.inverse()*trans;               // 前两帧位姿的增量   用于预测下一帧位姿
    // 判断是否重新设置关键帧
    double delta_trans = trans.block<3, 1>(0, 3).norm();         
    // 旋转矩阵对应 u*theta  对应四元数  e^u*theta/2  = [cos(theta/2), usin(theta/2)]
    Eigen::Quaternionf q_trans(trans.block<3, 3>(0, 0));
    q_trans.normalize();   
    double delta_angle = std::acos(q_trans.w())*2;     // 获得弧度    45度 约等于 0.8  
    double delta_time = (stamp - keyframe_stamp).toSec();
    // 满足关键帧条件
    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
        keyframe = cloud;                         //设置关键帧
        registration->setInputTarget(keyframe);   // 设定为匹配目标
        keyframe_pose = odom;
        keyframe_stamp = stamp;
        prev_trans.setIdentity();
        predict_trans = Eigen::Matrix4f::Identity()*delta_motion;
    }
    else
    {
        // 根据匀速运动假设进行预测    
        predict_trans = trans*delta_motion;         // Twb1*Tb1b2 = Twb2
        // 记录
        prev_trans = trans;   
    }
    // 返回当前帧的世界坐标 
    return odom;
}    

// scan -scan 匹配  
Eigen::Matrix4f lidarOdometry_node::matching_scan2scan(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    // 第一帧时
    if(frist) {
        odom.setIdentity();               
        prev_trans.setIdentity();
        registration->setInputTarget(cloud);     // 将关键帧设置为匹配对象
        frist = false;
        return Eigen::Matrix4f::Identity();
    }
    registration->setInputSource(cloud);
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned, prev_trans);                              // 进行配准 
    // 如果迭代没收敛   则该帧忽略  
    if(!registration->hasConverged()) {
        ROS_INFO("scan matching has not converged!!");
        odom = odom * prev_trans;
        return odom ;          // 假设是匀速运动 
    }
    else
    {
    float ndt_score = registration->getFitnessScore();   // 获得得分
    ROS_INFO_STREAM ( "front score: " << ndt_score);  
    if(ndt_score>1)
    {
        ROS_INFO("scan matching error!!");
        odom = odom * prev_trans;
        return odom ;          // 假设是匀速运动 
    }  
    }
    // 收敛的话   注意PCL中的T 是B -> W
    Eigen::Matrix4f trans = registration->getFinalTransformation();       // trans为前后两帧的
    ROS_INFO_STREAM("trans: "<<trans);
    // 检查
    double dx = trans.block<3, 1>(0, 3).norm();          // 平移量
    double da = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());    // 旋转矩阵->四元数  .w() = cos(theta/2)
    // 旋转平移过大就舍去
    if(dx > 50 || da > 5) {
        cout<<"too large transform!!  "<< dx << "[m] " << da << "[rad]"<<endl;
        cout<<"ignore this frame(" << stamp << ")"<<endl;
        odom = odom * prev_trans;
        return odom ;          // 假设是匀速运动 
    }
    // Twpi = Twpj * Tpjpi 
    odom = odom * trans;                                  // 当前帧的全局pose
    ROS_INFO_STREAM("odom: "<<odom);
    // 保存   用于下一次的预测   
    prev_trans = trans;   
    registration->setInputTarget(cloud);   // 设定为匹配目标
    // 返回当前帧的世界坐标 
    return odom;
} 

/**
 * @brief publish odometry
 * @param stamp  timestamp
 * @param pose   odometry pose to be published
 */
void lidarOdometry_node::publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // publish the transform                发布当前帧里程计到/odom话题                 
    nav_msgs::Odometry odom;                            
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;       // odom坐标    /odom
    odom.child_frame_id = base_frame_id;        // /lidar_odom
    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    // 旋转矩阵 -> 四元数
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    // 构造四元数   ROS信息
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();
    odom.pose.pose.orientation = odom_quat;  
    odom_pub.publish(odom);
    // 发布轨迹  
    geometry_msgs::PoseStamped laserPose;    
    laserPose.header = odom.header;
    laserPose.pose = odom.pose.pose;                // 和laserOdometry的pose相同  
    laserPath.header.stamp = odom.header.stamp;
    laserPath.poses.push_back(laserPose);
    laserPath.header.frame_id = odom_frame_id;      // odom坐标     /odom
    pubLaserPath.publish(laserPath);
}


void lidarOdometry_node::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if(!ros::ok()) {
        return;
    }
    // 转成PCL格式
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // 匹配
    TicToc t_pub;
    //const clock_t begin_time = clock() ;
    Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);
    //float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC ;
    //cout << "front matching time："<<seconds << endl ;
    printf("odometry time %f ms \n", t_pub.toc());
    // 发布话题     lidar_frame_id =   /lidar_odom
    publish_odometry(cloud_msg->header.stamp, lidar_frame_id, pose);
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "lidarOdometry_node");   
    ROS_INFO("Started lidarOdometry_node node");  
    lidarOdometry_node lidar_odometry;
    ros::spin(); 
    return 0;
}









