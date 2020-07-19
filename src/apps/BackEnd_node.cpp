
/**
 *  后端优化节点：  
 *   输入数据：   1、里程计数据    2、地面检测数据     3、GNSS数据    4、IMU数据   
 * 
 * 
 * */

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <thread>
#include <iostream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>     // pcl::fromROSMsg
#include <queue>

#include "ros_utils.hpp" 
#include "keyframe_updater.hpp" 
#include "keyframe.hpp"
#include "loop_detector.hpp"
#include "graph_slam.hpp"
#include "information_matrix_calculator.hpp"
#include "map_cloud_generator.hpp"
#include "nmea_sentence_parser.hpp"
#include "GNSSdata.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
//GPS
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Sentence.h>
// IMU
#include <sensor_msgs/Imu.h>


#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <ros_time_hash.hpp>


typedef pcl::PointXYZI PointT;  
using namespace std;

std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;

std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imu_sub;
std::unique_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> navsat_sub;
std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> Sync_odomCloud;
std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::NavSatFix>> Sync_GPSIMU;

ros::Subscriber floor_sub;
//ros::Subscriber odom_sub;
//ros::Subscriber cloud_sub;
// 可视化
ros::Publisher markers_pub;

std::string map_frame_id;
std::string odom_frame_id;

Eigen::Matrix4f trans_odom2map;
ros::Publisher odom2map_pub;

std::string points_topic;
ros::Publisher map_points_pub;

ros::Publisher pubGNSSPath;                // 发布轨迹
nav_msgs::Path GNSSPath;                    // 记录轨迹  
//tf::TransformListener tf_listener;

//ros::ServiceServer dump_service_server;
//ros::ServiceServer save_map_service_server;

// keyframe queue
std::string base_frame_id;

// 关键帧的处理队列
std::vector<KeyFrame::Ptr> keyframe_queue;

// floor_coeffs queue
double floor_edge_stddev;

//std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

// for map cloud generation
double map_cloud_resolution;

std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
//std::unique_ptr<MapCloudGenerator> map_cloud_generator;

// 局部优化每次处理的最大帧数  
int max_keyframes_per_update;
std::vector<KeyFrame::Ptr> new_keyframes;

g2o::VertexPlane* floor_plane_node;

std::vector<KeyFrame::Ptr> keyframes;                            // 保存关键帧信息
std::vector<Loop::Ptr> Loops;                                    // 保存回环检测   
std::vector<pair<Eigen::Vector3d,Eigen::Vector3d>> EdgeSE3;      // 保存连接节点的边  
std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

//std::unique_ptr<GraphSLAM> graph_slam;
//std::unique_ptr<LoopDetector> loop_detector;
std::unique_ptr<KeyframeUpdater> keyframe_updater;
std::unique_ptr<LoopDetector> loop_detector;
std::unique_ptr<InformationMatrixCalculator> inf_calclator;    // 计算信息矩阵  
// std::unique_ptr<GraphSLAM> graph_slam;                         // 后端优化  
std::unique_ptr<MapCloudGenerator> map_cloud_generator;
std::unique_ptr<NmeaSentenceParser> nmea_parser;               // nmea数据解析
//std::unique_ptr<InformationMatrixCalculator> inf_calclator;

std::mutex keyframe_queue_mutex;
std::mutex trans_odom2map_mutex;
std::mutex floor_coeffs_queue_mutex;
std::mutex keyframes_snapshot_mutex;
std::mutex main_thread_mutex;
std::mutex GNSS_queue_mutex;


//ros::WallTimer optimization_timer;
//ros::WallTimer map_publish_timer;

g2o::VertexSE3* anchor_node;
g2o::EdgeSE3* anchor_edge;

string fix_first_node_stddev;                       // 第一个节点的信息矩阵参数
string odometry_edge_robust_kernel;                 // 里程计边鲁棒核函数开关
double odometry_edge_robust_kernel_size;
string loop_closure_edge_robust_kernel;             // 闭环边的鲁棒核函数   
double loop_closure_edge_robust_kernel_size;  
bool fix_first_node_adaptive;
int num_iterations; 
double gps_time_offset;
string imu_orientation_edge_robust_kernel;
double imu_orientation_edge_robust_kernel_size;
double imu_orientation_edge_stddev;
double imu_time_offset;
Eigen::Vector3d Gnss_init_Map_loc = {0,0,0};     // GNSS初始化时  在地图上的坐标  
int init_imu_num = 1;     // 用于初始化Map系位姿的imu数量     
// GPS权重  
double gps_edge_stddev_xy;
double gps_edge_stddev_z;

// 数据队列
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2::ConstPtr> cloudBuf;
std::deque<GNSSData> GNSS_queue;

bool enable_imu_orientation;
bool gnss_origin_position_inited = false;
bool IMU_pose_inited = false;

std::mutex mBuf;
int KF_index = 0;
Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
Eigen::Quaterniond MAP2ENU = Eigen::Quaterniond::Identity();       // Map到ENU系的旋转
Eigen::Vector3d gravity_in_map = {0,0,0};  
Eigen::Quaterniond MAP2ENU2 = {0,0,0,0};       // Map到ENU系的旋转
Eigen::Matrix4d Tme = Eigen::Matrix4d::Identity();
Eigen::Isometry3d Tem = Eigen::Isometry3d::Identity();
Eigen::Matrix4d Til;



void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // ROS_INFO_STREAM("backend receive !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    const ros::Time& stamp = odom_msg->header.stamp;
    Eigen::Isometry3d odom = odom2isometry(odom_msg);                               // ROS ->Eigen
    // ROS_INFO_STREAM("receive odom cloud");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if(base_frame_id.empty()) {
        base_frame_id = cloud_msg->header.frame_id;
    }
    // 是否需要添加关键帧
    if(!keyframe_updater->update(odom)) {
        return;
    }
    // 通过点云与里程计和累计距离等来创建关键帧    
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, KF_index, cloud));
    // 线程锁开启
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);     // 加入处理队列
    KF_index++;
}

/*
void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
{

}

// 经过Map优化后   低频的里程计数据   
void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg)
{
   
}  
*/

void GNSS_callback(const sensor_msgs::ImuConstPtr& imu_msg, const sensor_msgs::NavSatFixConstPtr& navsat_msg)
{
    std::lock_guard<std::mutex> lock(GNSS_queue_mutex);
    GNSSData gnss_data;
    gnss_data.time = navsat_msg->header.stamp + ros::Duration(gps_time_offset);
    gnss_data.latitude = navsat_msg->latitude;
    gnss_data.longitude = navsat_msg->longitude;
    gnss_data.altitude = navsat_msg->altitude;
    gnss_data.status = navsat_msg->status.status;
    gnss_data.service = navsat_msg->status.service;
    // IMU 
    gnss_data.imu = imu_msg;
    // 原始数据直接存放
    GNSS_queue.push_back(gnss_data);
}

/**
 * @brief   处理关键帧队列中的数据   加入到图优化中      
 * @return if true, at least one keyframe was added to the pose graph
 * 在 optimization_timer_callback（）中调用
 */
bool flush_keyframe_queue() {
  if(keyframe_queue.empty()) {
    return false;
  }
  trans_odom2map_mutex.lock();
  Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
  trans_odom2map_mutex.unlock();
  int num_processed = 0;
  // 遍历全部关键帧队列   最多添加 max_keyframes_per_update 个关键帧  
  for(int i=0; i<std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
    num_processed = i;
    // 从keyframe_queue中取出关键帧
    const auto& keyframe = keyframe_queue[i];
    // odom系与map系对齐 
    keyframe->Pose = odom2map * keyframe->odom;  
    //  ROS_INFO_STREAM("keyframe->odom: "<<keyframe->odom.matrix());
    new_keyframes.push_back(keyframe);   
  }
  keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
  //  ROS_INFO_STREAM("surplus keyframe_queue: "<<keyframe_queue.size());
  return true;
}

//静态成员变量必须在类外初始化
bool GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian GNSSData::geo_converter;

bool flush_GNSS_queue()
{ 
  // 容器非空   
  if(new_keyframes.empty()||GNSS_queue.empty()) {
    return false;  
  }
  // ROS_INFO_STREAM("new_keyframes size: "<<new_keyframes.size()<<" GNSS_queue size: "<<GNSS_queue.size());
  if(GNSS_queue.back().time.toSec()<new_keyframes.front()->stamp.toSec())
  {
    // ROS_INFO_STREAM("GNSS_queue.back().time < new_keyframes.front() "<< GNSS_queue.back().time.toSec() - new_keyframes.front()->stamp.toSec());
    return false;
  }
  if(GNSS_queue.front().time.toSec()>new_keyframes.back()->stamp.toSec())
  {
    // ROS_INFO_STREAM("GNSS_queue.front().time > new_keyframes.back() "<<GNSS_queue.front().time.toSec() - new_keyframes.back()->stamp.toSec());
    return true;
  }
  geometry_msgs::PoseStamped Pose_lidar_in_map; 
  //ROS_INFO_STREAM("KEYFRAME SIZE: "<<new_keyframes.size()<<"gps num: "<<gps_queue.size());
  for(auto keyframe:new_keyframes)
  { 
    // ROS_INFO_STREAM(" time diff : "<< keyframe->stamp.toSec() - gps_queue.front().time);
    // 将gps_queue距离keyframe距离太远的舍弃 
    while(keyframe->stamp.toSec() - GNSS_queue.front().time.toSec() > 0.11)
    {
      //  ROS_INFO_STREAM("keyframe - gps > 0.1 : "<< keyframe->stamp.toSec() - GNSS_queue.front().time.toSec());
      GNSS_queue.pop_front();   // 头数据丢弃 
      if(GNSS_queue.empty())  return true;
    }
    // gps比keyframe 晚很多 
    if(keyframe->stamp.toSec() - GNSS_queue.front().time.toSec() < -0.01)
    {
       // ROS_INFO_STREAM("keyframe - gps < -0.01" << keyframe->stamp.toSec() - GNSS_queue.front().time.toSec());
       continue;
    }
    // 当前数据合格  
    GNSSData gnss_data = GNSS_queue.front();
    GNSS_queue.pop_front();
    
    // 判断是否需要插值   
    if(keyframe->stamp.toSec() - gnss_data.time.toSec() > 0.001)
    {
       GNSSData gnss_data_2 = GNSS_queue.front();      // 取下一个  
      
       double t1 = keyframe->stamp.toSec() - gnss_data.time.toSec();
       double t2 = gnss_data_2.time.toSec() - keyframe->stamp.toSec();
       if(t1+t2 > 0.15)   // 有误
       {
         ROS_INFO_STREAM("t1+t2 > 0.15"<<t1+t2);
         continue;
       }
       // 计算插值系数     
      double front_scale = t2 / (t2+t1);
      double back_scale = t1 / (t2+t1);
      // 对GPS进行插值  
      gnss_data.longitude = gnss_data_2.longitude*back_scale + gnss_data.longitude*front_scale;
      gnss_data.latitude = gnss_data_2.latitude*back_scale + gnss_data.latitude*front_scale;
      gnss_data.altitude = gnss_data_2.altitude*back_scale + gnss_data.altitude*front_scale;  
      // 对IMU的姿态进行插值  
      Eigen::Quaterniond imu_front,imu_back;
      imu_front.w() = gnss_data.imu->orientation.w;
      imu_front.x() = gnss_data.imu->orientation.x;
      imu_front.y() = gnss_data.imu->orientation.y;
      imu_front.z() = gnss_data.imu->orientation.z;
      imu_back.w() = gnss_data_2.imu->orientation.w;
      imu_back.x() = gnss_data_2.imu->orientation.x;
      imu_back.y() = gnss_data_2.imu->orientation.y;
      imu_back.z() = gnss_data_2.imu->orientation.z;
      keyframe->orientation = imu_front.slerp(back_scale, imu_back);       // 球面插值  
      // 直接线性插值 
      /*()
      keyframe->orientation.w() = gnss_data.imu->orientation.w*front_scale + gnss_data_2.imu->orientation.w*back_scale;
      keyframe->orientation.x() = gnss_data.imu->orientation.x*front_scale + gnss_data_2.imu->orientation.x*back_scale;
      keyframe->orientation.y() = gnss_data.imu->orientation.y*front_scale + gnss_data_2.imu->orientation.y*back_scale;
      keyframe->orientation.z() = gnss_data.imu->orientation.z*front_scale + gnss_data_2.imu->orientation.z*back_scale;
      */
    }
    else
    {
      keyframe->orientation.w() = gnss_data.imu->orientation.w;
      keyframe->orientation.x() = gnss_data.imu->orientation.x;
      keyframe->orientation.y() = gnss_data.imu->orientation.y;
      keyframe->orientation.z() = gnss_data.imu->orientation.z; 
    }
    
    // GNSS是否初始化了
    if (!gnss_origin_position_inited) {
      gnss_data.InitOriginPosition();
      /*
      // Tel*Tli    静态初始化配套
      Eigen::Matrix4d Tei = keyframe->Pose.matrix() * Til.inverse();
      Gnss_init_Map_loc = Tei.block<3,1>(0,3);
      */
      // Rel = Rei * Ril
      Eigen::Quaterniond Rel = keyframe->orientation * Eigen::Quaterniond(Til.block<3,3>(0,0));
      // Rem = Rel * Rml^-1
      MAP2ENU =  Rel * keyframe->Pose.matrix().block<3,3>(0,0).inverse();
      Tem.rotate(MAP2ENU);
      Tem.pretranslate(Eigen::Vector3d(0,0,0));
      gnss_origin_position_inited = true;
      // 计算当前ENU坐标系坐标原点相对与原点ENU坐标系的位置
      Eigen::Matrix4d Tmi = keyframe->Pose.matrix() * Til.matrix().inverse();     // Tmi = Tml * Tli
      Gnss_init_Map_loc = MAP2ENU * Tmi.block<3,1>(0,3);
      //ROS_INFO_STREAM("GNSS init OK !  Map loc: "<<Gnss_init_Map_loc);
      //ROS_INFO_STREAM("lidar pos:  "<< keyframe->Pose.matrix());
      // 更新keyframe的Pose
      for(auto& keyframe:keyframes)
        keyframe->Pose = Tem * keyframe->Pose;
      for(auto& keyframe:new_keyframes)
        keyframe->Pose = Tem * keyframe->Pose;
    }
    gnss_data.UpdateXYZ();       // 计算ENU坐标     
    // 将IMU的ENU坐标转到原点  Pei
    keyframe->utm_coord.x() = gnss_data.local_E + Gnss_init_Map_loc.x();
    keyframe->utm_coord.y() = gnss_data.local_N + Gnss_init_Map_loc.y();
    keyframe->utm_coord.z() = gnss_data.local_U + Gnss_init_Map_loc.z();  
    // 然后求出Lidar的UTM坐标  tel = Rei*til + tei  
    keyframe->utm_coord =  keyframe->orientation * Til.block<3,1>(0,3) + keyframe->utm_coord;
    //ROS_INFO_STREAM("keyframe id: "<<keyframe->id<<" gnss E:"<<keyframe->utm_coord[0]<<"gnss N:"<<keyframe->utm_coord[1]<<"gnss U:"<<keyframe->utm_coord[2]);
    // ROS_INFO_STREAM("gnss E:"<<keyframe->utm_coord[0]<<"gnss N:"<<keyframe->utm_coord[1]<<"gnss U:"<<keyframe->utm_coord[2]);
    Pose_lidar_in_map.header.stamp = gnss_data.time;
    keyframe->GNSS_Valid = true;
    // 发布GNSS位姿 
    // 获得的数据是IMU在ENU下的坐标   现在要用GNSS结果反推Map系中Lidar的位姿
    if(gnss_origin_position_inited&&IMU_pose_inited)
    {
      Eigen::Quaterniond Qml = Eigen::Quaterniond::Identity();
      Eigen::Vector3d tml = Eigen::Vector3d(0,0,0);
      tml = keyframe->utm_coord;
      Qml = keyframe->orientation;
      Pose_lidar_in_map.pose.position.x = tml.x();
      Pose_lidar_in_map.pose.position.y = tml.y();
      Pose_lidar_in_map.pose.position.z = tml.z();
      Pose_lidar_in_map.pose.orientation.w = Qml.w();
      Pose_lidar_in_map.pose.orientation.x = Qml.x();
      Pose_lidar_in_map.pose.orientation.y = Qml.y();
      Pose_lidar_in_map.pose.orientation.z = Qml.z();
      GNSSPath.header.stamp = Pose_lidar_in_map.header.stamp;
      GNSSPath.poses.push_back(Pose_lidar_in_map);
    }
    GNSSPath.header.frame_id = map_frame_id;      // odom坐标 
    pubGNSSPath.publish(GNSSPath);
  }   
  return true;
}


bool Parse_data()
{
  std::lock_guard<std::mutex> lock1(keyframe_queue_mutex);
  std::lock_guard<std::mutex> lock2(GNSS_queue_mutex);
  // 还未初始化  
  // 静态初始化   静止放置  等待初始化完成  
  if(!IMU_pose_inited)
  {
    static int index = 0;
    if(!GNSS_queue.empty())   
    {
      for(int i = 0; i<GNSS_queue.size(); i++)
      {  /* // 静态初始化    不适合kitti数据集  
         // 法一    imu侧量的 orientation 应该是 IMU -> ENU 系的旋转   Map系是建立在Lidar上的   所以要求Lidar->ENU系的旋转  即  Rei*Ril
         MAP2ENU.w() += GNSS_queue[i].imu->orientation.w;
         MAP2ENU.x() += GNSS_queue[i].imu->orientation.x;
         MAP2ENU.y() += GNSS_queue[i].imu->orientation.y;
         MAP2ENU.z() += GNSS_queue[i].imu->orientation.z;
         // 法二
         gravity_in_map.x() += GNSS_queue[i].imu->linear_acceleration.x;
         gravity_in_map.y() += GNSS_queue[i].imu->linear_acceleration.y;
         gravity_in_map.z() += GNSS_queue[i].imu->linear_acceleration.z;
        
         index++;
         if(index>=init_imu_num)
         {
           // 此时应该是  MAP2ENU = Rei
           MAP2ENU.w() /= index;
           MAP2ENU.x() /= index;
           MAP2ENU.y() /= index;
           MAP2ENU.z() /= index;
           // Rem = Rei*Ril
           MAP2ENU = MAP2ENU * Eigen::Quaterniond(Til.block<3,3>(0,0));
           MAP2ENU.normalize();
           Tem.rotate(MAP2ENU);  
           Tem.pretranslate(Eigen::Vector3d(0,0,0));
           ROS_INFO_STREAM("MAP init ok! Tem: "<< Tem.matrix());
           // 法二   
           // 此时应该是  gi   
           gravity_in_map.x() /= index;
           gravity_in_map.y() /= index;
           gravity_in_map.z() /= index;
           // gl = Rli*gi
           gravity_in_map = Til.block<3,3>(0,0).transpose()*gravity_in_map;

           Eigen::Vector3d g = {0,0,1};
           Eigen::Vector3d n = g.cross(gravity_in_map);
           double theta = atan2(n.norm(), g.dot(gravity_in_map));
           n.normalize();       // 单位化
           Eigen::AngleAxisd rotation_vector (theta, n);  
           MAP2ENU2 = Eigen::Quaterniond(rotation_vector);
           MAP2ENU2.normalize();
           IMU_pose_inited = true;
             //           ROS_INFO_STREAM("MAP init ok! res1: "<< MAP2ENU.coeffs()<<" res2: "<<MAP2ENU2.coeffs() );
           break;
           
         }*/
         IMU_pose_inited = true;
      }
    }
  }   
  // 提取需要优化的关键帧   
  bool keyframe_updated = flush_keyframe_queue();  
  if(!flush_GNSS_queue())      // 与keyframe 匹配  
     GNSS_queue.clear();

  if(!keyframe_updated)  return false;
  // ROS_INFO_STREAM("GNSS_queue SIZE: "<<GNSS_queue.size());
  return true;
}


/**
 * @brief 创建可视化队列
 * @param stamp
 * @return
 */
visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(4);

  // node markers    位姿节点
  visualization_msgs::Marker& traj_marker = markers.markers[0];
  traj_marker.header.frame_id = "map";
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "nodes";
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;
  
  // 数量
  traj_marker.points.resize(keyframes.size());
  // 颜色
  traj_marker.colors.resize(keyframes.size());
  
  for(int i=0; i<keyframes.size(); i++) {
    // 设置位置
    Eigen::Vector3d pos = keyframes[i]->Pose.translation();
    traj_marker.points[i].x = pos.x();
    traj_marker.points[i].y = pos.y();
    traj_marker.points[i].z = pos.z();
    // 颜色
    double p = static_cast<double>(i) / keyframes.size();
    traj_marker.colors[i].r = 1.0 - p;
    traj_marker.colors[i].g = p;
    traj_marker.colors[i].b = 0.0;
    traj_marker.colors[i].a = 1.0;
  }   
  
  // edge markers  边
  visualization_msgs::Marker& edge_marker = markers.markers[2];
  edge_marker.header.frame_id = "map";
  edge_marker.header.stamp = stamp;
  edge_marker.ns = "edges";
  edge_marker.id = 2;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;

  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.05;
  // 这里要注意 ！！！！！！！！！！！！！！！！！
  edge_marker.points.resize(keyframes.size() * 2 * 2 + Loops.size() * 2);
  edge_marker.colors.resize(keyframes.size() * 2 * 2 + Loops.size() * 2);
  int i=0;
  for(int num = 0; num<keyframes.size(); num++) {
    // 里程计边    Pc
    Eigen::Vector3d pt1 = keyframes[num]->Pose.translation();
    //  Twc*Tlc^-1 = Twl
    Eigen::Vector3d pt2 = (keyframes[num]->Pose*keyframes[num]->deltaPose.inverse()).translation();
    // 设置位置关系     每个frame 2个点 
    edge_marker.points[i*2].x = pt1.x();
    edge_marker.points[i*2].y = pt1.y();
    edge_marker.points[i*2].z = pt1.z();
    edge_marker.points[i*2 + 1].x = pt2.x();
    edge_marker.points[i*2 + 1].y = pt2.y();
    edge_marker.points[i*2 + 1].z = pt2.z();

    edge_marker.colors[i*2].r = 1.0 ;
    edge_marker.colors[i*2].g = 2.0;
    edge_marker.colors[i*2].a = 1.0;
    edge_marker.colors[i*2 + 1].r = 1.0 ;
    edge_marker.colors[i*2 + 1].g = 2.0;
    edge_marker.colors[i*2 + 1].a = 1.0;
    i++;

    // GNSS 先验边     2个点  
    if(keyframes[num]->GNSS_Valid) {
      Eigen::Vector3d pt1 = keyframes[num]->Pose.translation();  
      Eigen::Vector3d pt2 = keyframes[num]->utm_coord;

      edge_marker.points[i*2].x = pt1.x();
      edge_marker.points[i*2].y = pt1.y();
      edge_marker.points[i*2].z = pt1.z()+1;
      edge_marker.points[i*2 + 1].x = pt2.x();
      edge_marker.points[i*2 + 1].y = pt2.y();
      edge_marker.points[i*2 + 1].z = pt2.z();

      edge_marker.colors[i*2].r = 1.0;
      edge_marker.colors[i*2].a = 1.0;
      edge_marker.colors[i*2 + 1].r = 1.0;
      edge_marker.colors[i*2 + 1].a = 1.0;
      i++;
    }
    // IMU 先验 Pose 边  
    
  }
  // 回环检测的边   2个点 
  for(auto& loop:Loops)
  { 
    Eigen::Vector3d pt1 = loop->key1->Pose.translation();    // 新帧  
    Eigen::Vector3d pt2 = (loop->key1->Pose.matrix()*loop->relative_pose.inverse().cast<double>()).block<3,1>(0,3);     // 与新帧闭环的老帧   Twc * Tlc^-1

    edge_marker.points[i*2].x = pt1.x();
    edge_marker.points[i*2].y = pt1.y();
    edge_marker.points[i*2].z = pt1.z();
    edge_marker.points[i*2 + 1].x = pt2.x();
    edge_marker.points[i*2 + 1].y = pt2.y();
    edge_marker.points[i*2 + 1].z = pt2.z();

    edge_marker.colors[i*2].r = 2.0;
    edge_marker.colors[i*2].a = 2.0;
    edge_marker.colors[i*2 + 1].r = 2.0;
    edge_marker.colors[i*2 + 1].a = 2.0;
    i++;
  }
  
  // sphere
  visualization_msgs::Marker& sphere_marker = markers.markers[3];
  sphere_marker.header.frame_id = "map";
  sphere_marker.header.stamp = stamp;
  sphere_marker.ns = "loop_close_radius";
  sphere_marker.id = 3;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;

  if(!keyframes.empty()) {
    Eigen::Vector3d pos = keyframes.back()->Pose.translation();
    sphere_marker.pose.position.x = pos.x();
    sphere_marker.pose.position.y = pos.y();
    sphere_marker.pose.position.z = pos.z();
  }
  sphere_marker.pose.orientation.w = 1.0;
  sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

  sphere_marker.color.r = 1.0;
  sphere_marker.color.a = 0.3;

  return markers;
}


// 局部优化      里程计约束+地面约束+GNSS约束   
void local_optimize()
{
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,-1>> Block;
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int bias=0;
  if(keyframes.empty())
    bias = 2;
  else
    bias = 3;

  // 记录本次局部优化中  节点与边的序号   
  int vertexCnt=0, edgeCnt=0;
  ROS_INFO_STREAM("Local optimation!");
  vector<g2o::VertexSE3*> vertexs;         
  Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
  for(const auto& keyframe:new_keyframes){      
    // 添加节点  
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(vertexCnt++);
    // ROS_INFO_STREAM("setEstimate() : "<<keyframe->odom.matrix());
    v->setEstimate(keyframe->Pose);
    optimizer.addVertex(v);
    vertexs.push_back(v);

    // 添加先验的边    每一条GNSS的边  残差维度 3   单边   只与一个节点状态有关  Jt(6*3)*W(3*3)*J(3*6)  
    if(keyframe->GNSS_Valid)
    {
      g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
      edge->setMeasurement(keyframe->utm_coord);      // 设置观测先验  
      edge->vertices()[0] = v;
      // 信息矩阵     3*3      JtWJ  
      Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
      information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;     // 20
      information_matrix(2, 2) /= gps_edge_stddev_z;                  // 5  
      edge->setInformation(information_matrix);
      optimizer.addEdge(edge);
    //  optimizer.add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));
    }

    if(vertexCnt==1)
    { // 历史上第一个节点  
      if(keyframes.empty())
      {
        v->setFixed(true);
        keyframe->deltaPose = Eigen::Isometry3d::Identity();
      }
      else{    // 本次优化的第一个节点  
        // 创建一个固定的原点节点
        g2o::VertexSE3* org = new g2o::VertexSE3();
        org->setId(vertexCnt++);    
        org->setEstimate(keyframes.back()->Pose);
        org->setFixed(true);
        optimizer.addVertex(org);

        // 激光里程计的边  
        g2o::EdgeSE3* edge(new g2o::EdgeSE3());
        edge->setId(edgeCnt++);
        edge->setVertex(0, org);
        edge->setVertex(1, v);
        // Tlw * Twc = Tlc   
        relative_pose = org->estimate().matrix().inverse() * v->estimate().matrix();    
        keyframe->deltaPose = relative_pose;
        edge->setMeasurement(relative_pose);
        if(!keyframes.empty())
        {
          // 计算信息矩阵    通过kdtree检查点云通过变换后的匹配程度反映里程计是否准确   匹配程度越高  则信息矩阵各权重越大   则优化时  会更靠近里程计的结果   
          Eigen::MatrixXd information = inf_calclator->calc_information_matrix( keyframes.back()->cloud, keyframe->cloud, relative_pose);
          edge->setInformation(information);
        }
        optimizer.addEdge(edge);
      }
      continue;
    }
    // 激光里程计的边  
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setId(edgeCnt++);
    edge->setVertex(0, vertexs[vertexCnt-bias]);
    edge->setVertex(1, vertexs[vertexCnt-bias+1]);
    // add edge between consecutive keyframes
    const auto& prev_keyframe = new_keyframes[vertexCnt-bias];                        // 获取上一关键帧   
    // 计算相对位移        Tlw * Twc = Tlc 
    relative_pose = prev_keyframe->Pose.matrix().inverse()*keyframe->Pose.matrix();
    keyframe->deltaPose = relative_pose;
    //  ROS_INFO_STREAM("edge : "<<relative_pose.matrix());
    edge->setMeasurement(relative_pose);
    // 计算信息矩阵    通过kdtree检查点云通过变换后的匹配程度反映里程计是否准确   匹配程度越高  则信息矩阵各权重越大   则优化时  会更靠近里程计的结果   
    // prev_keyframe->cloud = Tlc * keyframe->cloud
    Eigen::MatrixXd information = inf_calclator->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud,  relative_pose);
    edge->setInformation(information);
    optimizer.addEdge(edge);
  } 
  // optimize the pose graph
  // 执行优化
  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << optimizer.vertices().size() << "   edges: " << optimizer.edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  std::cout << "init" << std::endl;
  optimizer.initializeOptimization(0);
  optimizer.setVerbose(true);

  std::cout << "chi2" << std::endl;
  double chi2 = optimizer.chi2();

  std::cout << "optimize!!" << std::endl;
  auto t1 = ros::WallTime::now();
  optimizer.optimize(30);

  auto t2 = ros::WallTime::now();
  std::cout << "done" << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer.chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

  for(int i=0; i<new_keyframes.size(); i++)
  {
    new_keyframes[i]->Pose = vertexs[i]->estimate();    //  获取优化结果
    //  ROS_INFO_STREAM("i: "<<i<<"after optimize: "<<new_keyframes[i]->Pose.matrix().cast<float>()); 
  }
}

// 局部优化      里程计约束+地面约束+GNSS约束  
void global_optimize(const Loop::Ptr& loop)
{
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,-1>> Block;
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // 本次回环连接的关键帧id  
  int id_min = loop->key2->id;
  vector<Loop::Ptr> add_loop; 
  map<int, g2o::VertexSE3*> loopInfo;        // 记录回环的每个keyfram 的 id与生成的节点  
  add_loop.push_back(loop);
  loopInfo[loop->key1->id] = NULL;
  loopInfo[loop->key2->id] = NULL;
  // 检查是否需要添加之前的回环约束     更新回环的最早节点id   
  for(int i = Loops.size()-1; i>=0; i--)
  {  // 该回环需要添加  
     if(Loops[i]->key1->id >= id_min)
     {
       add_loop.push_back(Loops[i]);
       if(Loops[i]->key2->id < id_min)
         id_min = Loops[i]->key2->id;
       // 记录该回环的两个帧的id
       loopInfo[Loops[i]->key1->id] = NULL;
       loopInfo[Loops[i]->key2->id] = NULL;
     }  
     else
       break;
  }
  int loop_size = add_loop.size();
  ROS_INFO_STREAM("********** global optimation!  loops num: "<<loop_size);

  int vertexCnt=0, edgeCnt=0, loop_vertex_num=0, frist_kf_index=0;
  vector<g2o::VertexSE3*> vertexs;    // 保存当前优化器中的节点  
  
  KeyFrame::Ptr pre_keyframe = NULL;   
  for(const auto& keyframe:keyframes){  
      // 只将闭环内的节点添加  
      if(keyframe->id<id_min)  
      {
        frist_kf_index++;
        continue;
      }
      // 添加节点  
      g2o::VertexSE3* v = new g2o::VertexSE3();
      v->setId(vertexCnt++);
      v->setEstimate(keyframe->Pose);
      if(vertexCnt==1)
        v->setFixed(true);
      optimizer.addVertex(v);
      vertexs.push_back(v);
      // 查找该节点是否属于回环  
      if(loop_vertex_num<loop_size*2&&loopInfo.find(keyframe->id)!=loopInfo.end())
      {
          loopInfo[keyframe->id] = v;      // 覆盖掉原结果  
          loop_vertex_num++;
      }
      if(vertexCnt==1) {
        pre_keyframe = keyframe;
        continue;
      }
      
      // 添加边  
      g2o::EdgeSE3* edge(new g2o::EdgeSE3());
      edge->setId(edgeCnt++);
      edge->setVertex(0, optimizer.vertices()[vertexCnt-2]);            // 老一帧  
      edge->setVertex(1, optimizer.vertices()[vertexCnt-1]);
      
      // 计算两节点间的相对位移         T t * delta = Tt-1
      Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
      relative_pose = pre_keyframe->Pose.matrix().inverse()*keyframe->Pose.matrix();
      //  ROS_INFO_STREAM("edge : "<<relative_pose.matrix());
      edge->setMeasurement(relative_pose);
      // 计算信息矩阵
      //Eigen::MatrixXd information = inf_calclator->calc_information_matrix(pre_keyframe->cloud, keyframe->cloud,  relative_pose);
      // 固定信息矩阵  
      Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
      inf.topLeftCorner(3, 3).array() /= 20;
      inf.bottomRightCorner(3, 3).array() /= 5;
      edge->setInformation(inf);
      optimizer.addEdge(edge);
      /*
      // 添加该keyframe 的GNSS先验
      if(keyframe->GNSS_Valid)
      { 
        g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
        edge->setMeasurement(keyframe->utm_coord);      // 设置观测先验  
        edge->vertices()[0] = v;
        // 信息矩阵  
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;     // 20
        information_matrix(2, 2) /= gps_edge_stddev_z;                  // 5  
        edge->setInformation(information_matrix);
        optimizer.addEdge(edge);
      //  optimizer.add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));
      } */
      // 添加地面约束

      // 添加imu Pose 约束  
      
      pre_keyframe = keyframe;
  }

  for(auto& l:add_loop) {
    // 添加回环边
    if(!loopInfo[l->key1->id]||!loopInfo[l->key2->id])  continue;   
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setId(edgeCnt++);
    edge->setVertex(0, loopInfo[l->key2->id]);     // 相对早一点的
    edge->setVertex(1, loopInfo[l->key1->id]);     // 相对晚一点的
    Eigen::Isometry3d relpose(l->relative_pose.cast<double>());    // relpose :  curr -> last  
    edge->setMeasurement(relpose);
    // 计算信息矩阵    维度与优化状态的维度相当   
    Eigen::MatrixXd information = inf_calclator->calc_information_matrix(l->key1->cloud, l->key2->cloud, relpose);
    ROS_INFO_STREAM("LOOP inf: "<<information.matrix());
    edge->setInformation(information);
    optimizer.addEdge(edge);
  }
  // optimize the pose graph
  // 执行优化
  std::cout << std::endl;
  std::cout << "--- global optimization ---" << std::endl;
  std::cout << "nodes: " << optimizer.vertices().size() << "   edges: " << optimizer.edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  std::cout << "init" << std::endl;
  optimizer.initializeOptimization(0);
  optimizer.setVerbose(true);

  std::cout << "chi2" << std::endl;
  double chi2 = optimizer.chi2();

  std::cout << "optimize!!" << std::endl;
  auto t1 = ros::WallTime::now();
  int iterations = optimizer.optimize(30);

  auto t2 = ros::WallTime::now();
  std::cout << "done" << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer.chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
  
  for(int i=0; i<vertexs.size(); i++)
  {
     keyframes[frist_kf_index+i]->Pose = vertexs[i]->estimate();    //  获取优化结果
  }
 
}

/**
 * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
 * @param event
 */
//void optimization_timer_callback(const ros::WallTimerEvent& event) {
void optimization_timer() {
  std::lock_guard<std::mutex> lock(main_thread_mutex);
  if(!Parse_data())  return;
  // 执行局部优化
  local_optimize();
  // loop detection
  Loop::Ptr loop = loop_detector->detect(keyframes, new_keyframes);
  std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
  new_keyframes.clear();
  if(loop)
  {
    // 执行全局优化
    global_optimize(loop);
    Loops.push_back(loop);
  }

  // 求odom -> map 的变换矩阵  
  const auto& keyframe = keyframes.back();
  trans_odom2map_mutex.lock();
  trans_odom2map = keyframe->Pose.matrix().cast<float>() * keyframe->odom.matrix().inverse().cast<float>();
  trans_odom2map_mutex.unlock();
  //  ROS_INFO_STREAM("node->estimate() : "<<keyframe->Pose.matrix().cast<float>());
  //  ROS_INFO_STREAM("keyframe->odom : "<<keyframe->odom.matrix().cast<float>());
  //  ROS_INFO_STREAM("odom inverse : "<<keyframe->odom.matrix().inverse().cast<float>());
  //  ROS_INFO_STREAM("odom to map trans : "<<trans_odom2map);

  // KeyFrameSnapshot 用于建图的关键帧节点  只有位姿与点云信息  
  std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
  std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(),
    [=](const KeyFrame::Ptr& k) {     
      return std::make_shared<KeyFrameSnapshot>(k);     // 用 KeyFrame 指针 k 构造  KeyFrameSnapshot 
  });
  keyframes_snapshot_mutex.lock();
  keyframes_snapshot.swap(snapshot);
  keyframes_snapshot_mutex.unlock();

  // 如果有订阅者
  if(odom2map_pub.getNumSubscribers()) {
    // 构造 ROS Msg
    geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans_odom2map.cast<float>(), map_frame_id, odom_frame_id);
    odom2map_pub.publish(ts);
  }
  // 可视化 
  if(markers_pub.getNumSubscribers()) {
    auto markers = create_marker_array(ros::Time::now());
    markers_pub.publish(markers);
  }  
}

/**
 * @brief generate map point cloud and publish it
 * @param event
 */
//void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
void map_points_publish() {  
  if(!map_points_pub.getNumSubscribers()) {
    return;
  }

  std::vector<KeyFrameSnapshot::Ptr> snapshot;

  keyframes_snapshot_mutex.lock();
  snapshot = keyframes_snapshot;
  keyframes_snapshot_mutex.unlock();

  auto cloud = map_cloud_generator->generate(snapshot, 0.05);
  if(!cloud) {
    return;
  }

  cloud->header.frame_id = map_frame_id;
  cloud->header.stamp = snapshot.back()->cloud->header.stamp;

  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud, *cloud_msg);
  
  map_points_pub.publish(cloud_msg);
}

// ROS 通信初始化 
void comm_init(ros::NodeHandle nh)
{
    // publishers
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);        // 可视化
    odom2map_pub = nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);      // odom到map的校正  
    map_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1);          // 发布地图  可视化   
    pubGNSSPath = nh.advertise<nav_msgs::Path>("/GNSSpath", 100);
    // subscrible      
    // IMU的订阅                                                                                             // 点云的回调函数  
    //imu_sub = nh.subscribe("/kitti/oxts/imu", 1024, imu_callback);
    // 地面检测的订阅
    //floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024,  floor_coeffs_callback);
    
    
    //  navsat_sub = nh.subscribe("/kitti/oxts/gps/fix", 1024, navsat_callback);
    // subscribers

    
    // 点云和里程计数据的订阅  并且进行同步处理  
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom_opt_low", 1000));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/key_cloud", 1000));
    Sync_odomCloud.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(*odom_sub, *cloud_sub, 1000));          
    Sync_odomCloud->registerCallback(boost::bind(&cloud_callback, _1, _2)); 
    
    /*
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_opt_low", 1000,odom_callback);
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/key_cloud", 1000,cloud_callback);
    */
    // 同步订阅 GNSS的数据   包括IMU与GPS
    // GPS订阅
    if(nh.param<bool>("enable_GNSS", true)) {
    imu_sub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/kitti/oxts/imu", 1000));
    navsat_sub.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, "/kitti/oxts/gps/fix", 1000));
    Sync_GPSIMU.reset(new message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::NavSatFix>(*imu_sub, *navsat_sub, 1000));          
    Sync_GPSIMU->registerCallback(boost::bind(&GNSS_callback, _1, _2));   
    }

    // 服务
    //dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    //save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);
}

void params_init(ros::NodeHandle nh)
{
    keyframe_updater.reset(new KeyframeUpdater(nh));
    loop_detector.reset(new LoopDetector(nh));
  //  graph_slam.reset(new GraphSLAM(nh.param<std::string>("g2o_solver_type", "lm_var")));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(nh));

    // 按 graph_update_interval 时间间隔定时执行optimization_timer_callback
  //  optimization_timer = nh.createWallTimer(ros::WallDuration(graph_update_interval), optimization_timer_callback);
  //  map_publish_timer = nh.createWallTimer(ros::WallDuration(10), map_points_publish_timer_callback);
    odometry_edge_robust_kernel = nh.param<std::string>("odometry_edge_robust_kernel", "NONE");
    odometry_edge_robust_kernel_size = 1.0;
    loop_closure_edge_robust_kernel_size = 1.0;
    fix_first_node_adaptive = true;
    num_iterations = 50;                  // 迭代次数
    fix_first_node_stddev = "10 10 10 1 1 1";
    max_keyframes_per_update = 20;
    odom_frame_id = nh.param<std::string>("odom_frame_id", "odom");
    map_frame_id = nh.param<std::string>("map_frame_id", "map");
    trans_odom2map.setIdentity();              // 里程计到map坐标的转换  
    gps_time_offset = 0;
    imu_orientation_edge_stddev = 1;           // IMU的信息矩阵的权重  
    imu_orientation_edge_robust_kernel = nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE");
    imu_orientation_edge_robust_kernel_size = nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0);
    enable_imu_orientation = nh.param<bool>("enable_imu_orientation", false);
    imu_time_offset = nh.param<double>("imu_time_offset", 0.0);
    Til <<  0.999998 ,   -0.000785403 ,  0.00202441  ,   0.810544,
            0.000755307 ,     0.99989 ,   0.0148245,    -0.307054,
            -0.00203583 ,   -0.014823  ,   0.999888  ,   0.802724,
            0      ,      0       ,     0   ,         1;

    gps_edge_stddev_xy = 25;
    gps_edge_stddev_z = 5;
}


ros::Time Optimize_previous_time = ros::Time(0);
ros::Time Map_updata_previous_time = ros::Time(0);
#define Optimize_duration 3
#define Map_updata_duration 10  

void process()
{
   while(true){
    
      if(Optimize_previous_time.toSec() == 0 && Map_updata_previous_time.toSec() == 0)
      {
        Optimize_previous_time = ros::Time::now();
        Map_updata_previous_time = ros::Time::now();
      }
      else
      {
        double Optimize_diff_time = (ros::Time::now() - Optimize_previous_time).toSec();     // 计算时间差
        double Map_updata_diff_time = (ros::Time::now() - Map_updata_previous_time).toSec();     // 计算时间差
        /*
        // 完成初始化   计算 lidar - 地理坐标系的外参
        if(!System_init)
        {
            // 读取 IMU - lidar的外参   
            try {
            static tf::TransformListener listener_;
            static tf::StampedTransform transform;
            listener_.lookupTransform("/imu_link", "velo_link", ros::Time(0), transform);
            Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
        
            double roll, pitch, yaw;
            tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
            Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

            // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
            lidar_to_imu = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
            ROS_INFO_STREAM("calibration: "<<lidar_to_imu);
            System_init = true;
            } catch (tf::TransformException &ex) {
            ROS_INFO_STREAM("System init error " );
            }   
        } */
        // 图优化  
        if(Optimize_diff_time>= Optimize_duration)
        { 
          Optimize_previous_time = ros::Time::now();
          ROS_INFO_STREAM("Graph optimization!");
          optimization_timer();
        }
        // 地图更新
        if(Map_updata_diff_time>= Map_updata_duration)
        {
          Map_updata_previous_time = ros::Time::now();
          ROS_INFO_STREAM("updata Map!");
          map_points_publish();
        }   
      }
   }
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "BackEnd_node");   
    ROS_INFO("Started BackEnd_node node");    
    ros::NodeHandle nh("~");  
    comm_init(nh);
    params_init(nh);
    std::thread measurement_process{process}; 
    ros::spin(); 
    return 0;
}

























