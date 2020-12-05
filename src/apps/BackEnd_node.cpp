
/**
 *  后端优化节点：  
 *   输入数据：   1、里程计数据    2、地面约束数据     3、GNSS数据    4、IMU数据   
 **/

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
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_so3_prior.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
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
// odom坐标系到map坐标系的变换  
Eigen::Matrix4f trans_odom2map;
ros::Publisher odom2map_pub;

std::string points_topic;
ros::Publisher map_points_pub;

ros::Publisher pubGNSSPath;                 // 发布轨迹
nav_msgs::Path GNSSPath;                    // 记录轨迹  
//tf::TransformListener tf_listener;

//ros::ServiceServer dump_service_server;
//ros::ServiceServer save_map_service_server;

// keyframe queue
std::string base_frame_id;

// 新添加的关键帧的处理队列
std::vector<KeyFrame::Ptr> new_keyframe_queue;

// floor_coeffs queue
double floor_edge_stddev;

//std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

// for map cloud generation
double map_cloud_resolution;

std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
//std::unique_ptr<MapCloudGenerator> map_cloud_generator;

// 局部优化每次处理的最大帧数  
int max_keyframes_per_update;
std::vector<KeyFrame::Ptr> wait_optimize_keyframes;
std::vector<KeyFrame::Ptr> wait_loopDetect_keyframes;

g2o::VertexPlane* floor_plane_node;

std::vector<KeyFrame::Ptr> keyframes;                            // 保存关键帧信息
std::vector<Loop::Ptr> Loops;                                    // 保存回环检测   
std::vector<pair<Eigen::Vector3d,Eigen::Vector3d>> EdgeSE3;      // 保存连接节点的边  
std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

//std::unique_ptr<GraphSLAM> graph_slam;
//std::unique_ptr<LoopDetector> loop_detector;
// 用于判断是否需要添加关键帧 
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
int init_imu_num = 1;                            // 用于初始化Map系位姿的imu数量     
// GPS权重  
double gps_edge_stddev_xy;
double gps_edge_stddev_z;
std::string key_frames_path;

// 数据队列
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2::ConstPtr> cloudBuf;
std::deque<GNSSData> GNSS_queue;

bool enable_imu_orientation;
bool gnss_origin_position_inited = false;
bool IMU_pose_inited = true;
bool enable_GNSS_optimize = false;  
bool enable_plane_optimize = false;  

std::mutex mBuf;
// 记录关键帧的index  
int KF_index = 0;
Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
Eigen::Quaterniond rotation_map2enu = Eigen::Quaterniond::Identity();       // Map到ENU系的旋转
Eigen::Vector3d gravity_in_map = {0,0,0};  
Eigen::Quaterniond MAP2ENU2 = {0,0,0,0};       // Map到ENU系的旋转
Eigen::Matrix4d Tme = Eigen::Matrix4d::Identity();
Eigen::Isometry3d Tem = Eigen::Isometry3d::Identity();
Eigen::Matrix4d Til;

using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::ColMajor>;

/************************************************************GNSS相关*********************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief GNSS坐标与Lidar坐标对齐
 * @param first_gnss_data 第一个有效GPS数据 
 * @param align_keyframe 与该GPS数据对齐的关键帧数据  其中 orientation 是通过IMU插值出来的精确姿态 
 * @return 是否初始化成功
 * TODO: 如何判断初始化失败 返回false ????? 
 */ 
bool GnssInitialize(GNSSData & first_gnss_data, KeyFrame::Ptr align_keyframe)
{
  // 进行初始化      将当前  gnss_data 的LLT数据 作为原点的 值    
  first_gnss_data.InitOriginPosition();
  Eigen::Quaterniond Rel = align_keyframe->orientation * Eigen::Quaterniond(Til.block<3,3>(0,0));    // Rel = Rei * Ril   激光雷达到ENU系的旋转  
  rotation_map2enu =  Rel * align_keyframe->Pose.matrix().block<3,3>(0,0).inverse();    // Rem = Rel * Rml^-1      局部MAP系到ENU系的旋转  
  // 构建 map系到enu系的变换 , 认为它们只有旋转的变化, enu系的原点还是在map系原点上  
  Tem.rotate(rotation_map2enu);
  Tem.pretranslate(Eigen::Vector3d(0,0,0));
  // 计算当前ENU坐标系坐标原点(GNSS初始化时的坐标系)相对与原点ENU坐标系的位置
  // 首先求得imu在MAP系下的位姿      
  Eigen::Matrix4d Tmi = align_keyframe->Pose.matrix() * Til.matrix().inverse();     // Tmi = Tml * Tli
  Gnss_init_Map_loc = rotation_map2enu * Tmi.block<3,1>(0,3);    // Rem * tmi = tei  ,  tmi 为 map 系下 m->i的位移向量   
  //ROS_INFO_STREAM("GNSS init OK !  Map loc: "<<Gnss_init_Map_loc);
  //ROS_INFO_STREAM("lidar pos:  "<< align_keyframe->Pose.matrix());

  // 更新keyframe的Pose     由于 坐标系只是变化了一个旋转  所以这里乘的Tem 只有旋转没有平移 
  // 之后 会更新 trans_odom2map 
  for(auto& keyframe:keyframes)
  {
    keyframe->Pose = Tem * keyframe->Pose;
  }
  for(auto& keyframe:wait_optimize_keyframes)
  {
    keyframe->Pose = Tem * keyframe->Pose;
  }
  // 如果初始化失败   这个就要保持false  
  gnss_origin_position_inited = true;

  return true;   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//静态成员变量必须在类外初始化
bool GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian GNSSData::geo_converter;
/**
 * @brief 将GNSS 经纬高信息 转换为 Map坐标系下Lidar坐标  
 * @param gnss_data 经纬高的值
 * @param orientation 对应位置IMU的姿态    由于 gnss_data 中的 imu 数据不是插值的数据  所以如果 有插值 这里要传入插值后的IMU   
 * @param[out] utm_coords 转换完成的值  
 */
void TransformWgs84ToMap(GNSSData &gnss_data, Eigen::Quaterniond const& orientation)
{
  // 如果初始化成功  
  if(gnss_origin_position_inited&&IMU_pose_inited)
  {
    gnss_data.UpdateXYZ();       // 计算ENU坐标  以初始化时IMU位置处ENU坐标系为原点    
    // 求得组合导航仪位于Map坐标系下的坐标         
    gnss_data.Lidar_Map_coords.x() = gnss_data.local_E + Gnss_init_Map_loc.x();
    gnss_data.Lidar_Map_coords.y() = gnss_data.local_N + Gnss_init_Map_loc.y();
    gnss_data.Lidar_Map_coords.z() = gnss_data.local_U + Gnss_init_Map_loc.z();  
    
    // 从IMU的UTM坐标转换为Lidar的UTM坐标   tel = Tei * til = Rei*til + tei  
    gnss_data.Lidar_Map_coords =  orientation * Til.block<3,1>(0,3) + gnss_data.Lidar_Map_coords;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 将关键帧与GNSS的数据进行插值对齐
 * 
 */
void flush_GNSS_queue()
{ 
  /******************************** 首先进行 GNSS 与 关键帧待处理队列 wait_optimize_keyframes 数据对齐 ******************************/    
  // 容器非空   
  if(wait_optimize_keyframes.empty()||GNSS_queue.empty()) 
  {
    return;  
  }

  // ROS_INFO_STREAM("GNSS_queue size: "<<GNSS_queue.size());
  // 如果GNSS 队尾 即 最新的GNSS数据早于 最早的一帧关键帧数据  说明没有与关键帧匹配的GNSS数据  直接退出
  if(GNSS_queue.back().time.toSec()<wait_optimize_keyframes.front()->stamp.toSec())
  {
    // ROS_INFO_STREAM("GNSS_queue.back().time < wait_optimize_keyframes.front() "<< GNSS_queue.back().time.toSec() - wait_optimize_keyframes.front()->stamp.toSec());
    GNSS_queue.clear();    // 清空无效的GNSS数据 
    return;
  }
  
  // 如果GNSS 队首 即 最早的GNSS数据晚于 最晚的一帧关键帧数据  说明没有与关键帧匹配的GNSS数据  直接退出
  if(GNSS_queue.front().time.toSec()>wait_optimize_keyframes.back()->stamp.toSec())
  {
    // ROS_INFO_STREAM("GNSS_queue.front().time > wait_optimize_keyframes.back() "<<GNSS_queue.front().time.toSec() - wait_optimize_keyframes.back()->stamp.toSec());
    // GNSS数据还有用  需要留着 
    return;
  }

  // 剩下的情况说明都有GNSS与关键帧有匹配的情况
  //ROS_INFO_STREAM("KEYFRAME SIZE: "<<wait_optimize_keyframes.size()<<"gps num: "<<gps_queue.size());
  for(auto keyframe:wait_optimize_keyframes)
  { 
    // ROS_INFO_STREAM(" time diff : "<< keyframe->stamp.toSec() - gps_queue.front().time);
    // GNSS_queue.front() 的数据 比 keyframe 早很多 
    while(keyframe->stamp.toSec() - GNSS_queue.front().time.toSec() > 0.10)
    {
      //  ROS_INFO_STREAM("keyframe - gps > 0.1 : "<< keyframe->stamp.toSec() - GNSS_queue.front().time.toSec());
      GNSS_queue.pop_front();   // 头数据丢弃 
      if(GNSS_queue.empty())  return;   
    }
    
    // gps 比 keyframe 晚很多     
    if(keyframe->stamp.toSec() - GNSS_queue.front().time.toSec() < -0.01)
    {
       // ROS_INFO_STREAM("keyframe - gps < -0.01" << keyframe->stamp.toSec() - GNSS_queue.front().time.toSec());
       continue;
    }

    // 当前数据合格    GNSSData 包括 经纬高以及 IMU测得的数据  
    GNSSData gnss_data = GNSS_queue.front();                  
    GNSS_queue.pop_front();         // 将头数据弹出
    GNSSData lidar_gnss_data = gnss_data;      // 插值后 lidar位置处 的数据 

    // 判断是否需要插值     如果GNSS与keyframe的数据差距在 5ms以上 那么进行插值 
    if(keyframe->stamp.toSec() - gnss_data.time.toSec() > 0.05)
    {
       if(GNSS_queue.empty())  return;       
       GNSSData gnss_data_2 = GNSS_queue.front();      // 取下一个      注意尾数据不要丢弃掉   要留着给下一次插值  
      
       double t1 = keyframe->stamp.toSec() - gnss_data.time.toSec();
       double t2 = gnss_data_2.time.toSec() - keyframe->stamp.toSec();

       // 时间间隔太大  那么线性插值就不准了  因此 退出
       if(t1+t2 > 0.2)   
       {
         ROS_INFO_STREAM("t1+t2 > 0.2"<<t1+t2);
         continue;
       }

      /************************************************* 进行插值  插值出 关键帧对应 IMU的 经纬高, 姿态 ********************************************/
      // 计算插值系数     
      double front_scale = t2 / (t2+t1);
      double back_scale = t1 / (t2+t1);
      // 对GPS进行插值  
      if (!gnss_origin_position_inited) 
      { // 如果没有对齐GNSS与Lidar的坐标  那么对经纬高进行插值  准备坐标对齐 
        lidar_gnss_data.longitude = gnss_data_2.longitude*back_scale + gnss_data.longitude*front_scale;
        lidar_gnss_data.latitude = gnss_data_2.latitude*back_scale + gnss_data.latitude*front_scale;
        lidar_gnss_data.altitude = gnss_data_2.altitude*back_scale + gnss_data.altitude*front_scale;  
      }
      else
      {
        lidar_gnss_data.Lidar_Map_coords.x() = gnss_data_2.Lidar_Map_coords.x()*back_scale + gnss_data.Lidar_Map_coords.x()*front_scale;
        lidar_gnss_data.Lidar_Map_coords.y() = gnss_data_2.Lidar_Map_coords.y()*back_scale + gnss_data.Lidar_Map_coords.y()*front_scale;
        lidar_gnss_data.Lidar_Map_coords.z() = gnss_data_2.Lidar_Map_coords.z()*back_scale + gnss_data.Lidar_Map_coords.z()*front_scale;  
      }

      // 对IMU的姿态进行插值 设置给  keyframe->orientation 
      Eigen::Quaterniond imu_front,imu_back;
      imu_front.w() = gnss_data.imu->orientation.w;
      imu_front.x() = gnss_data.imu->orientation.x;
      imu_front.y() = gnss_data.imu->orientation.y;
      imu_front.z() = gnss_data.imu->orientation.z;
      imu_back.w() = gnss_data_2.imu->orientation.w;
      imu_back.x() = gnss_data_2.imu->orientation.x;
      imu_back.y() = gnss_data_2.imu->orientation.y;
      imu_back.z() = gnss_data_2.imu->orientation.z;
      // 球面插值  
      keyframe->orientation = imu_front.slerp(back_scale, imu_back);       

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
      keyframe->orientation.w() = lidar_gnss_data.imu->orientation.w;
      keyframe->orientation.x() = lidar_gnss_data.imu->orientation.x;
      keyframe->orientation.y() = lidar_gnss_data.imu->orientation.y;
      keyframe->orientation.z() = lidar_gnss_data.imu->orientation.z; 
    }
                   
    /**************** 如果GNSS没有初始化  那么进行初始化   初始化主要是求 map->enu 的 Tem, 以及 Gnss_init_Map_loc   ***********************************/
    if (!gnss_origin_position_inited) 
    { 
      if(!GnssInitialize(lidar_gnss_data, keyframe))    // 进行GNSS与MAP对齐  
         return;
      // 初始化后转换为Map坐标
      TransformWgs84ToMap(lidar_gnss_data, keyframe->orientation);
    }

    // 获得Lidar的GNSS在Map系的坐标     
    keyframe->utm_coord = lidar_gnss_data.Lidar_Map_coords;
    keyframe->GNSS_Valid = true;
  }

  return ;
}

/************************************************************回调函数*********************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 读取前端发送的激光里程计数据  判断是否需要添加关键帧   
 * @details 检测是否需要添加关键帧, 如果需要将关键帧信息添加到 new_keyframe_queue 
 */
void cloudCallback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
{
  // ROS_INFO_STREAM("backend receive !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  const ros::Time& stamp = odom_msg->header.stamp;
  Eigen::Isometry3d odom = odom2isometry(odom_msg);                               // ROS ->Eigen
  // ROS_INFO_STREAM("receive odom cloud");
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if(base_frame_id.empty()) 
  {
    base_frame_id = cloud_msg->header.frame_id;
  }

  // 是否需要添加关键帧
  if(!keyframe_updater->update(odom)) 
  {
    return;
  }
  
  // 把关键帧点云存储到硬盘里     不消耗内存
  // 如果未来维护关键帧包括关键帧的删除或替换的话 , 那么 KF_index 的也需要去维护 
  std::string file_path = key_frames_path + "/key_frame_" + std::to_string(KF_index) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *cloud);

  // 通过点云与里程计和累计距离等来创建关键帧   实际的关键帧中就不包含点云数据了  
  KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, KF_index, cloud));
  // 线程锁开启
  std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
  new_keyframe_queue.push_back(keyframe);     // 加入处理队列
  // KF index 更新  
  KF_index++;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 读取组合导航仪的imu以及GNSS数据     
 * @details 初始化前, 将原始GNSS数据保存, 初始化后, 将GNSS数据转换为MAP坐标系值然后保存  并实时输出  
 */
void gnssCallback(const sensor_msgs::ImuConstPtr& imu_msg, const sensor_msgs::NavSatFixConstPtr& navsat_msg)
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
  // 如果初始化成功 
  if(gnss_origin_position_inited&&IMU_pose_inited)
  {
    Eigen::Quaterniond orientation;
    orientation.w() = gnss_data.imu->orientation.w;
    orientation.x() = gnss_data.imu->orientation.x;
    orientation.y() = gnss_data.imu->orientation.y;
    orientation.z() = gnss_data.imu->orientation.z; 

    TransformWgs84ToMap(gnss_data, orientation);      // 将WGS84坐标值转换为Map系坐标

    // 发布odom
    geometry_msgs::PoseStamped Pose_lidar_in_map; 
    Pose_lidar_in_map.header.stamp = gnss_data.time;
    Eigen::Quaterniond Qml = Eigen::Quaterniond::Identity();
    Eigen::Vector3d tml = Eigen::Vector3d(0,0,0);
    tml = gnss_data.Lidar_Map_coords;
    Qml = orientation * Eigen::Quaterniond(Til.block<3,3>(0,0));
    Pose_lidar_in_map.pose.position.x = tml.x();
    Pose_lidar_in_map.pose.position.y = tml.y();
    Pose_lidar_in_map.pose.position.z = tml.z();
    Pose_lidar_in_map.pose.orientation.w = Qml.w();
    Pose_lidar_in_map.pose.orientation.x = Qml.x();
    Pose_lidar_in_map.pose.orientation.y = Qml.y();
    Pose_lidar_in_map.pose.orientation.z = Qml.z();
    GNSSPath.header.stamp = Pose_lidar_in_map.header.stamp;
    GNSSPath.poses.push_back(Pose_lidar_in_map);
    // 控制发布频率
    static int i = 0;
    if(i++==2)
    {
      GNSSPath.header.frame_id = map_frame_id; // odom坐标
      pubGNSSPath.publish(GNSSPath);
      i = 0;
    }
  }

  // 原始数据直接存放
  GNSS_queue.push_back(gnss_data);
}


/************************************************************优化处理*********************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Parse_data()
{

  /*
  // 还未初始化  
  // 静态初始化   静止放置  等待初始化完成  
  if(!IMU_pose_inited)
  {
    static int index = 0;
    if(!GNSS_queue.empty())   
    {
      for(int i = 0; i<GNSS_queue.size(); i++)
      {  // 静态初始化    不适合kitti数据集  
         // 法一    imu侧量的 orientation 应该是 IMU -> ENU 系的旋转   Map系是建立在Lidar上的   所以要求Lidar->ENU系的旋转  即  Rei*Ril
         rotation_map2enu.w() += GNSS_queue[i].imu->orientation.w;
         rotation_map2enu.x() += GNSS_queue[i].imu->orientation.x;
         rotation_map2enu.y() += GNSS_queue[i].imu->orientation.y;
         rotation_map2enu.z() += GNSS_queue[i].imu->orientation.z;
         // 法二
         gravity_in_map.x() += GNSS_queue[i].imu->linear_acceleration.x;
         gravity_in_map.y() += GNSS_queue[i].imu->linear_acceleration.y;
         gravity_in_map.z() += GNSS_queue[i].imu->linear_acceleration.z;
        
         index++;
         if(index>=init_imu_num)
         {
           // 此时应该是  rotation_map2enu = Rei
           rotation_map2enu.w() /= index;
           rotation_map2enu.x() /= index;
           rotation_map2enu.y() /= index;
           rotation_map2enu.z() /= index;
           // Rem = Rei*Ril
           rotation_map2enu = rotation_map2enu * Eigen::Quaterniond(Til.block<3,3>(0,0));
           rotation_map2enu.normalize();
           Tem.rotate(rotation_map2enu);  
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
             //           ROS_INFO_STREAM("MAP init ok! res1: "<< rotation_map2enu.coeffs()<<" res2: "<<MAP2ENU2.coeffs() );
           break;
           
         }
         IMU_pose_inited = true;
      }
    }
  }   
  */
  // 对新加入的关键帧进行处理      
  // 如果没有新的关键帧  
  if(new_keyframe_queue.empty()) 
  {
    return false;
  }

  trans_odom2map_mutex.lock();
  Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
  trans_odom2map_mutex.unlock();
  int num_processed = std::min<int>(new_keyframe_queue.size(), max_keyframes_per_update);

  // 遍历全部关键帧队列 计算各种观测量         
  for(int i=0; i<num_processed; i++) 
  {
    // 从keyframe_queue中取出关键帧
    const auto& keyframe = new_keyframe_queue[i];
    // odom系与map系对齐 
    keyframe->Pose = odom2map * keyframe->odom;  
    // 计算该关键帧的全局描述子
    
    // 放置到待优化容器中     
    wait_optimize_keyframes.push_back(keyframe);   
    // 放置到待回环检测容器中
    wait_loopDetect_keyframes.push_back(keyframe); 
  }

  new_keyframe_queue.erase(new_keyframe_queue.begin(), new_keyframe_queue.begin() + num_processed);
  
  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 创建可视化队列
 * @param stamp
 * @return
 */
visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) 
{
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
  edge_marker.scale.x = 0.1;
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

    edge_marker.colors[i*2].r = 1.0;
    edge_marker.colors[i*2].g = 2.0;
    edge_marker.colors[i*2].a = 1.0;
    edge_marker.colors[i*2 + 1].r = 1.0;
    edge_marker.colors[i*2 + 1].g = 2.0;
    edge_marker.colors[i*2 + 1].a = 1.0;
    i++;
    
    if(enable_GNSS_optimize)
    {
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
    }

    // 地面约束边  
    if(enable_plane_optimize)
    {
       Eigen::Vector3d plane = {pt1[0], pt1[1], 0};

       edge_marker.points[i*2].x = pt1.x();
       edge_marker.points[i*2].y = pt1.y();
       edge_marker.points[i*2].z = pt1.z();
       edge_marker.points[i*2 + 1].x = plane.x();
       edge_marker.points[i*2 + 1].y = plane.y();
       edge_marker.points[i*2 + 1].z = plane.z()-1;

       edge_marker.colors[i*2].r = 1.0;
       edge_marker.colors[i*2].a = 2.0;
       edge_marker.colors[i*2 + 1].r = 1.0;
       edge_marker.colors[i*2 + 1].a = 2.0;
       i++;
    }
    
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

  return markers;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 局部优化      里程计约束+地面约束+GNSS约束   
void local_optimize()
{
  // Setup optimizer
  g2o::SparseOptimizer optimizer;   // 定义稀疏求解器  
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,-1>> Block;     // 重命名 块求解器     
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linearSolver);       // 定义 块求解器  
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);     // 构造优化方法  LM  
  optimizer.setAlgorithm(solver);            // 设置优化方法到 稀疏求解器

  int bias=3;
  if(keyframes.empty())
    bias = 2;

  // 记录本次局部优化中  节点与边的序号   
  int vertexCnt=0, edgeCnt=0;
  ROS_INFO_STREAM("Local optimation!");
  // 记录本次优化的全部节点  用于边寻找关联 节点    
  vector<g2o::VertexSE3*> vertexs;         
  Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();

  // 遍历本次优化的全部关键帧  
  for(const auto& keyframe:wait_optimize_keyframes)
  {      
    // 添加节点  
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(vertexCnt++);
    // ROS_INFO_STREAM("setEstimate() : "<<keyframe->odom.matrix());
    v->setEstimate(keyframe->Pose);
    optimizer.addVertex(v);
    vertexs.push_back(v);
    
    if(enable_GNSS_optimize)
    {
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
    }

    // 可以添加全局平面约束   
    if(enable_plane_optimize)
    {
        /*
        g2o::EdgeSE3PlanePrior* plane_prior(new g2o::EdgeSE3PlanePrior());
        plane_prior->setVertex(0, v); 
        std::cout<<"origin_pose--------------------------"<<v->estimate().matrix()<<std::endl;
        // 设置先验
        Eigen::Matrix3d Rwb = v->estimate().linear();     // 旋转
        Eigen::Vector3d twb = v->estimate().translation();   // XYZ   
        Eigen::Vector3d euler_angles = Rwb.eulerAngles(2,1,0);   // 转欧拉角    y, p , r
        std::cout<<"Rwb--------------------------"<<Rwb<<std::endl;
        std::cout<<"twb--------------------------"<<twb<<std::endl;
        euler_angles[1] = 0;
        euler_angles[2] = 0;    
        twb[2] = 0;
        // 欧拉角转旋转矩阵
        Eigen::AngleAxisd rollAngle(euler_angles(2),Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(euler_angles(1),Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(euler_angles(0),Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd prior_rotation_vector (yawAngle*pitchAngle*rollAngle);
        //Eigen::AngleAxisd prior_rotation_vector(Rwb) ;

        Eigen::Isometry3d prior_pose = Eigen::Isometry3d::Identity();
        prior_pose.rotate(prior_rotation_vector); 
        prior_pose.pretranslate(twb); 
        std::cout<<"prior_pose--------------------------"<<prior_pose.matrix()<<std::endl;
        plane_prior->setMeasurement(prior_pose); 

        // 设置信息矩阵 
        // 信息矩阵     6*6       JtWJ     与残差维度相等
        Matrix6d information_matrix = Matrix6d::Identity();
        information_matrix.block<3, 3>(0, 0) /= 5;     // 旋转相关  约束不大 
        information_matrix.block<3,3>(3,3) /= 1;                  // 
        plane_prior->setInformation(information_matrix);
        
        optimizer.addEdge(plane_prior);
        */
      // 高度约束以及姿态约束
      // 高度约束 
      g2o::EdgeSE3PriorXYZ* prior_height_edge(new g2o::EdgeSE3PriorXYZ());
      Eigen::Vector3d twb = v->estimate().translation();   // XYZ 
      std::cout<<" origin twb ----------------------------"<<std::endl<<twb.transpose()<<std::endl;
      twb[2] = 0;  
      prior_height_edge->setMeasurement(twb);              // 设置观测先验  
      prior_height_edge->vertices()[0] = v;
      // 信息矩阵     3*3      JtWJ  
      Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
      information_matrix.block<2, 2>(0, 0) *= 0;     // x,y直接设为0  
      information_matrix(2, 2) /= 10;                 // 5  
      prior_height_edge->setInformation(information_matrix);
      optimizer.addEdge(prior_height_edge);
      /*
      // 姿态约束
      g2o::EdgeSO3Prior* prior_rot_edge(new g2o::EdgeSO3Prior());
      prior_rot_edge->vertices()[0] = v;
      Eigen::Matrix3d Rwb = v->estimate().linear();   // 旋转
      
      // 转换到欧拉角
      Eigen::Vector3d euler_angles = Rwb.eulerAngles(2,1,0);   // 转欧拉角    y, p, r 
      euler_angles[1] = 0;
      euler_angles[2] = 0;
      // 欧拉角转旋转矩阵
      Eigen::Matrix3d prior_rotation_matrix;
      prior_rotation_matrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ()) * 
                              Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) * 
                              Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());
      prior_rot_edge->setMeasurement(prior_rotation_matrix);
      std::cout<<"prior_pose_RPY--------------------------"<<std::endl<<prior_rotation_matrix.eulerAngles(2,1,0)<<std::endl;
      // 信息矩阵     3*3      JtWJ  
      Eigen::Matrix3d prior_rot_information_matrix = Eigen::Matrix3d::Identity();
      prior_rot_information_matrix.block<3, 3>(0, 0) /= 1;                                
      prior_rot_edge->setInformation(prior_rot_information_matrix);
      optimizer.addEdge(prior_rot_edge);
      */
    }      

    // 处理本次局部优化的第一个节点  
    if(vertexCnt==1)
    { // 历史上第一个节点  
      if(keyframes.empty())
      {
        v->setFixed(true);            // 要进行fix处理  
        keyframe->deltaPose = Eigen::Isometry3d::Identity();
      }
      else{    // 本次优化的第一个节点  
        // 创建一个固定的原点节点   当前 keyframes 最后一个节点   
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
        keyframe->deltaPose = relative_pose;    // 记录计算出来的相对位移这样就不用在显示的时候去计算了
        edge->setMeasurement(relative_pose);
        // 计算信息矩阵    通过kdtree检查点云通过变换后的匹配程度反映里程计是否准确   匹配程度越高  则信息矩阵各权重越大   则优化时  会更靠近里程计的结果   
        Eigen::MatrixXd information = inf_calclator->calc_information_matrix( keyframes.back()->cloud, keyframe->cloud, relative_pose);
        edge->setInformation(information);
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
    const auto& prev_keyframe = wait_optimize_keyframes[vertexCnt-bias];                        // 获取上一关键帧   
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

  for(int i=0; i<wait_optimize_keyframes.size(); i++)
  {
    wait_optimize_keyframes[i]->Pose = vertexs[i]->estimate();    //  获取优化结果
    //  ROS_INFO_STREAM("i: "<<i<<"after optimize: "<<wait_optimize_keyframes[i]->Pose.matrix().cast<float>()); 
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 全局优化      里程计约束+GNSS约束+回环约束+(可选)地面约束   
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

  for(const auto& keyframe:keyframes)
  {  
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

  for(auto& l:add_loop) 
  {
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
 */
//void optimization_timer_callback(const ros::WallTimerEvent& event) {
void Optimization() 
{
  std::lock_guard<std::mutex> lock(main_thread_mutex);
  
  // 执行局部优化
  local_optimize();
  // loop detection
  Loop::Ptr loop = loop_detector->detect(keyframes, wait_optimize_keyframes);
  std::copy(wait_optimize_keyframes.begin(), wait_optimize_keyframes.end(), std::back_inserter(keyframes));
  wait_optimize_keyframes.clear();
  // 如果存在回环 
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

  // 如果有订阅者  发布odom到map坐标系的变换  
  if(odom2map_pub.getNumSubscribers()) 
  {
    ROS_INFO_STREAM("BackEnd_node - trans_odom2map: "<<std::endl<<trans_odom2map);   
    // 构造 ROS Msg
    geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans_odom2map.cast<float>(), map_frame_id, odom_frame_id);
    odom2map_pub.publish(ts);
  }
  // 可视化     
  if(markers_pub.getNumSubscribers()) 
  {
    auto markers = create_marker_array(ros::Time::now());
    markers_pub.publish(markers);
  }  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 发布地图给rviz显示
 */
//void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
void map_points_publish() 
{  
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ros::Time Optimize_previous_time = ros::Time(0);
ros::Time Map_updata_previous_time = ros::Time(0);
#define Optimize_duration 3
#define Map_updata_duration 10  
// 后端处理线程  
void BackEnd_process()
{
   while(true)
   {
      if(Optimize_previous_time.toSec() == 0 && Map_updata_previous_time.toSec() == 0)
      {
        Optimize_previous_time = ros::Time::now();
        Map_updata_previous_time = ros::Time::now();
      }
      else
      {
        double Optimize_diff_time = (ros::Time::now() - Optimize_previous_time).toSec();     // 计算时间差
        double Map_updata_diff_time = (ros::Time::now() - Map_updata_previous_time).toSec();     // 计算时间差
        
        if(Parse_data())                       // 数据处理  对齐GNSS, 计算描述子, 以及其他观测量  
        {
          // 图优化  周期控制    进行优化的时候  回环检测线程阻塞 !!!!
          if(Optimize_diff_time>= Optimize_duration)
          { 
            Optimize_previous_time = ros::Time::now();
            // 批量处理  -  进行GNSS 与keyframe 匹配
            flush_GNSS_queue();
            // 进行优化 
            Optimization();
            // ROS_INFO_STREAM("Graph optimization!");
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
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 回环检测线程
 * @details 对 wait_loopDetect_keyframes 中的关键帧进行回环检测
 */
void loopDetect_process()
{
  while(1)
  {
     
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS 通信初始化 
void comm_init(ros::NodeHandle nh)
{
    // publishers
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);        // 可视化
    odom2map_pub = nh.advertise<geometry_msgs::TransformStamped>("/odom2pub", 16);      // odom到map的校正  
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
    Sync_odomCloud->registerCallback(boost::bind(&cloudCallback, _1, _2)); 
    
    /*
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_opt_low", 1000,odom_callback);
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/key_cloud", 1000, cloudCallback);
    */
    // 同步订阅 GNSS的数据   包括IMU与GPS
    // GPS订阅
    
    imu_sub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/kitti/oxts/imu", 1000));
    navsat_sub.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, "/kitti/oxts/gps/fix", 1000));
    Sync_GPSIMU.reset(new message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::NavSatFix>(*imu_sub, *navsat_sub, 1000));          
    Sync_GPSIMU->registerCallback(boost::bind(&gnssCallback, _1, _2));   
    

    // 服务
    //dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    //save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 参数初始化 
void params_init(ros::NodeHandle nh)
{
    keyframe_updater.reset(new KeyframeUpdater(nh));
    loop_detector.reset(new LoopDetector(nh));
    // graph_slam.reset(new GraphSLAM(nh.param<std::string>("g2o_solver_type", "lm_var")));
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
    // GNSS优化
    enable_GNSS_optimize = nh.param<bool>("enable_GNSS_optimize", false);
    // 平面优化
    enable_plane_optimize = nh.param<bool>("enable_plane_optimize", false);
    // 关键帧点云保存的空间
    key_frames_path = "/home/mini/code/localization_ws/src/liv_slam/Map";  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init (argc, argv, "BackEnd_node");   
    ROS_INFO("Started BackEnd_node node");    
    ros::NodeHandle nh("~");  
    // 通信初始化
    comm_init(nh);
    // 参数初始化 
    params_init(nh);
    // 观测处理线程  - 后端优化主线程  
    std::thread measurement{BackEnd_process}; 
    // 回环检测线程 
    std::thread loopDetect{loopDetect_process}; 
    ros::spin(); 
    return 0;
}

























