#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>


/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped  Eigen转换为tf msg
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
  // 旋转矩阵 -> 四元数
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  // 四元数单位化
  quat.normalize();
  // 构造四元数   ROS信息
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();
  
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  // 该tf关系表示 从 frame_id-> child_frame_id
  odom_trans.header.frame_id = frame_id;       
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}


// 输入: 位姿的ROS Msg
// 输出: Eigen变换矩阵
static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  const auto& orientation = odom_msg->pose.pose.orientation;  
  const auto& position = odom_msg->pose.pose.position;
  // ROS   四元数转Eigen
  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  // linear是获取旋转矩阵
  isometry.linear() = quat.toRotationMatrix();
  // 赋值平移
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}    

#endif // ROS_UTILS_HPP
