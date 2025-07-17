/*
* This file is part of liodom.
*
* Copyright (C) 2020 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
*
* liodom is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* liodom is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with liodom. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INCLUDE_LIODOM_LASER_ODOMETRY_H
#define INCLUDE_LIODOM_LASER_ODOMETRY_H

#include <numeric>
#include <thread>
#include <deque>
#include <tuple>
#include <cmath>

// Ceres
#include <ceres/ceres.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

// Liodom
#include <liodom/defs.h>
#include <liodom/factors.hpp>
#include <liodom/params.h>
#include <liodom/shared_data.h>
#include <liodom/stats.h>


//lanelet
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/LaneletMap.h>

#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>
// #include <lanelet2_io/IoHelpers.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_matching/LaneletMatching.h>
// #include <lanelet2_matching/Matching.h>

#include <algorithm>
#include <set>
#include <limits>
namespace liodom {

// Local Map manager
class LocalMapManager {
 public:
  explicit LocalMapManager(const size_t max_frames);
  virtual ~LocalMapManager();

  void addPointCloud(const PointCloud::Ptr& pc);
  size_t getLocalMap(PointCloud::Ptr& map);
  void setMaxFrames(const size_t max_nframes);
  void replaceLocalMap(const PointCloud::Ptr& new_map);

 private:
  PointCloud::Ptr total_points_;
  size_t nframes_;
  size_t max_nframes_;
  std::queue<size_t> sizes_;  
};

// Laser Odometry
class LaserOdometer {
 public:
  explicit LaserOdometer(const rclcpp::Node::SharedPtr& nh);
  virtual ~LaserOdometer();

  void operator()(std::atomic<bool>& running);

 private:
  // ROS variables
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ ;
  std::shared_ptr<lanelet::projection::UtmProjector> projector_;
  // Variables
  bool init_;
  Eigen::Isometry3d prev_odom_;
  Eigen::Isometry3d odom_;
  double prev_stamp_;  
  double param_q[4] = {0, 0, 0, 1};
  double param_t[3] = {0, 0, 0};
  SharedData* sdata;
  Stats* stats;
  Params* params;
  LocalMapManager lmap_manager;
  Eigen::Isometry3d laser_to_base_; 
  double in_freqs_[5];
  double out_freqs_[5];
  double mean_in_freq_;
  double mean_out_freq_;
  int num_freqs_;
  double last_in_time_secs_;
  double last_out_time_secs_;
  lanelet::Id current_lanelet_id_;
  std::deque<Eigen::Isometry3d> pose_history_;
  std::vector<Eigen::Vector2d> lane_points;
  pcl::KdTreeFLANN<Point>::Ptr lane_kdtree_;
  PointCloud::Ptr lane_cloud_;

  void computeLocalMap(PointCloud::Ptr& local_map_gen, PointCloud::Ptr& local_map_rec);
  void addEdgeConstraints(const PointCloud::Ptr& edges,
                          const PointCloud::Ptr& local_map_gen,
                          const PointCloud::Ptr& local_map_rec,
                          const Eigen::Isometry3d& pose,
                          ceres::Problem* problem,
                          ceres::LossFunction* loss);
  bool getBaseToLaserTf(const std::string& frame_id);
  void publishOdom(const std_msgs::msg::Header& header, const Eigen::Isometry3d& pose);
  void addLaneletConstraints(const Eigen::Isometry3d& pose,
                          ceres::Problem* problem,
                          ceres::LossFunction* loss);
  void alignTrajectoryToLane();
  void updateLocalMapWithTransformation(const Eigen::Matrix2d& R, const Eigen::Vector2d& t);
  std::vector<Eigen::Vector2d> findClosestLanePoints(const std::deque<Eigen::Isometry3d>& pose_history);
  
  // Normal shooting correspondences method
  std::vector<Eigen::Vector2d> findNormalShootingCorrespondences(const std::vector<Eigen::Vector2d>& trajectory_points, 
                                                                 const std::vector<Eigen::Vector2d>& closest_lane_points);
  
  // ICP solver method
  std::tuple<Eigen::Matrix2d, Eigen::Vector2d, double> solveIcp2d(
    const std::vector<Eigen::Vector2d>& source_points,
    const std::vector<Eigen::Vector2d>& target_points,
    int max_iterations = 50,
    double tolerance = 1e-6);
  
  // Geometric pose correction method - finds intersection of perpendicular lines
  Eigen::Vector2d findGeometricIntersection(const Eigen::Vector2d& pn_minus_1, 
                                           const Eigen::Vector2d& pn, 
                                           const Eigen::Vector2d& pn_prime);
  
  // K-nearest neighbors method for finding k closest lane point indices for each trajectory point
  // Returns a vector of vectors: each inner vector contains k closest indices for one trajectory point
  std::vector<std::vector<size_t>> findKClosestNeighborsForPoints(
    const std::vector<Eigen::Vector2d>& map_points,
    const std::vector<Eigen::Vector2d>& points,
    int k = 1);
  
  // K-nearest neighbors method using PCL KD-tree for efficient search
  // Returns a vector of vectors: each inner vector contains k closest indices for one trajectory point
  std::vector<std::vector<size_t>> findKClosestNeighborsForPointsKdTree(
    const std::vector<Eigen::Vector2d>& points,
    int k = 3);
  
  // Normal shooting from KNN correspondences
  std::vector<Eigen::Vector2d> findNormalShootingFromKnn(
    const std::vector<Eigen::Vector2d>& trajectory_points,
    const std::vector<std::vector<size_t>>& knn_indices,
    const std::vector<Eigen::Vector2d>& map_points);
  
  // Helper method to extract trajectory points from pose history
  std::vector<Eigen::Vector2d> extractTrajectoryPoints(const std::deque<Eigen::Isometry3d>& pose_history);
};

}  // namespace liodom

#endif // INCLUDE_LIODOM_LASER_ODOMETRY_H
