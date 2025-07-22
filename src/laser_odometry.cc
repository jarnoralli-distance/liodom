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

#include <liodom/laser_odometry.h>

// These parameters are now accessed through params system


namespace liodom {

  LocalMapManager::LocalMapManager(const size_t max_frames) :  
    total_points_(new PointCloud),
    nframes_(0),
    max_nframes_(max_frames)
    {
  }

  LocalMapManager::~LocalMapManager() {
  }

  void LocalMapManager::addPointCloud(const PointCloud::Ptr& pc) {
    // Adding the current frame to the local map
    *total_points_ += *pc;
    nframes_++;
    sizes_.push(pc->size());

    // Removing frames at the beginning if required
    if (nframes_ > max_nframes_) {
      // Get the size of the first frame
      size_t pc_size = sizes_.front();
      sizes_.pop();

      // Remove the points corresponding to the first frame
      std::vector<int> ind(pc_size);
      std::iota(std::begin(ind), std::end(ind), 0);
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      indices->indices = ind;
      pcl::ExtractIndices<Point> extract;
      extract.setInputCloud(total_points_);
      extract.setIndices(indices);
      extract.setNegative(true);
      extract.filter(*total_points_);
    
      // Reducing the number of frames
      nframes_--;
    }
  }

  size_t LocalMapManager::getLocalMap(PointCloud::Ptr& map) {
    map = total_points_;
    return nframes_;
  }

  void LocalMapManager::setMaxFrames(const size_t max_nframes) {
    max_nframes_ = max_nframes;
  }

  LaserOdometer::LaserOdometer(const rclcpp::Node::SharedPtr& nh) :
    nh_(nh),
    init_(false),
    prev_odom_(Eigen::Isometry3d::Identity()),
    odom_(Eigen::Isometry3d::Identity()),
    prev_stamp_(0.0),
    sdata(SharedData::getInstance()),
    stats(Stats::getInstance()),
    params(Params::getInstance()),
    lmap_manager(params->local_map_size_),
    num_freqs_(0) {

    for (int i = 0; i < 5; i++) {
      in_freqs_[i] = 20.0; // 100/5
      out_freqs_[i] = 20.0; // 100/5
    }
    mean_in_freq_ = 100.0; // high freq.
    mean_out_freq_ = 100.0; // high freq.
    last_in_time_secs_ = nh_->now().seconds();  
    last_out_time_secs_ = last_in_time_secs_;

    // Publishers
    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);

    // TF listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if(params->use_icp_optimization_ ) {
    //Load lanlet map and projector
    projector_ = std::make_shared<lanelet::projection::UtmProjector>(lanelet::Origin({params->origin_coords_lanelet_[0], params->origin_coords_lanelet_[1]}));
    lanelet_map_= lanelet::load(params->map_lanelet_path_, *projector_);

      // Define rotation matrix (same as before)
    double rotation_angle = params->angle_lanelet_correction_ * (M_PI / 180.0);
    double cos_angle = std::cos(rotation_angle);
    double sin_angle = std::sin(rotation_angle);

    Eigen::Matrix2d R_M;
    R_M << cos_angle, -sin_angle,
            sin_angle, cos_angle;

    // Gather all lanelet centerline points into a vector
    for (const auto& lanelet : lanelet_map_->laneletLayer) {
        auto centerline = lanelet.centerline();
        for (const auto& point : centerline) {
          Eigen::Vector2d correct_point(point.x(), point.y());
          correct_point = R_M * correct_point;
          lane_points.emplace_back(correct_point.x(), correct_point.y());
            lane_point_ids.push_back(lanelet.id());
        }
    }
    
    // Build KD-tree for lane points (2D only, z=0)
    lane_kdtree_ = std::make_shared<pcl::KdTreeFLANN<Point>>();
    lane_cloud_ = std::make_shared<PointCloud>();
    lane_cloud_->reserve(lane_points.size());
    for (const auto& point : lane_points) {
      Point pcl_point;
      pcl_point.x = point.x();
      pcl_point.y = point.y();
      pcl_point.z = 0.0;  // Z is always 0 for 2D lane points
      lane_cloud_->push_back(pcl_point);
    }
    lane_kdtree_->setInputCloud(lane_cloud_);
    
  }

  LaserOdometer::~LaserOdometer() {
  }

  void LaserOdometer::operator()(std::atomic<bool>& running) {
    
    while(running) {   

      PointCloud::Ptr feats(new PointCloud);
      std_msgs::msg::Header feat_header;
      
      if (sdata->popFeatures(feats, feat_header)) {    
        if (!init_) {        

          // cache the static tf from base to laser
          if (params->laser_frame_ == "") {
            params->laser_frame_ = feat_header.frame_id;
          }

          if (!getBaseToLaserTf(params->laser_frame_))
          {
            RCLCPP_WARN(nh_->get_logger(), "Skipping point_cloud");
            return;
          }

          auto start_t = Clock::now();

          lmap_manager.addPointCloud(feats);
          init_ = true;
          prev_stamp_ = rclcpp::Time(feat_header.stamp).seconds();
          
          auto end_t = Clock::now();

          // Register stats
          if (params->save_results_) {
            stats->addPose(odom_.matrix());
            stats->addLaserOdometryTime(start_t, end_t);
            stats->stopFrame(end_t);          
          }

          publishOdom(feat_header, odom_);

        } else {

          auto start_t = Clock::now();

          // Computing local map
          PointCloud::Ptr local_map_rec(new PointCloud);
          PointCloud::Ptr local_map_gen(new PointCloud);
          computeLocalMap(local_map_gen, local_map_rec);    

          // Predict the current pose
          Eigen::Isometry3d pred_odom = odom_ * (prev_odom_.inverse() * odom_);
          prev_odom_ = odom_;
          odom_ = pred_odom;

          if (params->use_imu_){

            // 1.- get roll and pitch from IMU (frame baselink)
            Eigen::Quaterniond imu_ori = Eigen::Quaterniond::Identity();
            sdata->getLastIMUOri(imu_ori);
            tf2::Quaternion imu_quat(imu_ori.x(), imu_ori.y(), imu_ori.z(), imu_ori.w());
            tf2::Matrix3x3 imu_m(imu_quat);
            double imu_roll, imu_pitch, imu_yaw;
            imu_m.getRPY(imu_roll, imu_pitch, imu_yaw);

            //2.- rotate odom to frame baselink and get the orientation
            Eigen::Isometry3d odom_bl = odom_ ;
            Eigen::Quaterniond odom_ori_bl(odom_bl.rotation());
            tf2::Quaternion odom_bl_quat(odom_ori_bl.x(), odom_ori_bl.y(), odom_ori_bl.z(), odom_ori_bl.w());
            tf2::Matrix3x3 odom_bl_m(odom_bl_quat);
            double odom_bl_roll, odom_bl_pitch, odom_bl_yaw;
            odom_bl_m.getRPY(odom_bl_roll, odom_bl_pitch, odom_bl_yaw);

            //ROS_INFO("roll %2.2f:%2.2f, pitch %2.2f:%2.2f, yaw %2.2f:%2.2f", imu_roll, odom_bl_roll, imu_pitch, odom_bl_pitch, imu_yaw, odom_bl_yaw);

            //3.- overwrite roll and pitch
            odom_bl_m.setRPY(imu_roll, imu_pitch, odom_bl_yaw);

            //4.- fill the eigen structure with the new orientation
            odom_bl_m.getRotation(odom_bl_quat);
            odom_ori_bl = Eigen::Quaterniond(odom_bl_quat.w(), odom_bl_quat.x(), odom_bl_quat.y(), odom_bl_quat.z());
            odom_bl.linear() = odom_ori_bl.toRotationMatrix();

            //5.- rotate odom back to frame laser
            odom_ = odom_bl;
          }

          // Updating the initial guess
          Eigen::Quaterniond q_curr(odom_.rotation());
          q_curr.normalize();

          param_q[0] = q_curr.x();
          param_q[1] = q_curr.y();
          param_q[2] = q_curr.z();
          param_q[3] = q_curr.w();

          Eigen::Vector3d t_curr = odom_.translation();
          param_t[0] = t_curr.x();
          param_t[1] = t_curr.y();
          param_t[2] = t_curr.z();

          // Optimize the current pose
          for (int optim_it = 0; optim_it < 2; optim_it++) {
            
            // Define the optimization problem
            ceres::LossFunction* loss_function = new ceres::HuberLoss(0.2);
            ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(param_q, 4, q_parameterization);
            problem.AddParameterBlock(param_t, 3);

            // Adding constraints
            addEdgeConstraints(feats, local_map_gen, local_map_rec, odom_, &problem, loss_function);

            // Solving the optimization problem
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );          
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            
            // std::cout << summary.BriefReport() << "\n";

            odom_ = Eigen::Isometry3d::Identity();
            // odom_.linear() = q_curr.toRotationMatrix();
            // odom_.translation() = t_curr;
            Eigen::Quaterniond q_new(param_q[3], param_q[0], param_q[1], param_q[2]);
            odom_.linear() = q_new.toRotationMatrix();
            odom_.translation() = Eigen::Vector3d(param_t[0], param_t[1], param_t[2]);
          }      

          if(params->use_icp_optimization_) {
          pose_history_.push_back(odom_);

          // Only run alignment when pose history reaches full size and ICP optimization is enabled
            if ( pose_history_.size() == params->pose_history_size_) {
            // Store the old translation before any changes
              alignTrajectoryToLane();
              if  (pose_history_.size() > 0)
                pose_history_.pop_front();
              
              } 
          } 
        
        // Compute the position of the detectd edges according to the final estimate position
          PointCloud::Ptr edges_map(new PointCloud);
          pcl::transformPointCloud(*feats, *edges_map, odom_.matrix());

          // Save edges and update odometry window
          lmap_manager.addPointCloud(edges_map);



          auto end_t = Clock::now();

          double now_secs = nh_->now().seconds();
          mean_in_freq_ -= in_freqs_[num_freqs_];
          in_freqs_[num_freqs_] = (1.0/(rclcpp::Time(feat_header.stamp).seconds() - last_in_time_secs_))/5.0;
          mean_in_freq_ += in_freqs_[num_freqs_];
          last_in_time_secs_ = rclcpp::Time(feat_header.stamp).seconds();

          mean_out_freq_ -= out_freqs_[num_freqs_];
          out_freqs_[num_freqs_] = (1.0/(now_secs - last_out_time_secs_))/5.0;
          mean_out_freq_ += out_freqs_[num_freqs_];
          last_out_time_secs_ = now_secs;

          num_freqs_ ++;
          num_freqs_ = num_freqs_ % 5;

          RCLCPP_DEBUG(nh_->get_logger(), "Input frequency: %2.2f", mean_in_freq_);

          if (mean_out_freq_ < mean_in_freq_ * 0.8) {
            RCLCPP_WARN(nh_->get_logger(), "Output frequency too low: %2.2f (in: %2.2f)", mean_out_freq_, mean_in_freq_);
          }

          // Register stats
          if (params->save_results_) {
            stats->addPose(odom_.matrix());
            stats->addLaserOdometryTime(start_t, end_t);
            stats->stopFrame(end_t);
          }

          publishOdom(feat_header, odom_);
          prev_stamp_ = rclcpp::Time(feat_header.stamp).seconds();
        }
      } 

      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void LaserOdometer::computeLocalMap(PointCloud::Ptr& local_map_gen, PointCloud::Ptr& local_map_rec) {
    
    PointCloud::Ptr rec_local_map_(new PointCloud);
    sdata->getLocalMap(rec_local_map_);
    local_map_rec = rec_local_map_;
    RCLCPP_DEBUG(nh_->get_logger(), "Local Map points - Received: %lu", rec_local_map_->size());

    PointCloud::Ptr total_points;
    size_t nframes = lmap_manager.getLocalMap(total_points);

    PointCloud::Ptr gen_local_map_(new PointCloud);
    if (params->filter_local_map_ && nframes == params->local_map_size_ && !params->mapping_) {
      // Voxelize the points
      pcl::VoxelGrid<Point> voxel_filter;
      // voxel_filter.setLeafSize(0.15, 0.15, 0.15);
      voxel_filter.setLeafSize(0.4, 0.4, 0.4);
      voxel_filter.setInputCloud(total_points);
      voxel_filter.filter(*gen_local_map_);
    } else {
      gen_local_map_ = total_points;
    }
    local_map_gen = gen_local_map_;
    RCLCPP_DEBUG(nh_->get_logger(), "Local Map points - Generated: %lu", gen_local_map_->size());
  }

  void LaserOdometer::alignTrajectoryToLane() {

  
      std::vector<Eigen::Vector2d> trajectory_points = extractTrajectoryPoints(pose_history_);
      // // Use normal shooting if enabled, otherwise use closest points directly
      std::vector<Eigen::Vector2d> best_lane_points;
      if (params->use_normal_shooting_) {
            std::vector<std::vector<size_t>> knn_indices = findKClosestNeighborsForPointsKdTree(trajectory_points, params->knn_neighbors_);
            best_lane_points = findNormalShootingFromKnn(trajectory_points, knn_indices, lane_points);
            
      } else {
          best_lane_points = findClosestLanePoints(pose_history_);
      }


      // Filter out invalid correspondences (if any)
      std::vector<Eigen::Vector2d> valid_trajectory_points;
      std::vector<Eigen::Vector2d> valid_lane_points;
      
      for (size_t i = 0; i < best_lane_points.size(); ++i) {
          // Check if the lane point is valid (not NaN, invalid, or no correspondence)

          // Skip points that are NaN or quiet_NaN
          if (std::isfinite(best_lane_points[i].x()) && std::isfinite(best_lane_points[i].y()) &&
              !std::isnan(best_lane_points[i].x()) && !std::isnan(best_lane_points[i].y())) {
              valid_trajectory_points.push_back(trajectory_points[i]);
              valid_lane_points.push_back(best_lane_points[i]);
          }
      }

      
        RCLCPP_INFO(nh_->get_logger(), "Found %zu valid correspondences out of %zu trajectory points", 
                  valid_trajectory_points.size(), trajectory_points.size());
      // Only proceed if we have enough valid correspondences
      if (valid_trajectory_points.size() < static_cast<size_t>(0.6 * params->pose_history_size_)) {
          RCLCPP_WARN(nh_->get_logger(), "Not enough valid correspondences for ICP alignment");
          return;
      }
      
  

      // Apply 2D ICP using the new solver method
      auto [R_total, t_total, mean_error] = solveIcp2d(valid_trajectory_points, valid_lane_points);


      // Calculate average distance from original traj to closest point in lane_points (original error)
      double orig_error = 0.0;
      for (int i = 0; i < valid_trajectory_points.size(); i++) {
        Eigen::Vector2d traj_point = valid_trajectory_points[i];
        Eigen::Vector2d lane_point = valid_lane_points[i];
        double d = (traj_point - lane_point).norm();
        orig_error += d;
      }
      orig_error /= valid_trajectory_points.size();

      // Calculate average distance from transformed traj to closest point in lane_points (ICP error)
      double icp_error = 0.0;
      for (int i = 0; i < valid_trajectory_points.size(); i++) {
        Eigen::Vector2d transformed_point = R_total * valid_trajectory_points[i] + t_total;
        Eigen::Vector2d lane_point = valid_lane_points[i];
        double d = (transformed_point - lane_point).norm();
        icp_error += d;
      }
      icp_error /= valid_trajectory_points.size();

      RCLCPP_INFO(nh_->get_logger(),"ICP error %f,  orig error %f", icp_error, orig_error );

      if (icp_error < params->icp_error_threshold_) {
          
          // Update local map with the same transformation
          // Re-populate pose_history_ with the valid trajectory points, transformed by R_total and t_total
          std::deque<Eigen::Isometry3d> aux_pose_history;
        
          for (size_t i = 10; i < pose_history_.size(); ++i ) {
              auto aux_pose = pose_history_[i];
          // Skip points that are NaN or quiet_NaN
          if (std::isfinite(best_lane_points[i].x()) && std::isfinite(best_lane_points[i].y()) &&
              !std::isnan(best_lane_points[i].x()) && !std::isnan(best_lane_points[i].y())) {
                Eigen::Vector2d aux_translation(aux_pose.translation().x(), aux_pose.translation().y());
                Eigen::Vector2d transformed_translation = R_total * aux_translation + t_total; 
                aux_pose.translation() = Eigen::Vector3d(transformed_translation.x(), transformed_translation.y(),aux_pose.translation().z());
                aux_pose_history.push_back(aux_pose);
              }
          }
          pose_history_ = std::move(aux_pose_history);
          
                    // Update odometry translation
          odom_.translation().x() = pose_history_[pose_history_.size()-1].translation().x();
          odom_.translation().y() = pose_history_[pose_history_.size()-1].translation().y();
          
          prev_odom_.translation().x() = pose_history_[pose_history_.size()-2].translation().x();
          prev_odom_.translation().y() = pose_history_[pose_history_.size()-2].translation().y();
          
          // pose_history_.clear();

          RCLCPP_INFO(nh_->get_logger(), "Applied ICP transformation to odometry and local map");
    }
  }

  std::tuple<Eigen::Matrix2d, Eigen::Vector2d, double> LaserOdometer::solveIcp2d(
      const std::vector<Eigen::Vector2d>& source_points,
      const std::vector<Eigen::Vector2d>& target_points,
      int max_iterations,
      double tolerance) {
    
    std::vector<Eigen::Vector2d> src_points = source_points;
    std::vector<Eigen::Vector2d> tgt_points = target_points;

    // ICP parameters
    double prev_error = std::numeric_limits<double>::max();
    size_t N = src_points.size();

    // Initialize transformation
    Eigen::Matrix2d R_total = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_total = Eigen::Vector2d::Zero();
    double mean_error = 0.0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Find closest target point for each source point (here, 1-to-1, so just use tgt_points)
        // Compute centroids
        Eigen::Vector2d centroid_src = Eigen::Vector2d::Zero();
        Eigen::Vector2d centroid_tgt = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < N; ++i) {
            centroid_src += src_points[i];
            centroid_tgt += tgt_points[i];
        }
        centroid_src /= static_cast<double>(N);
        centroid_tgt /= static_cast<double>(N);

        // Center the points
        std::vector<Eigen::Vector2d> src_centered(N), tgt_centered(N);
        for (size_t i = 0; i < N; ++i) {
            src_centered[i] = src_points[i] - centroid_src;
            tgt_centered[i] = tgt_points[i] - centroid_tgt;
        }

        // Compute cross-covariance
        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < N; ++i) {
            W += src_centered[i] * tgt_centered[i].transpose();
        }

        // SVD for optimal rotation
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        Eigen::Matrix2d R = V * U.transpose();

        // Ensure proper rotation (determinant = 1)
        if (R.determinant() < 0) {
            V.col(1) *= -1;
            R = V * U.transpose();
        }
        
        // Compute translation
        Eigen::Vector2d t = centroid_tgt - R * centroid_src;

        // Update cumulative transformation
        R_total = R * R_total;
        t_total = R * t_total + t;

        // Apply transformation to src_points for next iteration
        for (size_t i = 0; i < N; ++i) {
            src_points[i] = R * src_points[i] + t;
        }

        // Compute mean error
        mean_error = 0.0;
        for (size_t i = 0; i < N; ++i) {
            mean_error += (src_points[i] - tgt_points[i]).norm();
        }
        mean_error /= static_cast<double>(N);

        if (std::abs(prev_error - mean_error) < tolerance) {
            break;
        }
        prev_error = mean_error;
    }

    return std::make_tuple(R_total, t_total, mean_error);
  }


  std::vector<Eigen::Vector2d> LaserOdometer::findClosestLanePoints(const std::deque<Eigen::Isometry3d>& pose_history) {
      std::vector<Eigen::Vector2d> best_lane_points(pose_history.size());
      std::vector<bool> lane_point_used(lane_points.size(), false);
      
      // Process points in reverse order to prioritize latest points
      for (int i = static_cast<int>(pose_history.size()) - 1; i >= 0; --i) {
          Eigen::Vector2d traj_point(pose_history[i].translation().x(), pose_history[i].translation().y());
          
          double best_dist = std::numeric_limits<double>::max();
          int best_idx = -1;
          
          // Find closest unused lane point
          for (size_t j = 0; j < lane_points.size(); ++j) {
              if (lane_point_used[j]) continue;
              double dist = (traj_point - lane_points[j]).norm();
              if (dist < best_dist) {
                  best_dist = dist;
                  best_idx = static_cast<int>(j);
              }
          }
          
          // Assign the closest lane point
          if (best_idx >= 0) {
              best_lane_points[i] = lane_points[best_idx];
              lane_point_used[best_idx] = true; // Mark as used
          } else {
              // If no unused lane point found, use the closest one (even if used)
              best_dist = std::numeric_limits<double>::max();
              for (size_t j = 0; j < lane_points.size(); ++j) {
                  double dist = (traj_point - lane_points[j]).norm();
                  if (dist < best_dist) {
                      best_dist = dist;
                      best_idx = static_cast<int>(j);
                  }
              }
              best_lane_points[i] = lane_points[best_idx];
          }
      }
      
      return best_lane_points;
  }

  std::vector<Eigen::Vector2d> LaserOdometer::findNormalShootingCorrespondences(const std::vector<Eigen::Vector2d>& trajectory_points, 
                                                                                const std::vector<Eigen::Vector2d>& closest_lane_points) {
      std::vector<Eigen::Vector2d> correspondences(trajectory_points.size());
      std::vector<bool> lane_point_used(closest_lane_points.size(), false);
      
      // Process points in reverse order to prioritize latest points
      for (int i = static_cast<int>(trajectory_points.size()) - 1; i >= 0; --i) {
          Eigen::Vector2d p = trajectory_points[i];
          
          // Estimate tangent: use previous and next point if possible
          Eigen::Vector2d tangent;
          if (i > 0 && i < static_cast<int>(trajectory_points.size()) - 1) {
              tangent = trajectory_points[i+1] - trajectory_points[i-1];
          } else if (i < static_cast<int>(trajectory_points.size()) - 1) {
              tangent = trajectory_points[i+1] - p;
          } else if (i > 0) {
              tangent = p - trajectory_points[i-1];
          } else {
              tangent = Eigen::Vector2d(1.0, 0.0); // Default direction
          }
          
          tangent = tangent / (tangent.norm() + 1e-8);
          // Normal is perpendicular to tangent
          Eigen::Vector2d normal(-tangent.y(), tangent.x());
          
          // For each map segment, check for intersection with the normal line
          double min_dist = std::numeric_limits<double>::max();
          int best_idx = -1;
          Eigen::Vector2d best_proj;
          
          std::vector<int> available_indices;
          for (size_t j = 0; j < closest_lane_points.size(); ++j) {
              if (!lane_point_used[j]) {
                  available_indices.push_back(j);
              }
          }
          
          if (available_indices.size() < 2) {
              correspondences[i] = closest_lane_points[0]; // Fallback
              continue;
          }
          
          for (size_t j = 0; j < available_indices.size() - 1; ++j) {
              int idx1 = available_indices[j];
              int idx2 = available_indices[j+1];
              Eigen::Vector2d a = closest_lane_points[idx1];
              Eigen::Vector2d b = closest_lane_points[idx2];
              
              // Line segment ab, normal line through p in direction 'normal'
              // Solve for intersection: a + t*(b-a) = p + s*normal
              Eigen::Matrix2d A;
              A.col(0) = b - a;
              A.col(1) = -normal;
              
              if (A.determinant() < 1e-8) {
                  continue; // Parallel, skip
              }
              
              Eigen::Vector2d sol = A.inverse() * (p - a);
              double t = sol(0);
              double s = sol(1);
              
              if (t >= 0 && t <= 1) {
                  Eigen::Vector2d intersection = a + t * (b - a);
                  double dist = (intersection - p).norm();
                  if (dist < min_dist) {
                      min_dist = dist;
                      best_idx = (dist < (a - intersection).norm()) ? idx1 : idx2;
                      best_proj = intersection;
                  }
              }
          }
          
          if (best_idx >= 0) {
              correspondences[i] = closest_lane_points[best_idx];
              lane_point_used[best_idx] = true; // Mark as used
          } else {
              // Fallback to closest point if no intersection found
              double best_dist = std::numeric_limits<double>::max();
              for (size_t j = 0; j < closest_lane_points.size(); ++j) {
                  if (!lane_point_used[j]) {
                      double dist = (p - closest_lane_points[j]).norm();
                      if (dist < best_dist) {
                          best_dist = dist;
                          best_idx = static_cast<int>(j);
                      }
                  }
              }
              if (best_idx >= 0) {
                  correspondences[i] = closest_lane_points[best_idx];
                  lane_point_used[best_idx] = true;
              } else {
                  correspondences[i] = closest_lane_points[0]; // Final fallback
              }
          }
      }
      
      return correspondences;
  }

  void LaserOdometer::addEdgeConstraints(const PointCloud::Ptr& edges,
                          const PointCloud::Ptr& local_map_gen,
                          const PointCloud::Ptr& local_map_rec,
                          const Eigen::Isometry3d& pose,
                          ceres::Problem* problem,
                          ceres::LossFunction* loss) {
    // Translate edges
    PointCloud::Ptr edges_map(new PointCloud);
    pcl::transformPointCloud(*edges, *edges_map, pose.matrix());
    
    PointCloud::Ptr local_map(new PointCloud);
    *local_map += *local_map_gen;
    if (params->mapping_) {
      *local_map += *local_map_rec;
    }

    // Trying to match against the received local map
    int correct_matches = 0;
    pcl::KdTreeFLANN<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>);
    tree->setInputCloud(local_map);
    for (size_t i = 0; i < edges_map->points.size(); i++) {
      std::vector<int> indices;
      std::vector<float> sq_dist;
      tree->nearestKSearch(edges_map->points[i], 5, indices, sq_dist);
      if (sq_dist[4] < 1.0) {
        std::vector<Eigen::Vector3d> nearCorners;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(local_map->points[indices[j]].x,
                              local_map->points[indices[j]].y,
                              local_map->points[indices[j]].z);
          center = center + tmp;
          nearCorners.push_back(tmp);
        }
        center = center / 5.0;

        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
        
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          // Set a correct match
          correct_matches++;
          Eigen::Vector3d curr_point(edges->points[i].x,
                                    edges->points[i].y,
                                    edges->points[i].z);

          Eigen::Vector3d pt_a(local_map->points[indices[0]].x,
                              local_map->points[indices[0]].y,
                              local_map->points[indices[0]].z);
        
          Eigen::Vector3d pt_b(local_map->points[indices[1]].x,
                              local_map->points[indices[1]].y,
                              local_map->points[indices[1]].z);

          ceres::CostFunction* cost_function = Point2LineFactor::create(curr_point, pt_a, pt_b, params->min_range_, params->max_range_);
          problem->AddResidualBlock(cost_function, loss, param_q, param_t);
        }
      }
    }

    RCLCPP_DEBUG(nh_->get_logger(), "Correct matchings: %i", correct_matches);
  }

  bool LaserOdometer::getBaseToLaserTf (const std::string& frame_id) {

    geometry_msgs::msg::TransformStamped laser_to_base_tf;
    try {
      rclcpp::Time now = nh_->get_clock()->now();
      laser_to_base_tf = tf_buffer_->lookupTransform(
        params->base_frame_, // target frame
        frame_id,  // source frame
        now,
        rclcpp::Duration(2.0, 0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(nh_->get_logger(), "Could not get initial transform from %s to %s: %s", frame_id.c_str(), params->base_frame_.c_str(), ex.what());
      return false;
    }

    laser_to_base_.translation() = Eigen::Vector3d(laser_to_base_tf.transform.translation.x,
                                                  laser_to_base_tf.transform.translation.y,
                                                  laser_to_base_tf.transform.translation.z);
    Eigen::Quaterniond q_aux(laser_to_base_tf.transform.rotation.w,
                            laser_to_base_tf.transform.rotation.x,
                            laser_to_base_tf.transform.rotation.y,
                            laser_to_base_tf.transform.rotation.z);
    laser_to_base_.linear() = q_aux.toRotationMatrix();

    return true;
  }

  void LaserOdometer::publishOdom(const std_msgs::msg::Header& header, const Eigen::Isometry3d& pose) {
    // Publishing odometry
    nav_msgs::msg::Odometry laser_odom_msg;
    laser_odom_msg.header.frame_id = params->fixed_frame_;
    laser_odom_msg.child_frame_id = params->base_frame_;
    laser_odom_msg.header.stamp = header.stamp;
    // Transform to base_link frame before publication
    Eigen::Isometry3d odom_base_link = laser_to_base_* pose ;
    Eigen::Quaterniond q_current(odom_base_link.rotation());
    q_current.normalize();

    Eigen::Vector3d t_current = odom_base_link.translation();
    //Filling pose
    laser_odom_msg.pose.pose.orientation.x = q_current.x();
    laser_odom_msg.pose.pose.orientation.y = q_current.y();
    laser_odom_msg.pose.pose.orientation.z = q_current.z();
    laser_odom_msg.pose.pose.orientation.w = q_current.w();
    laser_odom_msg.pose.pose.position.x = t_current.x();
    laser_odom_msg.pose.pose.position.y = t_current.y();
    laser_odom_msg.pose.pose.position.z = t_current.z();
    //Filling twist
    double delta_time = rclcpp::Time(header.stamp).seconds() - prev_stamp_;
    Eigen::Isometry3d delta_odom = ((laser_to_base_*prev_odom_ ).inverse() * odom_base_link);
    Eigen::Vector3d t_delta = delta_odom.translation();
    laser_odom_msg.twist.twist.linear.x = t_delta.x() / delta_time;
    laser_odom_msg.twist.twist.linear.y = t_delta.y() / delta_time;
    laser_odom_msg.twist.twist.linear.z = t_delta.z() / delta_time;
    Eigen::Quaterniond q_delta(delta_odom.rotation());
    // we use tf because euler angles in Eigen present singularity problems
    tf2::Quaternion quat(q_delta.x(), q_delta.y(), q_delta.z(), q_delta.w());
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    laser_odom_msg.twist.twist.angular.x = roll / delta_time;
    laser_odom_msg.twist.twist.angular.y = pitch / delta_time;
    laser_odom_msg.twist.twist.angular.z = yaw / delta_time;
    odom_pub_->publish(laser_odom_msg);

    // Publishing twist
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.frame_id = params->base_frame_;
    twist_msg.header.stamp = header.stamp;
    twist_msg.twist = laser_odom_msg.twist.twist;
    twist_pub_->publish(twist_msg);

    //Publishing TF
    if (params->publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;

      // corresponding tf variables
      transform.header.stamp = header.stamp;
      transform.header.frame_id = params->fixed_frame_;
      transform.child_frame_id = params->base_frame_;
    
      transform.transform.translation.x = t_current.x();
      transform.transform.translation.y = t_current.y();
      transform.transform.translation.z = t_current.z();
      transform.transform.rotation.x = q_current.x();
      transform.transform.rotation.y = q_current.y();
      transform.transform.rotation.z = q_current.z();
      transform.transform.rotation.w = q_current.w();

      tf_broadcaster_->sendTransform(transform);
    }
  }

  /*
   * findKClosestNeighborsForPoints - Finds k closest neighbors for each point
   * 
   * Returns: std::vector<std::vector<size_t>> where:
   * - Outer vector has n elements (one for each trajectory point)
   * - Each inner vector has k elements (k closest map point indices for that trajectory point)
   * 
   * Usage example:
   * std::vector<Eigen::Vector2d> trajectory_points = extractTrajectoryPoints(pose_history);
   * std::vector<std::vector<size_t>> knn_indices = findKClosestNeighborsForPoints(lane_points, trajectory_points, 3);
   * // knn_indices[i][j] gives the index of the j-th closest map point for trajectory point i
   */
  std::vector<std::vector<size_t>> LaserOdometer::findKClosestNeighborsForPoints(
    const std::vector<Eigen::Vector2d>& map_points,
    const std::vector<Eigen::Vector2d>& points,
    int k) {
    
    std::vector<std::vector<size_t>> knn_results(points.size());
    
    // For each trajectory point, find k closest map point indices
    for (size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector2d& query_point = points[i];
      
      // Calculate distances to all map points
      std::vector<std::pair<double, size_t>> distances;
      for (size_t j = 0; j < map_points.size(); ++j) {
        double dist = (query_point - map_points[j]).norm();
        distances.emplace_back(dist, j);
      }
      
      // Sort by distance and take the k closest
      std::sort(distances.begin(), distances.end());
      int num_neighbors = std::min(k, static_cast<int>(distances.size()));
      
      // Store the k closest indices
      knn_results[i].reserve(num_neighbors);
      for (int j = 0; j < num_neighbors; ++j) {
        size_t map_idx = distances[j].second;
        knn_results[i].push_back(map_idx);
      }
    }
    
    return knn_results;
  }

  /*
   * findKClosestNeighborsForPointsKdTree - Finds k closest neighbors for each point using pre-built PCL KD-tree
   * Optimized for 2D lane points (z=0 always)
   * 
   * Returns: std::vector<std::vector<size_t>> where:
   * - Outer vector has n elements (one for each trajectory point)
   * - Each inner vector has k elements (k closest map point indices for that trajectory point)
   * 
   * Usage example:
   * std::vector<Eigen::Vector2d> trajectory_points = extractTrajectoryPoints(pose_history);
   * std::vector<std::vector<size_t>> knn_indices = findKClosestNeighborsForPointsKdTree(trajectory_points, 3);
   * // knn_indices[i][j] gives the index of the j-th closest map point for trajectory point i
   */
  std::vector<std::vector<size_t>> LaserOdometer::findKClosestNeighborsForPointsKdTree(
    const std::vector<Eigen::Vector2d>& points,
    int k) {
    
    std::vector<std::vector<size_t>> knn_results(points.size());
    
    // For each trajectory point, find k closest map point indices using pre-built KD-tree
    for (size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector2d& query_point = points[i];
      
      // Create PCL point for query (2D only, z=0)
      Point query_pcl_point;
      query_pcl_point.x = query_point.x();
      query_pcl_point.y = query_point.y();
      query_pcl_point.z = 0.0;  // Z is always 0 for 2D lane matching
      
      // Find k nearest neighbors using pre-built KD-tree
      std::vector<int> indices;
      std::vector<float> sq_distances;
      lane_kdtree_->nearestKSearch(query_pcl_point, k, indices, sq_distances);
      
      // Store the k closest indices
      knn_results[i].reserve(indices.size());
      for (size_t j = 0; j < indices.size(); ++j) {
        knn_results[i].push_back(static_cast<size_t>(indices[j]));
      }
    }
    
    return knn_results;
  }

  /*
   /**
    * findNormalShootingFromKnn - Improved normal shooting using KNN and map neighbor segments.
    *
    * For each trajectory point, iterate over its KNNs. For each KNN, use the map point and its
    * immediate neighbors in the map to form segments, and check intersection with the normal
    * projected from the trajectory point. Returns the best intersection or indicates no correspondence.
    */
  std::vector<Eigen::Vector2d> LaserOdometer::findNormalShootingFromKnn(
    const std::vector<Eigen::Vector2d>& trajectory_points,
    const std::vector<std::vector<size_t>>& knn_indices,
    const std::vector<Eigen::Vector2d>& map_points) {

    std::vector<Eigen::Vector2d> correspondences(trajectory_points.size());

    for (size_t i = 0; i < trajectory_points.size(); ++i) {
      const Eigen::Vector2d& p = trajectory_points[i];

      // Estimate tangent direction from trajectory
      Eigen::Vector2d tangent;
      if (i > 0 && i < trajectory_points.size() - 1) {
        tangent = trajectory_points[i + 1] - trajectory_points[i - 1];
      } else if (i < trajectory_points.size() - 1) {
        tangent = trajectory_points[i + 1] - p;
      } else if (i > 0) {
        tangent = p - trajectory_points[i - 1];
      } else {
        tangent = Eigen::Vector2d(1.0, 0.0); // Default direction
      }
      tangent.normalize();
      Eigen::Vector2d normal(-tangent.y(), tangent.x()); // Perpendicular to tangent

      double min_dist = std::numeric_limits<double>::max();
      Eigen::Vector2d best_point = Eigen::Vector2d::Zero();
      bool found_valid = false;

      // For each KNN index, try to use its neighbors in the map to form a segment
      const std::vector<size_t>& knn_idx = knn_indices[i];
      for (size_t j = 0; j < knn_idx.size(); ++j) {
        size_t map_idx = knn_idx[j];

        // Try previous neighbor
        if (map_idx > 0) {
          const Eigen::Vector2d& a = map_points[map_idx - 1];
          const Eigen::Vector2d& b = map_points[map_idx];
          Eigen::Vector2d ab = b - a;

          Eigen::Matrix2d A;
          A.col(0) = ab;
          A.col(1) = -normal;

          if (std::abs(A.determinant()) > 1e-10) {
            Eigen::Vector2d sol = A.inverse() * (p - a);
            double t = sol(0);
            double s = sol(1);

            if (t >= 0.0 && t <= 1.0) {
              double dist = std::abs(s);
              if (dist < min_dist) {
                min_dist = dist;
                Eigen::Vector2d proj = a + t * ab;
                best_point = proj;
                found_valid = true;
              }
            }
          }
        }

        // Try next neighbor
        if (map_idx + 1 < map_points.size()) {
          const Eigen::Vector2d& a = map_points[map_idx];
          const Eigen::Vector2d& b = map_points[map_idx + 1];
          Eigen::Vector2d ab = b - a;

          Eigen::Matrix2d A;
          A.col(0) = ab;
          A.col(1) = -normal;

          if (std::abs(A.determinant()) > 1e-10) {
            Eigen::Vector2d sol = A.inverse() * (p - a);
            double t = sol(0);
            double s = sol(1);

            if (t >= 0.0 && t <= 1.0) {
              double dist = std::abs(s);
              if (dist < min_dist) {
                min_dist = dist;
                Eigen::Vector2d proj = a + t * ab;
                best_point = proj;
                found_valid = true;
              }
            }
          }
        }
      }

      if (found_valid) {
        correspondences[i] = best_point;
      } else {
        correspondences[i] = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(),
                                             std::numeric_limits<double>::quiet_NaN());
      }
    }

    return correspondences;
  }

  std::vector<Eigen::Vector2d> LaserOdometer::extractTrajectoryPoints(const std::deque<Eigen::Isometry3d>& pose_history) {
    std::vector<Eigen::Vector2d> trajectory_points;
    trajectory_points.reserve(pose_history.size());
    
    for (const auto& pose : pose_history) {
      Eigen::Vector2d point(pose.translation().x(), pose.translation().y());
      trajectory_points.push_back(point);
    }
    
    return trajectory_points;
  }

}  // namespace liodom
