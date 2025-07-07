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

#ifndef INCLUDE_LIODOM_FACTORS_HPP
#define INCLUDE_LIODOM_FACTORS_HPP

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace liodom {

struct Point2PointFactor {

  Point2PointFactor(Eigen::Vector3d curr_point, Eigen::Vector3d map_point) :
    curr_point_(curr_point), map_point_(map_point) {};

  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {

    Eigen::Matrix<T, 3, 1> cp{T(curr_point_.x()), T(curr_point_.y()), T(curr_point_.z())};
    Eigen::Matrix<T, 3, 1> mp{T(map_point_.x()), T(map_point_.y()), T(map_point_.z())};

		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};		
		Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

    Eigen::Matrix<T, 3, 1> error = lp - mp;

		residual[0] = T(error.x());
		residual[1] = T(error.y());
		residual[2] = T(error.z());
    
    return true;
  }

  static ceres::CostFunction* create(const Eigen::Vector3d curr_point, const Eigen::Vector3d map_point) {
    return new ceres::AutoDiffCostFunction<Point2PointFactor, 3, 4, 3>(new Point2PointFactor(curr_point, map_point));
	}
  
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d map_point_;
};


struct Point2LineFactor {

	Point2LineFactor(Eigen::Vector3d curr_point, Eigen::Vector3d map_point_a, Eigen::Vector3d map_point_b, 
		double min_d, double max_d) :
    curr_point_(curr_point), map_point_a_(map_point_a), map_point_b_(map_point_b),
		min_dist_(min_d), max_dist_(max_d) {};

	template <typename T>
	bool operator()(const T* q, const T* t, T* residual) const {
    
		Eigen::Matrix<T, 3, 1> cp{T(curr_point_.x()), T(curr_point_.y()), T(curr_point_.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(map_point_a_.x()), T(map_point_a_.y()), T(map_point_a_.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(map_point_b_.x()), T(map_point_b_.y()), T(map_point_b_.z())};
		
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(1), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(1) * t[0], T(1) * t[1], T(1) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		Eigen::Matrix<T, 3, 1> cp_l{T(curr_point_.x() - t[0]), T(curr_point_.y() - t[1]), T(curr_point_.z() - t[2])};

		T d = ceres::sqrt(
			cp_l.x() * cp_l.x() + cp_l.y() * cp_l.y()
		);
		d = (d - T(min_dist_)) / (T(max_dist_) - T(min_dist_)); // Normalize distance

		//T w = ceres::exp(d);
		// T w_z = ceres::pow(T(10.0), -d);
		T w = T(1.01) - d;

		residual[0] = w * (nu.x() / de.norm());
		residual[1] = w * (nu.y() / de.norm());
		residual[2] = w * (nu.z() / de.norm());

		return true;
	}

	static ceres::CostFunction* create(const Eigen::Vector3d curr_point, 
                                     const Eigen::Vector3d map_point_a,
                                     const Eigen::Vector3d map_point_b,
																		 const double min_range_,
																		 const double max_range_) {
		return new ceres::AutoDiffCostFunction<Point2LineFactor, 3, 4, 3>(new Point2LineFactor(curr_point, map_point_a, map_point_b, min_range_, max_range_));
	}

	Eigen::Vector3d curr_point_;
  Eigen::Vector3d map_point_a_;
  Eigen::Vector3d map_point_b_;

	double min_dist_;
	double max_dist_;
};

struct LaneletFactor {
  LaneletFactor(const Eigen::Vector2d& closest_point) 
    : closest_point_(closest_point) {}

  template<typename T>
  bool operator()(const T* t, T* residual) const {
    // Apply 35-degree rotation to current position
    // T rotation_angle = T(35.0 * M_PI / 180.0);
    // T cos_angle = ceres::cos(rotation_angle);
    // T sin_angle = ceres::sin(rotation_angle);
    
    // T px = t[0] * cos_angle - t[1] * sin_angle;
    // T py = t[0] * sin_angle + t[1] * cos_angle;
    
    T px = t[0];
    T py = t[1];
    // Closest lane point
    T cx = T(closest_point_.x());
    T cy = T(closest_point_.y());
    
    // Calculate distance between current position and closest lane point
    T dx = px - cx;
    T dy = py - cy;
    T distance = ceres::sqrt(dx * dx + dy * dy);
    
    // Return distance as residual
    residual[0] = distance;
    
    return true;
  }

	static ceres::CostFunction* create(const Eigen::Vector2d& closest_point) {
    return new ceres::AutoDiffCostFunction<LaneletFactor, 1, 3>(
      new LaneletFactor(closest_point));
  }

  Eigen::Vector2d closest_point_;
};
}  // namespace liodom

#endif // INCLUDE_LIODOM_FACTORS_HPP