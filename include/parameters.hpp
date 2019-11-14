/*
* Copyright (c) 2018, Vision for Robotics Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/*
 * parameters.hpp
 * @brief Storage for the system parameters.
 * @author: Marco Karrer
 * Created on: Aug 13, 2018
 */

#pragma once

#include <deque>
#include <vector>
#include <Eigen/Dense>
#include <aslam/cameras/ncamera.h>

#include "typedefs.hpp"
#include "pose_graph_backend/brisk-vocabulary.hpp"

/// \brief pgbe Main namespace of this package.
namespace  pgbe {

/// @brief Camera parameters.
struct CameraParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraParameters() :
    agent_id(0),
    camera(NULL) {}
  CameraParameters(const uint64_t& agent_id_,
                   const std::string& cam_file) :
    agent_id(agent_id_),
    camera(aslam::NCamera::loadFromYaml(cam_file)) {}
  aslam::NCamera::Ptr camera;
  uint64_t agent_id;
};

typedef std::vector<CameraParameters, Eigen::aligned_allocator<
  CameraParameters>> CameraParametersVector;

/// @brief GPS parameters
struct GpsParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GpsParameters() :
    agent_id(0),
    local_reference(Eigen::Vector3d::Zero()),
    offset(Eigen::Vector3d::Zero()){}
  GpsParameters(const uint64_t& agent_id_,
                const Eigen::Vector3d& local_reference_,
                const Eigen::Vector3d& offset_) :
    agent_id(agent_id_),
    local_reference(local_reference_),
    offset(offset_) {}
  uint64_t agent_id;
  Eigen::Vector3d local_reference; // [latitude, longitude, altitude]
  Eigen::Vector3d offset;
};

typedef std::vector<GpsParameters, Eigen::aligned_allocator<
    GpsParameters>> GpsParametersVector;

/// @brief System Parameters
struct SystemParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SystemParameters() :
    camera_parameters(CameraParametersVector()),
    gps_parameters(GpsParametersVector()),
    num_agents(0),
    loop_candidate_min_score(0.01),
    loop_image_min_matches(50),
    loop_detect_sac_thresh(25),
    loop_detect_sac_max_iter(300),
    loop_detect_min_sac_inliers(10),
    loop_detect_min_sac_inv_inliers(10),
    loop_detect_min_pose_inliers(12),
    rel_pose_outlier_norm_min(12),
    loop_detect_reset_time(6),
    max_loop_candidates(5),
    gps_align_num_corr(5),
    gps_align_cov_max(1.0),
    loop_detect_skip_kf(2),
    gps_active(std::vector<bool>(1,0)) {}
  SystemParameters(const size_t num_agents_,
                   const CameraParametersVector& cam_vector_,
                   const GpsParametersVector& gps_parameters_,
                   const double loop_candidate_min_score_,
                   const int loop_image_min_matches_,
                   const int loop_detect_sac_thresh_,
                   const int loop_detect_sac_max_iter_,
                   const int loop_detect_min_sac_inliers_,
                   const int loop_detect_min_sac_inv_inliers_,
                   const int loop_detect_min_pose_inliers_,
                   const double rel_pose_outlier_norm_min_,
                   const double loop_detect_reset_time_,
                   const int max_loop_candidates_,
                   const int gps_align_num_corr_,
                   const double gps_align_cov_max_,
                   const int loop_detect_skip_kf_,
                   const std::vector<bool> gps_active_):
    num_agents(num_agents_), camera_parameters(cam_vector_),
    gps_parameters(gps_parameters_), loop_candidate_min_score(loop_candidate_min_score_),
    loop_image_min_matches(loop_image_min_matches_), 
    loop_detect_sac_thresh(loop_detect_sac_thresh_),
    loop_detect_sac_max_iter(loop_detect_sac_max_iter_), 
    loop_detect_min_sac_inliers(loop_detect_min_sac_inliers_),
    loop_detect_min_sac_inv_inliers(loop_detect_min_sac_inv_inliers_),
    loop_detect_min_pose_inliers(loop_detect_min_pose_inliers_),
    rel_pose_outlier_norm_min(rel_pose_outlier_norm_min_),
    loop_detect_reset_time(loop_detect_reset_time_),
    max_loop_candidates(max_loop_candidates_),
    gps_align_num_corr(gps_align_num_corr_),
    gps_align_cov_max(gps_align_cov_max_),
    loop_detect_skip_kf(loop_detect_skip_kf_),
    gps_active(gps_active_) {}
    
  size_t num_agents;
  CameraParametersVector camera_parameters;
  GpsParametersVector gps_parameters;
  std::shared_ptr<BRISKVocabulary> voc_ptr;
  double loop_candidate_min_score;
  int loop_image_min_matches;
  int loop_detect_sac_thresh;
  int loop_detect_sac_max_iter;
  int loop_detect_min_sac_inliers;
  int loop_detect_min_sac_inv_inliers;
  int loop_detect_min_pose_inliers;
  double rel_pose_outlier_norm_min;
  double loop_detect_reset_time;
  int max_loop_candidates;
  int gps_align_num_corr;
  double gps_align_cov_max;
  std::vector<bool> gps_active;
  int loop_detect_skip_kf;
};


} // namespace pgbe
