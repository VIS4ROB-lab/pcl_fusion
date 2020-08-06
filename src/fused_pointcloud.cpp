#include "fused_pointcloud.hpp"
#include "pcl_fusion_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <pcl/filters/voxel_grid.h>

void FusedPointcloud::setNewAnchor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_anchor,
    const Eigen::Matrix4d& T_O_S_anchor,  const uint64_t kf_id_anchor, const double &timestamp){

    kf_id_anchor_ = kf_id_anchor;
    T_O_S_anchor_ = T_O_S_anchor;
    fused_pcl_cloud_ = pcl_cloud_anchor;
    anchor_timestamp_ = timestamp;
    included_kf_ids_.clear();
    included_kf_ids_.push_back(kf_id_anchor);
}


void FusedPointcloud::addPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
    const Eigen::Matrix4d& T_O_S_i, const uint64_t kf_id) {

    Eigen::Matrix4d T_O_C_anchor = T_O_S_anchor_ * T_S_C;
    Eigen::Matrix4d T_O_C_i = T_O_S_i * T_S_C;
    Eigen::Matrix4d T_Ca_Ci = T_O_C_anchor.inverse() * T_O_C_i;

    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    // Eigen::Matrix4d test = Eigen::Matrix4d::Identity();
    pcl::transformPointCloud(*pcl_cloud, transformed_cloud, T_Ca_Ci);
    *fused_pcl_cloud_ += transformed_cloud;


    // auto start = std::chrono::steady_clock::now();
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(fused_pcl_cloud_);
    // sor.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
    // sor.filter(*fused_pcl_cloud_filtered);
    // auto end = std::chrono::steady_clock::now();

    // fused_pcl_cloud_ = fused_pcl_cloud_filtered;
    // std::cout << "Filter time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

    included_kf_ids_.push_back(kf_id);
}

void FusedPointcloud::filterFusedPCLCloud() {
    
    auto start = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(fused_pcl_cloud_);
    sor.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
    sor.filter(*fused_pcl_cloud_filtered);
    auto end = std::chrono::steady_clock::now();

    fused_pcl_cloud_ = fused_pcl_cloud_filtered;
    std::cout << "Filter time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

}
