#pragma once

#include <ros/ros.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

class FusedPointcloud {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    FusedPointcloud() : kf_id_anchor_(),
                        anchor_timestamp_(),
                        included_kf_ids_(),
                        T_O_S_anchor_(),
                        fused_pcl_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>){};
    ~FusedPointcloud() {};

    void setNewAnchor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_anchor,
                    const Eigen::Matrix4d& T_O_S_anchor,
                    const uint64_t kf_id_anchor,
                    const double &timestamp);

    void addPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                     const Eigen::Matrix4d& T_O_S_i,
                     const uint64_t kf_id);

    uint64_t getNumKfs() { return included_kf_ids_.size(); }

    uint64_t getAnchorId() { return kf_id_anchor_; }

    Eigen::Matrix4d getAnchorTransform() { return T_O_S_anchor_; }

    double getAnchorTimestamp() { return anchor_timestamp_; }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFusedPCLCloud() {
        return fused_pcl_cloud_;
    }

    void filterFusedPCLCloud();



protected:
    uint64_t kf_id_anchor_;

    double anchor_timestamp_;
    
    std::vector<uint64_t> included_kf_ids_;

    Eigen::Matrix4d T_O_S_anchor_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  fused_pcl_cloud_;
};
