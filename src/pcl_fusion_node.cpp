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

#include <iostream>
#include <fstream>
#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <list>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <comm_msgs/keyframe.h>
#include <comm_msgs/fused_pcl.h>
#include "fused_pointcloud.hpp"
#include <chrono>
#include <pcl/filters/voxel_grid.h>

#include "pcl_conversions/pcl_conversions.h"
#include <aslam/cameras/ncamera.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

std::deque<sensor_msgs::PointCloud2ConstPtr> pcl_buf;
std::deque<comm_msgs::keyframeConstPtr> kf_buf;
std::mutex m_buf;

bool new_fused_cloud = true;
int kf_skip = 0;
FusedPointcloud fused_pcl_cloud;

int kf_per_cloud;
double voxel_filter_size;
int agent_id;
int num_odom_connections;
std::list<uint32_t> synced_kf_ids;

ros::Publisher pub_pcl;
ros::Publisher pub_pcl_raw;
ros::Publisher pub_transform;
ros::Publisher pub_keyframe;

aslam::NCamera::Ptr camera;
Eigen::Matrix4d T_S_C;

void process() {
    while (true) {
        sensor_msgs::PointCloud2ConstPtr pcl_msg = NULL;
        comm_msgs::keyframeConstPtr kf_msg = NULL;
        m_buf.lock();

        if (!kf_buf.empty() && !pcl_buf.empty()){
            if (kf_buf.front()->header.stamp.toSec() > pcl_buf.front()->header.stamp.toSec()){
                pcl_buf.pop_front();
                //std::cout << "Throw pcl at front" << std::endl;
            }
            else if (kf_buf.back()->header.stamp.toSec() >= pcl_buf.front()->header.stamp.toSec()){
                pcl_msg = pcl_buf.front();
                pcl_buf.pop_front();
                while (!pcl_buf.empty() && (pcl_buf.front()->header.stamp.toSec() < (pcl_msg->header.stamp.toSec() + 0.5)))
                    pcl_buf.pop_front();
                while (kf_buf.front()->header.stamp.toSec() < pcl_msg->header.stamp.toSec())
                    kf_buf.pop_front();
                kf_msg = kf_buf.front();
                kf_buf.pop_front();

                synced_kf_ids.push_back(kf_msg->frameId);
            }
        }

        m_buf.unlock();
        if (pcl_msg != NULL)
        {
            //std::cout << std::setprecision(14) << "KF Timestamp: " << kf_msg->header.stamp.toSec();
            //std::cout << "\t PCL Timestamp: " << pcl_msg->header.stamp.toSec() << std::endl;
            if (new_fused_cloud){
                // create new fused pcl
                //std::cout << "Creating new fused PCL Cloud" << std::endl;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_pcl_cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromROSMsg (*pcl_msg, *fused_pcl_cloud_tmp);

                // extract odometry 
                Eigen::Matrix4d T_O_S_anchor = Eigen::Matrix4d::Identity();
                T_O_S_anchor.block<3,3>(0,0) = Eigen::Quaterniond(
                            kf_msg->odometry.pose.pose.orientation.w,
                            kf_msg->odometry.pose.pose.orientation.x,
                            kf_msg->odometry.pose.pose.orientation.y,
                            kf_msg->odometry.pose.pose.orientation.z).toRotationMatrix();
                T_O_S_anchor(0,3) = kf_msg->odometry.pose.pose.position.x;
                T_O_S_anchor(1,3) = kf_msg->odometry.pose.pose.position.y;
                T_O_S_anchor(2,3) = kf_msg->odometry.pose.pose.position.z;

                // create fused pointcloud object
                fused_pcl_cloud.setNewAnchor(fused_pcl_cloud_tmp, T_O_S_anchor, kf_msg->frameId,
                    kf_msg->header.stamp.toSec());
                new_fused_cloud = false;
            }
            else {
                // extract pcl cloud from msg
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromROSMsg (*pcl_msg, *sub_pcl_cloud);

                // extract odometry
                Eigen::Matrix4d T_O_S_i = Eigen::Matrix4d::Identity();
                T_O_S_i.block<3,3>(0,0) = Eigen::Quaterniond(
                            kf_msg->odometry.pose.pose.orientation.w,
                            kf_msg->odometry.pose.pose.orientation.x,
                            kf_msg->odometry.pose.pose.orientation.y,
                            kf_msg->odometry.pose.pose.orientation.z).toRotationMatrix();
                T_O_S_i(0,3) = kf_msg->odometry.pose.pose.position.x;
                T_O_S_i(1,3) = kf_msg->odometry.pose.pose.position.y;
                T_O_S_i(2,3) = kf_msg->odometry.pose.pose.position.z;
								//std::cout << "Adding PCL with stamp: " << pcl_msg->header.stamp.toSec() << std::endl;
                fused_pcl_cloud.addPCLCloud(sub_pcl_cloud, T_O_S_i, kf_msg->frameId);
            }

            if (fused_pcl_cloud.getNumKfs() >= kf_per_cloud) {

                fused_pcl_cloud.filterFusedPCLCloud();

                //send transform to tf for rviz
                static tf::TransformBroadcaster tf_pub; 
                geometry_msgs::TransformStamped msg_T_O_S_anchor;
                msg_T_O_S_anchor.header.stamp = kf_msg->header.stamp;
                // msg_T_O_S_anchor.header.stamp = ros::Time::now();
                msg_T_O_S_anchor.header.frame_id = "odom_pcl";
                msg_T_O_S_anchor.child_frame_id = "cam_pcl";
                Eigen::Matrix4d T_O_S_anchor = fused_pcl_cloud.getAnchorTransform() * T_S_C;
                const Eigen::Quaterniond q_O_S_anchor(T_O_S_anchor.block<3,3>(0, 0));
                msg_T_O_S_anchor.transform.rotation.w = q_O_S_anchor.w();
                msg_T_O_S_anchor.transform.rotation.x = q_O_S_anchor.x();
                msg_T_O_S_anchor.transform.rotation.y = q_O_S_anchor.y();
                msg_T_O_S_anchor.transform.rotation.z = q_O_S_anchor.z();
                msg_T_O_S_anchor.transform.translation.x = T_O_S_anchor(0, 3);
                msg_T_O_S_anchor.transform.translation.y = T_O_S_anchor(1, 3);
                msg_T_O_S_anchor.transform.translation.z = T_O_S_anchor(2, 3);

                pub_transform.publish(msg_T_O_S_anchor);
                tf_pub.sendTransform(msg_T_O_S_anchor);

                // create fused pcl message to send to pose graph backend
                comm_msgs::fused_pcl fused_pcl_msg;
                ros::Time time_obj;
                fused_pcl_msg.header.stamp = kf_msg->header.stamp;

                sensor_msgs::PointCloud2 fused_cloud_msg_tmp;
                pcl::toROSMsg(*fused_pcl_cloud.getFusedPCLCloud(), fused_cloud_msg_tmp);

                fused_cloud_msg_tmp.header.frame_id = "cam_pcl";
                fused_cloud_msg_tmp.header.stamp = kf_msg->header.stamp;
                // fused_cloud_msg_tmp.header.stamp = ros::Time::now();
                fused_pcl_msg.fusedPointcloud = fused_cloud_msg_tmp;
                fused_pcl_msg.anchorId = fused_pcl_cloud.getAnchorId();

                pub_pcl.publish(fused_pcl_msg);
                pub_pcl_raw.publish(fused_cloud_msg_tmp);

								//std::cout << "Keyframe time stamp: " << kf_msg->header.stamp.toSec() << std::endl;
								//std::cout << "Last pointcloud time stamp: " << pcl_msg->header.stamp.toSec() << std::endl;

                new_fused_cloud = true;
            }

            // publish keyframe's included in pointclouds for pose graph backend
            // first change the keyfame connections to only include those previously published
            comm_msgs::keyframe kf_msg_new = *kf_msg;
            kf_msg_new.connections.clear();
            std::list<uint32_t>::reverse_iterator rit = synced_kf_ids.rbegin();
            rit++;
            for (int i = 0; i < num_odom_connections; i++) {
                if (rit == synced_kf_ids.rend())
                    break;
                kf_msg_new.connections.push_back(*rit);
                rit++;
            }
            
            /*
            std::cout << "KF connections for " << kf_msg->frameId << ":";
            for (auto iter = kf_msg_new.connections.begin(); iter != kf_msg_new.connections.end(); iter++) {
                std::cout << *iter << '\t';
            }
            std::cout << std::endl;
            */

            pub_keyframe.publish(kf_msg_new);

        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void registerPub(ros::NodeHandle &nh) {
    pub_pcl = nh.advertise<comm_msgs::fused_pcl>("fused_pcl", 1000); // update to include agent_id
    pub_pcl_raw = nh.advertise<sensor_msgs::PointCloud2>("fused_pcl_raw", 1000);
    pub_transform = nh.advertise<geometry_msgs::TransformStamped>("transform", 1000);
    pub_keyframe = nh.advertise<comm_msgs::keyframe>("fused_vins_keyframe", 1000);
}

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {
    pcl_buf.push_back(pcl_msg);
}

void keyframe_callback(const comm_msgs::keyframeConstPtr &kf_msg) {
    kf_buf.push_back(kf_msg);
}


int main(int argc, char **argv) {
    // Initialize ros
    ros::init(argc, argv, "pcl_fusion_node");

    // Set up the node
    ros::NodeHandle nh("~");

    // Register publishers
    registerPub(nh);

    // camara parameters
    std::string cam_file;
    if (!nh.getParam("cam_config", cam_file)) {
        ROS_ERROR("Parameters failed to load");
    }

    if (!nh.getParam("kf_per_cloud", kf_per_cloud)) {
        ROS_ERROR("Parameters failed to load");
    }

    if (!nh.getParam("voxel_filter_size", voxel_filter_size)) {
        ROS_ERROR("Parameters failed to load");
    }

    if (!nh.getParam("agent_id", agent_id)) {
        ROS_ERROR("Parameters failed to load");
    }

    if (!nh.getParam("num_odom_connections", num_odom_connections)) {
        ROS_ERROR("Parameters failed to load");
    }

    camera = std::make_shared<aslam::NCamera>();
    camera->deserializeFromFile(cam_file);
    T_S_C = (camera->get_T_C_B(0).getTransformationMatrix()).inverse();

    // Subscribers
    ros::Subscriber sub_pcl = nh.subscribe("/pointcloud", 2000, pcl_callback);
    ros::Subscriber sub_keyframe = nh.subscribe("/vins_keyframe", 2000, keyframe_callback);
    // ros::Subscriber sub_odometry = nh.subscribe("/odometry", 2000, odometry_callback);

    std::thread fusion_process;
    fusion_process = std::thread(process);

    ros::spin();

    return 0;
}
