/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// #define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <cstdio>
#include <ros/ros.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud_density.h>

#include <pcl/visualization/pcl_visualizer.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"

#include "tough_perception_common/periodic_snapshotter.h"

/***
 * This used to be a simple test app that requests a point cloud from the
 * point_cloud_assembler every x seconds, and then publishes the
 * resulting data
 */

using namespace laser_assembler;

/****************************************************
 * namespaces and typedefs required for registeration
 ****************************************************/
// using pcl::visualization::PointCloudColorHandlerCustom;
// using pcl::visualization::PointCloudColorHandlerGenericField;

// // convenient typedefs
// typedef pcl::PointXYZI PointT;
// typedef pcl::PointCloud<PointT> PointCloud;
// typedef pcl::PointNormal PointNormalT;
// typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// struct PointTWithTime
// {
//   PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
//   double time;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
// } EIGEN_ALIGN16;                  // enforce SSE padding for correct memory alignment

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointTWithTime, // here we assume a XYZ + "test" (as fields)
//                                   (float, x, x)(float, y, y)(float, z, z)(double, time, time))

PeriodicSnapshotter::PeriodicSnapshotter() : assembled_pc(new PointCloud),
                                             assembled_pc_I(new PointCloud_I),
                                             prev_msg_(new sensor_msgs::PointCloud2)
{
  robot_state_ = RobotStateInformer::getRobotStateInformer(n_);
  rd_ = RobotDescription::getRobotDescription(n_);

  snapshot_pub_ = n_.advertise<sensor_msgs::PointCloud2>("snapshot_cloud2", 1, true);
  registered_pointcloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("assembled_cloud2", 1, true);
  pointcloud_for_octomap_pub_ = n_.advertise<sensor_msgs::PointCloud2>("assembled_octomap_cloud2", 10, true);

  resetPointcloudSub_ = n_.subscribe("reset_pointcloud", 10, &PeriodicSnapshotter::resetPointcloudCB, this);
  pausePointcloudSub_ = n_.subscribe("pause_pointcloud", 10, &PeriodicSnapshotter::pausePointcloudCB, this);
  boxFilterSub_ = n_.subscribe("clearbox_pointcloud", 10, &PeriodicSnapshotter::setBoxFilterCB, this);

  // Create the service client for calling the assembler
  client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");
  snapshot_sub_ = n_.subscribe("snapshot_cloud2", 10, &PeriodicSnapshotter::mergeClouds, this);

  // Start the timer that will trigger the processing loop (timerCallback)
  float timeout;
  n_.param<float>("laser_assembler_svc/laser_snapshot_timeout", timeout, 5.0);
  ROS_INFO("PeriodicSnapshotter::PeriodicSnapshotter : Snapshot timeout : %.2f seconds", timeout);
  timer_ = n_.createTimer(ros::Duration(timeout, 0), &PeriodicSnapshotter::timerCallback, this);

  // Need to track if we've called the timerCallback at least once
  first_time_ = true;
  downsample_ = true;
  enable_box_filter_ = false;
  resetPointcloud_ = true;

  n_.param<float>("filter_min_x", filter_min_x, -10.0);
  n_.param<float>("filter_max_x", filter_max_x, 10.0);

  n_.param<float>("filter_min_y", filter_min_y, -10.0);
  n_.param<float>("filter_max_y", filter_max_y, 10.0);

  n_.param<float>("filter_min_z", filter_min_z, -10.0);
  n_.param<float>("filter_max_z", filter_max_z, 10.0);
  snapshotCount_ = 0;
}

void PeriodicSnapshotter::timerCallback(const ros::TimerEvent &e)
{
  // We don't want to build a cloud the first callback, since we
  //   don't have a start and end time yet
  if (first_time_)
  {
    ROS_INFO("PeriodicSnapshotter::timerCallback : Ignoring current snapshot");
    first_time_ = false;
    return;
  }

  // Populate our service request based on our timer callback times
  AssembleScans2 srv;
  srv.request.begin = e.last_real;
  srv.request.end = e.current_real;

  // Make the service call
  if (client_.call(srv))
  {
    ROS_INFO("PeriodicSnapshotter::timerCallback : Published Cloud with %u points",
             (uint32_t)(srv.response.cloud.data.size()));
    snapshot_pub_.publish(srv.response.cloud);
    ++snapshotCount_;
    if (snapshotCount_ > MAX_SNAPSHOTS)
    {
      pausePointcloud(true);
    }
  }
  else
  {
    ROS_WARN("Error making service call\n");
  }
}

void PeriodicSnapshotter::pairAlign(const PointCloud::Ptr cloud_src,
                                    const PointCloud::Ptr cloud_tgt,
                                    PointCloud::Ptr output)
{
  // Downsample for consistency and speed
  // note enable this for large datasets
  PointCloud::Ptr src(new PointCloud);
  PointCloud::Ptr tgt(new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize(0.05, 0.05, 0.05);
  grid.setInputCloud(cloud_src);
  grid.filter(*src);

  grid.setInputCloud(cloud_tgt);
  grid.filter(*tgt);

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*points_with_normals_src);
  pcl::copyPointCloud(*src, *points_with_normals_src);

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  // Instantiate our custom point representation (defined above) ...
  customPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.1);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const customPointRepresentation>(point_representation));

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  // Run the same optimization in a loop
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations(2);
  for (int i = 0; i < 30; ++i)
  {
    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();
  }

  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  // Transform target back in source frame
  pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

  // PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
  // PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);

  // add the source to the transformed target
  *output += *cloud_src;

  // final_transform = targetToSource;
}

void PeriodicSnapshotter::pairAlign_I(const PointCloud_I::Ptr cloud_src,
                                      const PointCloud_I::Ptr cloud_tgt,
                                      PointCloud_I::Ptr output)
{
  // Downsample for consistency and speed
  // note enable this for large datasets
  PointCloud_I::Ptr src(new PointCloud_I);
  PointCloud_I::Ptr tgt(new PointCloud_I);
  pcl::VoxelGrid<PointTI> grid;
  grid.setLeafSize(0.05, 0.05, 0.05);
  grid.setInputCloud(cloud_src);
  grid.filter(*src);

  grid.setInputCloud(cloud_tgt);
  grid.filter(*tgt);

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

  pcl::NormalEstimation<PointTI, PointNormalT> norm_est;
  pcl::search::KdTree<PointTI>::Ptr tree(new pcl::search::KdTree<PointTI>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*points_with_normals_src);
  pcl::copyPointCloud(*src, *points_with_normals_src);

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  // Instantiate our custom point representation (defined above) ...
  customPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.1);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const customPointRepresentation>(point_representation));

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  // Run the same optimization in a loop
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations(2);
  for (int i = 0; i < 30; ++i)
  {
    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();
  }

  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  // Transform target back in source frame
  pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

  // add the source to the transformed target
  *output += *cloud_src;
}

void PeriodicSnapshotter::resetPointcloud(bool resetPointcloud)
{
  // reset the assembler
  state_request = PCL_STATE_CONTROL::RESET;
  snapshotCount_ = 0;
}

void PeriodicSnapshotter::resetPointcloudCB(const std_msgs::Empty &msg)
{
  state_request = PCL_STATE_CONTROL::RESET;
  snapshotCount_ = 0;
}

void PeriodicSnapshotter::pausePointcloud(bool pausePointcloud)
{
  state_request = pausePointcloud ? PCL_STATE_CONTROL::PAUSE : PCL_STATE_CONTROL::RESUME;
  if (!pausePointcloud)
  {
    snapshotCount_ = 0;
  }
}

void PeriodicSnapshotter::pausePointcloudCB(const std_msgs::Bool &msg)
{
  // reset will make sure that older scans are discarded
  state_request = msg.data ? PCL_STATE_CONTROL::PAUSE : PCL_STATE_CONTROL::RESUME;
  if (!msg.data)
  {
    snapshotCount_ = 0;
  }
}

void PeriodicSnapshotter::setBoxFilterCB(const std_msgs::Int8 &msg)
{
  enable_box_filter_ = true;
  pcl::PointCloud<PointT>::Ptr pcl_prev_msg(new pcl::PointCloud<PointT>);
  convertROStoPCL(prev_msg_, pcl_prev_msg);
  geometry_msgs::Pose pelvisPose;
  pcl::PointCloud<PointT>::Ptr tgt(new pcl::PointCloud<PointT>);
  robot_state_->getCurrentPose(rd_->getPelvisFrame(), pelvisPose);
  Eigen::Vector4f minPoint;
  Eigen::Vector4f maxPoint;

  // this indicates that if the msg contains element 1 it, would clear point cloud from waist up
  if (msg.data == 1)
  {
    // waist up condition
    minPoint[0] = -1;  // x
    minPoint[1] = -1;  // y
    minPoint[2] = 0.0; // z

    maxPoint[0] = 2;
    maxPoint[1] = 1;
    maxPoint[2] = 1;
  }
  else if (msg.data == 2)
  {
    // large box condition
    minPoint[0] = -1.0;
    minPoint[1] = -1.5;
    minPoint[2] = -0.5;

    maxPoint[0] = 4;
    maxPoint[1] = 1.5;
    maxPoint[2] = 1.5;
  }
  else
  {
    // full box condition
    minPoint[0] = -1;
    minPoint[1] = -1;
    minPoint[2] = -0.5;

    maxPoint[0] = 2;
    maxPoint[1] = 1;
    maxPoint[2] = 1;
  }

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = pelvisPose.position.x;
  boxTranslatation[1] = pelvisPose.position.y;
  boxTranslatation[2] = pelvisPose.position.z;
  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0;                                  // rotation around x-axis
  boxRotation[1] = 0;                                  // rotation around y-axis
  boxRotation[2] = tf::getYaw(pelvisPose.orientation); // in radians rotation around z-axis. this rotates your cube
                                                       // 45deg around z-axis.

  pcl::CropBox<PointT> box_filter;
  std::vector<int> indices;
  indices.clear();
  box_filter.setInputCloud(pcl_prev_msg);
  box_filter.setMin(minPoint);
  box_filter.setMax(maxPoint);
  box_filter.setTranslation(boxTranslatation);
  box_filter.setRotation(boxRotation);
  box_filter.setNegative(true);
  box_filter.filter(*tgt);
  sensor_msgs::PointCloud2::Ptr merged_cloud(new sensor_msgs::PointCloud2());
  convertPCLtoROS<PointCloud::Ptr, PointCloudSensorMsg::Ptr>(tgt, merged_cloud);
  prev_msg_ = merged_cloud;

  registered_pointcloud_pub_.publish(merged_cloud);
  enable_box_filter_ = false;
}

void PeriodicSnapshotter::mergeClouds(const PointCloudSensorMsg::Ptr msg)
{
  if (enable_box_filter_)
    return;

  if (state_request == PCL_STATE_CONTROL::PAUSE)
  {
    ROS_INFO("PeriodicSnapshotter::mergeClouds : Laser assembling paused");
    registered_pointcloud_pub_.publish(prev_msg_);
    return;
  }

  PointCloudSensorMsg::Ptr merged_cloud(new PointCloudSensorMsg());
  PointCloud::Ptr pcl_msg(new PointCloud);
  PointCloud_I::Ptr pclI_msg(new PointCloud_I);

  convertROStoPCL<PointCloud::Ptr, PointCloudSensorMsg::Ptr>(msg, pcl_msg);
  addIntensity(pcl_msg, pclI_msg); // add initial value

  if (state_request == PCL_STATE_CONTROL::RESET || prev_msg_->data.empty())
  {
    ROS_INFO("PeriodicSnapshotter::mergeClouds : Resetting Pointcloud");
    merged_cloud = msg;
    assembled_pc_I = pclI_msg;
    pointcloud_for_octomap_pub_.publish(prev_msg_->data.empty() ? msg : prev_msg_);
    state_request = PCL_STATE_CONTROL::RESUME;
  }
  else
  {
    // merge the current msg with previous messages published till now
    // http://www.pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration
    // PointCloud::Ptr pcl_prev_msg(new PointCloud);
    // convertROStoPCL(prev_msg_, pcl_prev_msg);
    // pcl_prev_msg = assembled_pc;

    // PointCloud::Ptr result(new PointCloud);
    PointCloud_I::Ptr result_I(new PointCloud_I);
    // Eigen::Matrix4f pairTransform;

    // check if the cloud size is growing exceptionally high. if yes, downsample the pointcloud without impacting
    // objects and features
    // pairAlign(pcl_msg, pcl_prev_msg, result, pairTransform);
    // pairAlign(pcl_msg, pcl_prev_msg, result);
    pairAlign_I(pclI_msg, assembled_pc_I, result_I);

    // clip the point cloud in x y and z direction
    // clipPointCloud(result);
    clipPointCloud(result_I);

    // ROS_INFO("PeriodicSnapshotter::mergeClouds : PC size : %d", result->size());
    ROS_INFO("PeriodicSnapshotter::mergeClouds : PC size : %d", result_I->size());
    ROS_INFO("PeriodicSnapshotter::mergeClouds : PC intensity %f", result_I->points[0].intensity);

    decayPoint(result_I);
    min_internsity(pclI_msg);
    min_internsity(assembled_pc_I);
    min_internsity(result_I);

    ROS_INFO("PeriodicSnapshotter::mergeClouds : result_I intensity %f", result_I->points[0].intensity);

    // assembled_pc = result;
    // convertPCLtoROS<PointCloud::Ptr, PointCloudSensorMsg::Ptr>(result, merged_cloud);
    convertPCLtoROS<PointCloud_I::Ptr, PointCloudSensorMsg::Ptr>(result_I, merged_cloud);
    assembled_pc_I = result_I;
  }

  // publish the merged message
  prev_msg_ = merged_cloud;
  // convertROStoPCL<PointCloud::Ptr, PointCloudSensorMsg::Ptr>(merged_cloud, assembled_pc);
  // addIntensity(assembled_pc, assembled_pc_I);
  registered_pointcloud_pub_.publish(merged_cloud);
}

void PeriodicSnapshotter::clipPointCloud(const PointCloud::Ptr input_cloud)
{
  pcl::PassThrough<PointT> globalPassThroughFilter;

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("z");
  globalPassThroughFilter.setFilterLimits(filter_min_z, filter_max_z);
  globalPassThroughFilter.filter(*input_cloud);

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("y");
  globalPassThroughFilter.setFilterLimits(filter_min_y, filter_max_y);
  globalPassThroughFilter.filter(*input_cloud);

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("x");
  globalPassThroughFilter.setFilterLimits(filter_min_x, filter_max_x);
  globalPassThroughFilter.filter(*input_cloud);
}

void PeriodicSnapshotter::clipPointCloud(const PointCloud_I::Ptr input_cloud)
{
  pcl::PassThrough<PointTI> globalPassThroughFilter;

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("z");
  globalPassThroughFilter.setFilterLimits(filter_min_z, filter_max_z);
  globalPassThroughFilter.filter(*input_cloud);

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("y");
  globalPassThroughFilter.setFilterLimits(filter_min_y, filter_max_y);
  globalPassThroughFilter.filter(*input_cloud);

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("x");
  globalPassThroughFilter.setFilterLimits(filter_min_x, filter_max_x);
  globalPassThroughFilter.filter(*input_cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;

  float timeout;
  n.param<float>("laser_assembler_svc/laser_snapshot_timeout", timeout, 5.0);

  ros::Rate looprate((double)timeout);
  while (ros::ok())
  {
    ros::spinOnce();
    looprate.sleep();
  }
  return 0;
}
