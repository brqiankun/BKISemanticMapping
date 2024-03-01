#include <string>
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "bkioctomap.h"
#include "markerarray_pub.h"

#include <dbg.h>

tf::TransformListener *listener;
std::string frame_id("lidar_init");
std::string child_frame_id("lidar");
semantic_bki::SemanticBKIOctoMap* map;
semantic_bki::MarkerArrayPub* m_pub;

tf::Vector3 last_position;
tf::Quaternion last_orientation;
bool first = true;
double position_change_thresh = 0.01;
double orientation_change_thresh = 0.02;
bool updated = false;

std::string map_topic("/occupied_cells_vis_array");
int block_depth = 4;
double sf2 = 1.0;
double ell = 1.0;
float prior = 1.0f;
float var_thresh = 1.0f;
double free_thresh = 0.3;
double occupied_thresh = 0.7;
double resolution = 0.1;
int num_class = 2;
double free_resolution = 0.5;
double ds_resolution = 0.1;
double max_range = -1;


void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloud) {
  std::printf("bkiSemantic_server receive a pointcloud msg\n");

  tf::StampedTransform transform;
  try {
        listener->waitForTransform(frame_id, child_frame_id, cloud->header.stamp, ros::Duration(3.0));
        listener->lookupTransform(frame_id, child_frame_id, cloud->header.stamp, transform); //ros::Time::now() -- Don't use this because processing time delay breaks it
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return;
  }

  ros::Time start = ros::Time::now();
  semantic_bki::point3f origin;
  tf::Vector3 translation = transform.getOrigin();
  tf::Quaternion orientation = transform.getRotation();

  if (first || orientation.angleShortestPath(last_orientation) > orientation_change_thresh || 
      translation.distance(last_position) > position_change_thresh) {
    ROS_INFO_STREAM("Cloud received");
        
    last_position = translation;
    last_orientation = orientation;
    origin.x() = (float) translation.x();
    origin.y() = (float) translation.y();
    origin.z() = (float) translation.z();

    sensor_msgs::PointCloud2 cloud_map;
    pcl_ros::transformPointCloud(frame_id, *cloud, cloud_map, *listener);

    //pointer required for downsampling
    pcl::PointCloud<pcl::PointXYZL>::Ptr pcl_cloud { new pcl::PointCloud<pcl::PointXYZL>() };
    pcl::fromROSMsg(cloud_map, *pcl_cloud);
    // std::printf("before remove nan point: ");
    // dbg(pcl_cloud->size());
    pcl_cloud->is_dense = false;    // 设置为false使得removeNaN生效
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_cloud, *pcl_cloud, indices);
    // std::printf("after remove nan point: ");
    // dbg(pcl_cloud->size());
    //downsample for faster mapping
    pcl::PointCloud<pcl::PointXYZL> filtered_cloud;
    pcl::VoxelGrid<pcl::PointXYZL> filterer;
    filterer.setInputCloud(pcl_cloud);
    filterer.setLeafSize(ds_resolution, ds_resolution, ds_resolution);
    filterer.filter(filtered_cloud);
    dbg(filtered_cloud.size());

    if (filtered_cloud.size() > 5) {
      map->insert_pointcloud(filtered_cloud, origin, ds_resolution, free_resolution, max_range);
    }

    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("One cloud finished in " << (end - start).toSec() << "s");
    updated = true;
  } else {
        dbg(orientation.angleShortestPath(last_orientation));
        dbg(translation.distance(last_position));
        dbg(first);
  }

  if (updated) {
    ros::Time start2 = ros::Time::now();
    m_pub->clear_map(resolution);
    for (auto it = map->begin_leaf(); it != map->end_leaf(); ++it) {
      if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
        semantic_bki::point3f p = it.get_loc();
        // 3表示使用的数据集颜色方式，此时使用的是3类 BrMapColor
        m_pub->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), 3);
      }
    }
    updated = false;
    m_pub->publish();
    ros::Time end2 = ros::Time::now();
    ROS_INFO_STREAM("One map published in " << (end2 - start2).toSec() << "s\n");
  }
  first = false;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "br_semantickitti_server");
    ros::NodeHandle nh("~");
    //incoming pointcloud topic, this could be put into the .yaml too
    std::string cloud_topic("/velodyne_cloud_registered_curr");

    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<double>("max_range", max_range, max_range);
    nh.param<std::string>("cloud_topic", cloud_topic, cloud_topic);

    ROS_INFO_STREAM("Parameters:" << std::endl <<
      "cloud_topic: " << cloud_topic << std::endl << 
      "block_depth: " << block_depth << std::endl <<
      "sf2: " << sf2 << std::endl <<
      "ell: " << ell << std::endl <<
      "prior:" << prior << std::endl <<
      "var_thresh: " << var_thresh << std::endl <<
      "free_thresh: " << free_thresh << std::endl <<
      "occupied_thresh: " << occupied_thresh << std::endl <<
      "resolution: " << resolution << std::endl <<
      "num_class: " << num_class << std::endl << 
      "free_resolution: " << free_resolution << std::endl <<
      "ds_resolution: " << ds_resolution << std::endl <<
      "max_range: " << max_range << std::endl
    );
    
  map = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, 
                                              prior, var_thresh, free_thresh, occupied_thresh);
  m_pub = new semantic_bki::MarkerArrayPub(nh, map_topic, resolution);
  listener = new tf::TransformListener();
  ros::Subscriber point_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1, cloudHandler);

  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}


