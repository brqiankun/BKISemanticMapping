#include <string>
#include <iostream>
#include <ros/ros.h>
#include "bkioctomap.h"
#include "markerarray_pub.h"
#include <dbg.h>

void load_pcd(std::string filename, semantic_bki::point3f &origin, semantic_bki::PCLPointCloud &cloud) {
    pcl::PCLPointCloud2 cloud2;
    Eigen::Vector4f _origin;
    Eigen::Quaternionf orientaion;
    // 从xxx.pcd中读取并转换为pointcloud
    pcl::io::loadPCDFile(filename, cloud2, _origin, orientaion);
    pcl::fromPCLPointCloud2(cloud2, cloud);
    origin.x() = _origin[0];
    origin.y() = _origin[1];
    origin.z() = _origin[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "toy_example_node");
    ros::NodeHandle nh("~");
    
    std::string map_topic_csm("/semantic_csm");
    std::string map_topic("/semantic_bki");
    std::string var_topic_csm("/semantic_csm_variance");
    std::string var_topic("/semantic_bki_variance");
    std::string dir;
    std::string prefix;
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
    int scan_num = 0;
    double max_range = -1;

    nh.param<std::string>("dir", dir, dir);               // 数据集路径
    nh.param<std::string>("prefix", prefix, prefix);      // 数据集名称
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
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);
   
    ROS_INFO_STREAM("Parameters:" << std::endl <<
            "dir: " << dir << std::endl <<
            "prefix: " << prefix << std::endl <<
            "block_depth: " << block_depth << std::endl <<
            "sf2: " << sf2 << std::endl <<
            "ell: " << ell << std::endl <<
            "prior: " << prior << std::endl <<
            "var_thresh: " << var_thresh << std::endl <<
            "free_thresh: " << free_thresh << std::endl <<
            "occupied_thresh: " << occupied_thresh << std::endl <<
            "resolution: " << resolution << std::endl <<
            "num_class: " << num_class << std::endl <<
            "free_resolution: " << free_resolution << std::endl <<
            "ds_resolution: " << ds_resolution << std::endl <<
            "scan_sum: " << scan_num << std::endl <<
            "max_range: " << max_range
            );

    // visual the pcd 
    ros::Publisher raw_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("raw_pc", 1);
    ros::Publisher origin_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("origin_pc", 1);
    pcl::PointCloud<pcl::PointXYZL> raw_pc;
    pcl::PointCloud<pcl::PointXYZ> origin_pc;
    sensor_msgs::PointCloud2 raw_pc_msg;
    sensor_msgs::PointCloud2 origin_pc_msg;
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        semantic_bki::PCLPointCloud cloud;
        semantic_bki::point3f origin;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, cloud);
        std::printf("origin: x: %f, y: %f, z: %f\n", origin.x(), origin.y(), origin.z());
        pcl::PointXYZ origin_point {origin.x(), origin.y(), origin.z()};
        raw_pc += cloud;
        origin_pc.points.push_back(origin_point);
        ROS_INFO_STREAM("Scan " << scan_id << " done");
    }
    pcl::toROSMsg(raw_pc, raw_pc_msg);
    raw_pc_msg.header.frame_id = "map";
    pcl::toROSMsg(origin_pc, origin_pc_msg);
    origin_pc_msg.header.frame_id = "map";

    /////////////////////// Semantic CSM //////////////////////  semantic counting sensor model
    semantic_bki::SemanticBKIOctoMap map_csm(resolution, 1, num_class, sf2, ell, prior, 
                                             var_thresh, free_thresh, occupied_thresh);
    ros::Time start = ros::Time::now();
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        semantic_bki::PCLPointCloud cloud;
        semantic_bki::point3f origin;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, cloud);
        std::printf("origin: x: %f, y: %f, z: %f\n", origin.x(), origin.y(), origin.z());
        //                        点云(包含label)，sensor origin， 地图分辨率，地图free space采样分辨率， 最大范围
        map_csm.insert_pointcloud_csm(cloud, origin, resolution, free_resolution, max_range);
        ROS_INFO_STREAM("Scan " << scan_id << " done\n");
    }
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("Semantic CSM finished in " << (end - start).toSec() << "s\n");

    /////////////////////// Publish Map //////////////////////
    float max_var = std::numeric_limits<float>::min();
    float min_var = std::numeric_limits<float>::max(); 
    semantic_bki::MarkerArrayPub m_pub_csm(nh, map_topic_csm, resolution);
    for (auto it = map_csm.begin_leaf(); it != map_csm.end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
            semantic_bki::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            m_pub_csm.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), semantics, 0);
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);    // 得到当前node的vars
            if (vars[semantics] > max_var)
		          max_var = vars[semantics];
		        if (vars[semantics] < min_var)
		          min_var = vars[semantics];
        }
    }
    m_pub_csm.publish();
    std::cout << "max_var: " << max_var << std::endl;
    std::cout << "min_var: " << min_var << std::endl;
    
    /////////////////////// Variance Map //////////////////////
    semantic_bki::MarkerArrayPub v_pub_csm(nh, var_topic_csm, resolution);
    for (auto it = map_csm.begin_leaf(); it != map_csm.end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
            semantic_bki::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            v_pub_csm.insert_point3d_variance(p.x(), p.y(), p.z(), min_var, max_var, it.get_size(), vars[semantics]);
        }
    }
    v_pub_csm.publish();

    
    /////////////////////// Semantic BKI //////////////////////
    semantic_bki::SemanticBKIOctoMap map(resolution, block_depth, num_class, sf2, ell, prior, 
                                         var_thresh, free_thresh, occupied_thresh);
    start = ros::Time::now();
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        semantic_bki::PCLPointCloud cloud;
        semantic_bki::point3f origin;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, cloud);
        std::printf("origin: x: %f, y: %f, z: %f\n", origin.x(), origin.y(), origin.z());
        map.insert_pointcloud(cloud, origin, resolution, free_resolution, max_range);
        ROS_INFO_STREAM("Scan " << scan_id << " done");
    }
    end = ros::Time::now();
    ROS_INFO_STREAM("Semantic BKI finished in " << (end - start).toSec() << "s\n");
 
    
    /////////////////////// Publish Map //////////////////////
    max_var = std::numeric_limits<float>::min();
    min_var = std::numeric_limits<float>::max(); 
    semantic_bki::MarkerArrayPub m_pub(nh, map_topic, resolution);
    for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
            semantic_bki::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            m_pub.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), semantics, 0);
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            if (vars[semantics] > max_var)
		          max_var = vars[semantics];
		        if (vars[semantics] < min_var)
		          min_var = vars[semantics];
        }
    }

    m_pub.publish();
    std::cout << "max_var: " << max_var << std::endl;
    std::cout << "min_var: " << min_var << std::endl;

    /////////////////////// Variance Map //////////////////////
    semantic_bki::MarkerArrayPub v_pub(nh, var_topic, resolution);
    for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
            semantic_bki::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            v_pub.insert_point3d_variance(p.x(), p.y(), p.z(), min_var, max_var, it.get_size(), vars[semantics]);
        }
    }
    v_pub.publish();

    ros::Rate loop_rate(0.1);
    while (ros::ok()) {
        raw_pc_pub.publish(raw_pc_msg);
        origin_pc_pub.publish(origin_pc_msg);
        ros::spinOnce();
        ROS_INFO_STREAM("raw_pc and origin_pc done\n");
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}
