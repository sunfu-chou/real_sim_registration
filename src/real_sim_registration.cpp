#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace ros;
using namespace std;

class MapFilter
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map;

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster br_;

    Eigen::Matrix4f map_to_laser_eigen;

    double y_range;
    double x_range;
    double z_range;
    double leaf_size;
    double map_leaf_size;
    string env_name;
    string obj_file_path;

    sensor_msgs::PointCloud2 ros_pc;
    sensor_msgs::PointCloud2 ros_map;
    sensor_msgs::PointCloud2 ros_transformed_map;

    geometry_msgs::Transform map_to_laser_geo;

    Publisher pub_map;
    Publisher pub_transformed_map;
    Publisher pub_map_to_icp_laser;
    Subscriber sub_pc;
    Subscriber sub_map_to_base;

public:
    void pc_cb(const sensor_msgs::PointCloud2 msg);
    void pose_cb(const geometry_msgs::Transform::ConstPtr &msg);
    MapFilter(NodeHandle &nh, NodeHandle &nh_local);
    ~MapFilter();
};

MapFilter::MapFilter(NodeHandle &nh, NodeHandle &nh_local) : tf2_listener_(tf2_buffer_)
{
    nh_local.param<double>("x_range", x_range, 10.0);
    nh_local.param<double>("y_range", y_range, 10.0);
    nh_local.param<double>("z_range", z_range, 20.0);
    nh_local.param<double>("voxel_size", leaf_size, 0.25);
    nh_local.param<double>("map_voxel_size", map_leaf_size, 0.25);
    nh_local.param<string>("env_name", env_name, string("EE6F"));
    nh_local.param<std::string>("obj_file_path", obj_file_path, std::string(""));

    ROS_INFO("filter x range %f", x_range);
    ROS_INFO("filter y range %f", y_range);
    ROS_INFO("filter z range %f", z_range);
    ROS_INFO("filter voxel size %f", leaf_size);

    map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadOBJFile(obj_file_path, *map);
    ROS_INFO_STREAM("map width:" << map->width << "map height:" << map->height);
    if (map_leaf_size > 0)
    {
        voxel.setInputCloud(map);
        voxel.setLeafSize(map_leaf_size, map_leaf_size, map_leaf_size);
        voxel.filter(*map);
    }

    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    icp.setMaxCorrespondenceDistance(5);
    icp.setTransformationEpsilon(1e-12);
    icp.setEuclideanFitnessEpsilon(1e-3);
    icp.setMaximumIterations(200);

    ros_pc.header.frame_id = "map";

    // pub_map = nh.advertise<sensor_msgs::PointCloud2>("robot3/visualize_map", 1);
    pub_transformed_map = nh.advertise<sensor_msgs::PointCloud2>("robot3/visualize_transformed_map", 1);
    pub_map_to_icp_laser = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot3/pose", 1);
    sub_pc = nh.subscribe("velodyne/velodyne_points", 1, &MapFilter::pc_cb, this);
    sub_map_to_base = nh.subscribe("pose", 1, &MapFilter::pose_cb, this);
    ROS_INFO("map filter initialized");

    pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

MapFilter::~MapFilter()
{
}

void MapFilter::pc_cb(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, *pc);
    voxel.setInputCloud(pc);
    voxel.filter(*pc);

    geometry_msgs::TransformStamped map_to_laser_geo_stamped;
    try
    {
        map_to_laser_geo_stamped = tf2_buffer_.lookupTransform("map", "velodyne", ros::Time());
    }
    catch (const tf2::TransformException &ex)
    {
        try
        {
            map_to_laser_geo_stamped = tf2_buffer_.lookupTransform("map", "velodyne", ros::Time());
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_STREAM(ex.what());
        }
    }
    map_to_laser_eigen = tf2::transformToEigen(map_to_laser_geo_stamped).matrix().cast<float>();

    geometry_msgs::TransformStamped odom_to_laser_geo;
    try
    {
        odom_to_laser_geo = tf2_buffer_.lookupTransform("odom", "velodyne", ros::Time());
    }
    catch (const tf2::TransformException &ex)
    {
        try
        {
            odom_to_laser_geo = tf2_buffer_.lookupTransform("odom", "velodyne", ros::Time());
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_STREAM(ex.what());
        }
    }

    tf2::Transform odom_to_laser_tf2;
    tf2::fromMsg(odom_to_laser_geo.transform, odom_to_laser_tf2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_map(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*map, *transformed_map, map_to_laser_eigen.inverse());
    ROS_INFO_STREAM("pc: " << pc->size());
    ROS_INFO_STREAM("map: " << map->size());
    ROS_INFO_STREAM("transformed_map: " << transformed_map->size());

    toROSMsg(*transformed_map, ros_transformed_map);
    ros_transformed_map.header.frame_id = "velodyne";
    ros_transformed_map.header.stamp = ros::Time::now();
    pub_transformed_map.publish(ros_transformed_map);

    icp.setInputSource(pc);
    icp.setInputTarget(transformed_map);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp;
    tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO_STREAM("starting icp");
    icp.align(*tmp);
    ROS_INFO_STREAM("finished icp");

    ROS_INFO_STREAM("icp fitness score: " << icp.getFitnessScore());
    if (icp.getFitnessScore() > 1.0)
    {
        ROS_INFO_STREAM("icp failed");
        return;
    }

    // Convert Eigen::Matrix4f: icp.getFinalTransformation() to tf2::Transform: laser_to_icp_laser_tf2
    Eigen::Matrix4f laser_to_icp_laser_eigen = icp.getFinalTransformation();
    Eigen::Affine3d laser_to_icp_laser_eigen_aff = Eigen::Affine3d(laser_to_icp_laser_eigen.cast<double>());
    geometry_msgs::TransformStamped laser_to_icp_laser_geo_stamped = tf2::eigenToTransform(laser_to_icp_laser_eigen_aff);
    tf2::Transform laser_to_icp_laser_tf2;
    tf2::fromMsg(laser_to_icp_laser_geo_stamped.transform, laser_to_icp_laser_tf2);

    // Convert geometry_msgs::Transform: map_to_laser_geo to tf2::Transform: map_to_laser_tf2
    tf2::Transform map_to_laser_tf2;
    tf2::fromMsg(map_to_laser_geo_stamped.transform, map_to_laser_tf2);

    // Brocast tf map to odom
    tf2::Transform map_to_icp_laser_tf2 = map_to_laser_tf2 * laser_to_icp_laser_tf2;
    tf2::Transform map_to_odom_tf2 = map_to_icp_laser_tf2 * odom_to_laser_tf2.inverse();
    // geometry_msgs::TransformStamped map_to_odom_geo_stamped;
    // map_to_odom_geo_stamped.header.stamp = ros::Time::now();
    // map_to_odom_geo_stamped.header.frame_id = "map";
    // map_to_odom_geo_stamped.child_frame_id = "odom";
    // map_to_odom_geo_stamped.transform = tf2::toMsg(map_to_odom_tf2);
    // br_.sendTransform(map_to_odom_geo_stamped);

    // Publish pose map to laser
    geometry_msgs::PoseWithCovarianceStamped map_to_icp_laser_geo_pose_stamped;
    map_to_icp_laser_geo_pose_stamped.header.stamp = ros::Time::now();
    map_to_icp_laser_geo_pose_stamped.header.frame_id = "map";
    geometry_msgs::Transform map_to_icp_laser_geo = tf2::toMsg(map_to_icp_laser_tf2);
    map_to_icp_laser_geo_pose_stamped.pose.pose.position.x = map_to_icp_laser_geo.translation.x;
    map_to_icp_laser_geo_pose_stamped.pose.pose.position.y = map_to_icp_laser_geo.translation.y;
    map_to_icp_laser_geo_pose_stamped.pose.pose.position.z = map_to_icp_laser_geo.translation.z;
    map_to_icp_laser_geo_pose_stamped.pose.pose.orientation = map_to_icp_laser_geo.rotation;
    map_to_icp_laser_geo_pose_stamped.pose.covariance = {0.5, 0, 0, 0, 0, 0,
                                                         0, 0.5, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0, 0.5};
    pub_map_to_icp_laser.publish(map_to_icp_laser_geo_pose_stamped);
}

void MapFilter::pose_cb(const geometry_msgs::Transform::ConstPtr &msg)
{
    map_to_laser_geo = *msg;
    map_to_laser_eigen = tf2::transformToEigen(*msg).matrix().cast<float>();
}

int main(int argc, char **argv)
{
    init(argc, argv, "real_sim_registration");
    NodeHandle nh("");
    NodeHandle nh_local("~");

    MapFilter mapfilter(nh, nh_local);

    spin();

    return 0;
}
