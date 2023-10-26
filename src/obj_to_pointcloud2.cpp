#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/obj_io.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

class Obj2PointCloud2
{
public:
  Obj2PointCloud2(ros::NodeHandle &nh, ros::NodeHandle &nh_local);
  ~Obj2PointCloud2();
  void timerCallback(const ros::TimerEvent &e);

private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_local;
  ros::Publisher pub_cloud;
  ros::Timer timer;
  sensor_msgs::PointCloud2 ros_cloud;

  std::string obj_file_path;
  std::string map_frame_id;
  double leaf_size;
  double publish_period;
};

Obj2PointCloud2::Obj2PointCloud2(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
{
  nh_local.param<std::string>("map_frame_id", map_frame_id, std::string("map"));
  nh_local.param<std::string>("obj_file_path", obj_file_path, std::string(""));
  nh_local.param<double>("leaf_size", leaf_size, 0.0);
  nh_local.param<double>("publish_period", publish_period, 1.0);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadOBJFile(obj_file_path, *pcl_cloud);
  ROS_INFO_STREAM("[OBJ 2 PointCloud2]: Loaded " << pcl_cloud->size() << " points from " << obj_file_path);

  if (leaf_size)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(pcl_cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*pcl_cloud);
  }
  ROS_INFO_STREAM("[OBJ 2 PointCloud2]: Filtered " << pcl_cloud->size() << " points from " << obj_file_path);

  toROSMsg(*pcl_cloud, ros_cloud);
  ros_cloud.header.frame_id = map_frame_id;
  ROS_INFO_STREAM("[OBJ 2 PointCloud2]: Converted ros cloud " << ros_cloud.height << ": " << ros_cloud.width);

  timer = nh.createTimer(ros::Duration(publish_period), &Obj2PointCloud2::timerCallback, this);
}
Obj2PointCloud2::~Obj2PointCloud2()
{
}
void Obj2PointCloud2::timerCallback(const ros::TimerEvent &e)
{
  ros::Time now = ros::Time::now();
  ros_cloud.header.stamp = now;
  pub_cloud.publish(ros_cloud);
  ROS_INFO_STREAM("[OBJ 2 PointCloud2]: Published pointcloud at " << now);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obj_2_pointcloud2");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[OBJ 2 PointCloud2]: Initializing node");
    Obj2PointCloud2 obj_2_pointcloud2(nh, nh_local);
    ros::spin();
  }
  catch (const char *s)
  {
    ROS_FATAL_STREAM("[OBJ 2 PointCloud2]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[OBJ 2 PointCloud2]: Unexpected error");
  }

  return 0;
}
