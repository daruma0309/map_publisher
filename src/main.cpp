#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>

#ifndef M_PI
#define M_PI 3.14159265358979             // 円周率
#endif

#ifndef NULL
#define NULL 0                     // 基本的には、C++11のnullptrを使う
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度
#endif

geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw){
  tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion geometry_quat;
  quaternionTFToMsg(quat, geometry_quat);
  return geometry_quat;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "map_publisher");

  ros::NodeHandle nh;
  ros::Publisher pub_map = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("map", 1);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("init_pose", 1);

// param
  std::string map_filename;
  double LeafSize, start_x, start_y, start_th;
  ros::param::get("map_filename", map_filename);
  ros::param::get("LeafSize", LeafSize);
  ros::param::get("start_x", start_x);
  ros::param::get("start_y", start_y);
  ros::param::get("start_th", start_th);

// map
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(map_filename, *p_cloud);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(LeafSize, LeafSize, LeafSize);
  approximate_voxel_filter.setInputCloud(p_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  approximate_voxel_filter.filter(*filtered_cloud);
  filtered_cloud->header.frame_id = "map";

// init_pose
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.pose.position.x = start_x;
  pose_stamped.pose.position.y = start_y;
  pose_stamped.pose.position.z = 0;
  pose_stamped.pose.orientation = rpy_to_geometry_quat(0.0, 0.0, DEG2RAD(start_th));
  pose_stamped.header.frame_id = "map";
  pose_stamped.header.stamp = ros::Time::now();

  ros::Rate loop_rate(1);

  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), filtered_cloud->header.stamp);
    pub_map.publish(filtered_cloud);
    pub_pose.publish(pose_stamped);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
