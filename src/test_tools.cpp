#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
// TimeSyncronizer headers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "tools.hpp"
#include "tools_thread_pool.hpp"
#include "common_tools.h"
using namespace std;

std::shared_ptr<std::ofstream> outfile;
string pointcloud_topic,odom_topic,outfile_path;
int lidar_type;
bool print_cloud;
bool save_odom;
bool save_pcd;
bool frame_wise;
bool pcd_name_stamp; // 0:序号命名 1:时间戳命名
bool flg_exit = false;
shared_ptr<pcl::PointCloud<robosense_ros::Point>> pcl_wait_save1;
shared_ptr<pcl::PointCloud<velodyne_ros::Point>> pcl_wait_save2;
shared_ptr<pcl::PointCloud<ouster_ros::Point>> pcl_wait_save3;
shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pcl_wait_save4;
shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> pcl_wait_save5;

inline void SavePCD(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    static int count=0;
    switch (lidar_type)
    {
    case ROBOSENSE:
    {
        pcl::PointCloud<robosense_ros::Point> laserCloud;
        pcl::fromROSMsg(*msg, laserCloud);
        if(!frame_wise) *pcl_wait_save1 += laserCloud;
        else{
            pcl::PCDWriter pcd_writer;
            if (pcd_name_stamp) {
                stringstream ss;
                ss.setf(std::ios::fixed);
                ss.precision(9);
                ss<<msg->header.stamp.toSec();
                string file_name(outfile_path+"pcd/"+ss.str()+".pcd");
                pcd_writer.write(file_name, laserCloud, false);
            }
            else pcd_writer.write(outfile_path+"pcd/"+std::to_string(count)+".pcd", laserCloud, false);
        }
        break;
    }
    case VELODYNE:
    {
        pcl::PointCloud<velodyne_ros::Point> laserCloud;
        pcl::fromROSMsg(*msg, laserCloud);
        if(!frame_wise) *pcl_wait_save2 += laserCloud;
        else{
            pcl::PCDWriter pcd_writer;
            if (pcd_name_stamp) {
                stringstream ss;
                ss.setf(std::ios::fixed);
                ss.precision(9);
                ss<<msg->header.stamp.toSec();
                string file_name(outfile_path+"pcd/"+ss.str()+".pcd");
                pcd_writer.write(file_name, laserCloud, false);
            }
            else pcd_writer.write(outfile_path+"pcd/"+std::to_string(count)+".pcd", laserCloud, false);
        }
        break;
    }
    case OUSTER:
    {
        pcl::PointCloud<ouster_ros::Point> laserCloud;
        pcl::fromROSMsg(*msg, laserCloud);
        if(!frame_wise) *pcl_wait_save3 += laserCloud;
        else{
            pcl::PCDWriter pcd_writer;
            if (pcd_name_stamp) {
                stringstream ss;
                ss.setf(std::ios::fixed);
                ss.precision(9);
                ss<<msg->header.stamp.toSec();
                string file_name(outfile_path+"pcd/"+ss.str()+".pcd");
                pcd_writer.write(file_name, laserCloud, false);
            }
            else pcd_writer.write(outfile_path+"pcd/"+std::to_string(count)+".pcd", laserCloud, false);
        }
        break;
    }
    case XYZI:
    {
        pcl::PointCloud<pcl::PointXYZI> laserCloud;
        pcl::fromROSMsg(*msg, laserCloud);
        if(!frame_wise) *pcl_wait_save4 += laserCloud;
        else{
            pcl::PCDWriter pcd_writer;
            if (pcd_name_stamp) {
                stringstream ss;
                ss.setf(std::ios::fixed);
                ss.precision(9);
                ss<<msg->header.stamp.toSec();
                string file_name(outfile_path+"pcd/"+ss.str()+".pcd");
                pcd_writer.write(file_name, laserCloud, false);
            }
            else pcd_writer.write(outfile_path+"pcd/"+std::to_string(count)+".pcd", laserCloud, false);
        }
        break;
    }
    case XYZINORMAL:
    {
        pcl::PointCloud<pcl::PointXYZINormal> laserCloud;
        pcl::fromROSMsg(*msg, laserCloud);
        if(!frame_wise) *pcl_wait_save5 += laserCloud;
        else{
            pcl::PCDWriter pcd_writer;
            if (pcd_name_stamp) {
                stringstream ss;
                ss.setf(std::ios::fixed);
                ss.precision(9);
                ss<<msg->header.stamp.toSec();
                string file_name(outfile_path+"pcd/"+ss.str()+".pcd");
                pcd_writer.write(file_name, laserCloud, false);
            }
            else pcd_writer.write(outfile_path+"pcd/"+std::to_string(count)+".pcd", laserCloud, false);
        }
        break;
    }
    default:
        break;
    }
    count++;
}
void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ROS_INFO("In LidarCallback!");
    if (save_pcd) {
        SavePCD(msg);
    }
    if (print_cloud) pointcloud_print(msg, lidar_type, 1000);
}

inline void SaveOdom(const nav_msgs::OdometryConstPtr &msg)
{
    (*outfile)<<msg->pose.pose.position.x<<" "\
              <<msg->pose.pose.position.y<<" "\
              <<msg->pose.pose.position.z<<" "\
              <<msg->pose.pose.orientation.w<<" "\
              <<msg->pose.pose.orientation.x<<" "\
              <<msg->pose.pose.orientation.y<<" "\
              <<msg->pose.pose.orientation.z<<endl;
}
void OdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    ROS_INFO("In OdomCallback!");
    SaveOdom(msg);
}
void SyncedCallback(const nav_msgs::Odometry::ConstPtr &msgOdom, const sensor_msgs::PointCloud2::ConstPtr &msgCloud)
{
    ROS_INFO("In SyncedCallback!");
    if (save_pcd) {
        SavePCD(msgCloud);
    }
    if (print_cloud) pointcloud_print(msgCloud, lidar_type, 1000);
    SaveOdom(msgOdom);
}
std::shared_ptr<ros::Subscriber> lidarSub;
std::shared_ptr<ros::Subscriber> odomSub;
std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> filterSubOdom;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> filterSubCloud;
typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;
std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> odom_cloud_sync;

int main(int argc, char **argv)
{
	std::cout.precision(20);//设置输出精度
    ros::init(argc,argv,"test"); 
    ros::NodeHandle nh("~");
    nh.param<bool>("save_odom", save_odom, false);
    nh.param<bool>("save_pcd", save_pcd, false);
    nh.param<bool>("pcd_name_stamp", pcd_name_stamp, false);
    nh.param<bool>("frame_wise", frame_wise, false);
    nh.param<bool>("print_cloud", print_cloud, false);
    nh.param<int>("lidar_type", lidar_type, 1);
    nh.param<string>("pointcloud_topic",pointcloud_topic,"/rslidar_points");
    nh.param<string>("odom_topic",odom_topic,"/odometry");
    nh.param<string>("outfile_path",outfile_path,"./");

    Common_tools::if_not_exist_then_create(outfile_path);
    if (save_pcd) {
        string pcd_path(outfile_path+"pcd/");
        Common_tools::if_not_exist_then_create(pcd_path);
        if(!frame_wise){
            switch (lidar_type)
            {
            case ROBOSENSE:
            {
                pcl_wait_save1.reset( new pcl::PointCloud<robosense_ros::Point>() );
                break;
            }
            case VELODYNE:
            {
                pcl_wait_save2.reset( new pcl::PointCloud<velodyne_ros::Point>() );
                break;
            }
            case OUSTER:
            {
                pcl_wait_save3.reset( new pcl::PointCloud<ouster_ros::Point>() );
                break;
            }
            case XYZI:
            {
                pcl_wait_save4.reset( new pcl::PointCloud<pcl::PointXYZI>() );
                break;
            }
            case XYZINORMAL:
            {
                pcl_wait_save5.reset( new pcl::PointCloud<pcl::PointXYZINormal>() );
                break;
            }
            default:
                break;
            }
        }
    }
    outfile = make_shared<std::ofstream>(outfile_path+"pose.csv", std::ios::out | std::ios::trunc);

    if (save_pcd && save_odom) {
        filterSubOdom = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh, odom_topic, 1000, ros::TransportHints().tcpNoDelay());
        filterSubCloud = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, pointcloud_topic, 1000, ros::TransportHints().tcpNoDelay());
        odom_cloud_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), *filterSubOdom, *filterSubCloud); // 1000是消息队列长度
        odom_cloud_sync->registerCallback(boost::bind(&SyncedCallback, _1, _2));
    }
    else if (save_pcd) {
        lidarSub = std::make_shared<ros::Subscriber>(nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 100, &LidarCallback));
    }
    else {
        odomSub = std::make_shared<ros::Subscriber>(nh.subscribe<nav_msgs::Odometry>(odom_topic, 100, &OdomCallback));
    }

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    /**************** save map ****************/
    if(save_pcd && !frame_wise){
        string file_name = string("scans.pcd");
        string all_points_dir(outfile_path  + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        switch (lidar_type)
        {
        case ROBOSENSE:
        {
            pcd_writer.write(all_points_dir, *pcl_wait_save1, false);
            break;
        }
        case VELODYNE:
        {
            pcd_writer.write(all_points_dir, *pcl_wait_save2, false);
            break;
        }
        case OUSTER:
        {
            pcd_writer.write(all_points_dir, *pcl_wait_save3, false);
            break;
        }
        case XYZI:
        {
            pcd_writer.write(all_points_dir, *pcl_wait_save4, false);
            break;
        }
        case XYZINORMAL:
        {
            pcd_writer.write(all_points_dir, *pcl_wait_save5, false);
            break;
        }
        default:
            break;
        }
    }
    
    return 0;
}
