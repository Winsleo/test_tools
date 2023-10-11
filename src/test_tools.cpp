#include <csignal>  
#include <ros/ros.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
//dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <test_tools/DeltaExtConfig.h>
// TimeSyncronizer headers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "tools.hpp"
#include "tools_thread_pool.hpp"
#include "common_tools.h"
using namespace std;
using namespace Eigen;
Eigen::Isometry3d extrinsic_init;
Eigen::Isometry3d extrinsic;
// std::shared_ptr<Common_tools::ThreadPool> thread_pool_ptr;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<std::ofstream> outfile;
string pointcloud_topic,image_topic,compressed_topic,odom_topic,outfile_path;
string father_frame_id("father");
string child_frame_id("child");
string map_frame_id;
string camera_frame_id;
int lidar_type;
bool print_cloud;
bool save_pose_image;
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

void SigHandle(int sig)
{
    flg_exit = true;
    scope_color(ANSI_COLOR_CYAN_BOLD);
    std::cout<<"========================================="<<std::endl;
    std::cout<<"Modified extrinsic matrix:"<<std::endl;
    auto mat=extrinsic.matrix();
    std::cout<< "["<<mat(0,0)<<", "<<mat(0,1)<<", "<<mat(0,2) <<std::endl;
    std::cout<<mat(1,0)<<", "<<mat(1,1)<<", "<<mat(1,2) <<std::endl;
    std::cout<<mat(2,0)<<", "<<mat(2,1)<<", "<<mat(2,2) <<"]"<<std::endl;
    std::cout<<"Euler angles(RPY): "<< extrinsic.matrix().block<3,3>(0,0).eulerAngles(0,1,2).transpose() <<std::endl;
    std::cout<<"Translation(XYZ):   ["<< extrinsic.matrix()(0,3)<<", "<<extrinsic.matrix()(1,3)<<", "<<extrinsic.matrix()(2,3) <<"]"<<std::endl;
    std::cout<<"========================================="<<std::endl;
    cout<<ANSI_COLOR_RESET;
    outfile->close();
}
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
void TfRegistration(const Eigen::Isometry3d &ext, const ros::Time &timeStamp,const string& frame_id,const string& child_frame_id)
{
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped geometry_tf = tf2::eigenToTransform(ext);
    geometry_tf.header.stamp=timeStamp;
    geometry_tf.header.frame_id=frame_id;
    geometry_tf.child_frame_id=child_frame_id;
    broadcaster.sendTransform(geometry_tf);
}
void ReconfigureCallback(test_tools::DeltaExtConfig& config)
{
    if(config.inverse){
        /*
        外参求逆
        */
        extrinsic_init = extrinsic_init.inverse();
        extrinsic = extrinsic_init;
    }
    else{
        /*
        外参微调
        */    
        auto delta = euler2matrix(deg2rad(config.roll), deg2rad(config.pitch), deg2rad(config.yaw));
        extrinsic = extrinsic_init;
        extrinsic.rotate(delta);
    }
}
void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("In ImageCallback!");
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    double timestamp = msg->header.stamp.toSec();
    stringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(9);
    ss<<timestamp;
    cv::imwrite( outfile_path+ "image/" + ss.str() + ".jpg", cv_ptr_compressed->image);
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer->lookupTransform(map_frame_id, camera_frame_id, msg->header.stamp);
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    (*outfile)<<std::to_string(timestamp)<<","<<transform.transform.translation.x<<","<<transform.transform.translation.y<<","<<transform.transform.translation.z<<","<<
    transform.transform.rotation.x<<","<<transform.transform.rotation.y<<","<<transform.transform.rotation.z<<","<<transform.transform.rotation.w<<endl;
}
void CompressedCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    ROS_INFO("In CompressedCallback!");
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    double timestamp = msg->header.stamp.toSec();
    stringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(9);
    ss<<timestamp;
    cv::imwrite( outfile_path+ "image/" + ss.str() + ".jpg", cv_ptr_compressed->image);
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer->lookupTransform(map_frame_id, camera_frame_id, msg->header.stamp);
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    (*outfile)<<std::to_string(timestamp)<<","<<transform.transform.translation.x<<","<<transform.transform.translation.y<<","<<transform.transform.translation.z<<","<<
    transform.transform.rotation.x<<","<<transform.transform.rotation.y<<","<<transform.transform.rotation.z<<","<<transform.transform.rotation.w<<endl;
}

inline void SaveOdom(const nav_msgs::OdometryConstPtr &msg)
{
    (*outfile)<<msg->pose.pose.position.x<<" "\
              <<msg->pose.pose.position.y<<" "\
              <<msg->pose.pose.position.z<<","\
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
std::shared_ptr<ros::Subscriber> imageSub; 
std::shared_ptr<ros::Subscriber> compressedSub;
std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> filterSubOdom;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> filterSubCloud;
typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;
std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> odom_cloud_sync;

int main(int argc, char **argv)
{
	std::cout.precision(20);//设置输出精度

    ros::init(argc,argv,"test"); 
    ros::NodeHandle nh("~");
    tf_buffer=make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf2_ros::TransformListener tf_listener(*tf_buffer);
    nh.param<bool>("save_pose_image", save_pose_image, false);
    nh.param<bool>("save_odom", save_odom, false);
    nh.param<bool>("save_pcd", save_pcd, false);
    nh.param<bool>("pcd_name_stamp", pcd_name_stamp, false);
    nh.param<bool>("frame_wise", frame_wise, false);
    nh.param<bool>("print_cloud", print_cloud, false);
    nh.param<int>("lidar_type", lidar_type, 1);
    nh.param<string>("map_frame_id",map_frame_id,"map");
    nh.param<string>("camera_frame_id",camera_frame_id,"camera_link");
    nh.param<string>("pointcloud_topic",pointcloud_topic,"/rslidar_points");
    nh.param<string>("image_topic",image_topic,"/image_raw");
    nh.param<string>("compressed_topic",compressed_topic,"/image_raw/compressed");
    nh.param<string>("odom_topic",odom_topic,"/odometry");
    nh.param<string>("outfile_path",outfile_path,"./");

    Common_tools::if_not_exist_then_create(outfile_path);
    if (save_pose_image) {
        string image_path(outfile_path+"image/");
        Common_tools::if_not_exist_then_create(image_path);
    }
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
    if (save_pose_image) {
        (*outfile)<<"timestamp"<<","<<"x"<<","<<"y"<<","<<"z"<<","<<"qx"<<","<<"qy"<<","<<"qz"<<","<<"qw"<<endl;
    }
    vector<double> vec_rot;
    vector<double> vec_trans;
    Eigen::Matrix3d rot_init;
    Eigen::Vector3d trans_init;
    if (nh.getParam("Rot", vec_rot)) {
        rot_init << MAT3_FROM_ARRAY(vec_rot);
    }
    else ros::shutdown();
    nh.param<vector<double>>("Trans",vec_trans,vector<double>(3,0));
    trans_init << VEC3_FROM_ARRAY(vec_trans);
    extrinsic_init=Eigen::Isometry3d::Identity();//欧式变换矩阵
    extrinsic_init.rotate(rot_init);//用旋转矩阵设置欧式变换矩阵的旋转部分
    extrinsic_init.pretranslate(trans_init);//设置欧式变换矩阵的平移部分
    if (save_pose_image) {
        imageSub = std::make_shared<ros::Subscriber>(nh.subscribe<sensor_msgs::Image>(image_topic, 100, &ImageCallback));
        compressedSub = std::make_shared<ros::Subscriber>(nh.subscribe<sensor_msgs::CompressedImage>(compressed_topic, 100, &CompressedCallback));
    }
    if (save_pcd && save_odom) {
        filterSubOdom = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh, odom_topic, 1000, ros::TransportHints().tcpNoDelay());
        filterSubCloud = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, pointcloud_topic, 1000, ros::TransportHints().tcpNoDelay());
        odom_cloud_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(1000), *filterSubOdom, *filterSubCloud); // 1000是消息队列长度
        odom_cloud_sync->registerCallback(boost::bind(&SyncedCallback, _1, _2));
    }
    else if (save_pcd) {
        lidarSub = std::make_shared<ros::Subscriber>(nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 2, &LidarCallback));
    }
    else {
        odomSub = std::make_shared<ros::Subscriber>(nh.subscribe<nav_msgs::Odometry>(odom_topic, 100, &OdomCallback));
    }
    //动态参数服务器
    dynamic_reconfigure::Server<test_tools::DeltaExtConfig> server;
    dynamic_reconfigure::Server<test_tools::DeltaExtConfig>::CallbackType server_callback = boost::bind(&ReconfigureCallback, _1);
    //回调函数与服务端绑定
    server.setCallback(server_callback);
    signal(SIGINT, SigHandle);//处理键盘Ctrl+C信号
    // thread_pool_ptr = make_shared<Common_tools::ThreadPool>(2, true, false);
    // thread_pool_ptr->commit_task(&tf_register_thread);
    ros::Rate rate(10);
    while(ros::ok()){
        if(flg_exit) break;
        ros::spinOnce();
        TfRegistration(extrinsic,ros::Time::now(),father_frame_id,child_frame_id);
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
