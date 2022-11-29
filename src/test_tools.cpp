#include<ros/ros.h>
#include<pcl/common/eigen.h>
#include<pcl/conversions.h>
#include"tools.hpp"
#include"tools_thread_pool.hpp"
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/buffer.h>
#include<tf2_ros/transform_listener.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2_eigen/tf2_eigen.h>
#include<csignal>  
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/CompressedImage.h>
#include<geometry_msgs/TransformStamped.h>
//dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <test_tools/DeltaExtConfig.h>
using namespace std;
using namespace Eigen;
Eigen::Isometry3d extrinsic_init;
Eigen::Isometry3d extrinsic;
// std::shared_ptr<Common_tools::ThreadPool> thread_pool_ptr;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<std::ofstream> outfile;
string pointcloud_topic,image_topic,compressed_topic,outfile_path;
string father_frame_id("father");
string child_frame_id("child");
string map_frame_id;
string camera_frame_id;
void SigHandle(int sig)
{
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
void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pointcloud_print(msg, ROBOSENSE, 1000);
}
void tfRegistration(const Eigen::Isometry3d &ext, const ros::Time &timeStamp,const string& frame_id,const string& child_frame_id)
{
	static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped geometry_tf = tf2::eigenToTransform(ext);
    geometry_tf.header.stamp=timeStamp;
    geometry_tf.header.frame_id=frame_id;
    geometry_tf.child_frame_id=child_frame_id;
	broadcaster.sendTransform(geometry_tf);
}
void reconfigure_callback(test_tools::DeltaExtConfig& config) {
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
void ImageCallback(const sensor_msgs::ImageConstPtr &msg){
    if( !tf_buffer->canTransform(map_frame_id, camera_frame_id, msg->header.stamp,ros::Duration(1)) ) return;
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    double timestamp = msg->header.stamp.toSec();
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
    cv::imwrite( outfile_path+ "image/" + std::to_string(timestamp) + ".jpg", cv_ptr_compressed->image);
}
void CompressedCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    if( !tf_buffer->canTransform(map_frame_id, camera_frame_id, msg->header.stamp,ros::Duration(1)) ) return;
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    double timestamp = msg->header.stamp.toSec();
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
    cv::imwrite( outfile_path+ "image/" + std::to_string(timestamp) + ".jpg", cv_ptr_compressed->image);
}

int main(int argc, char **argv)
{
	std::cout.precision(20);//设置输出精度

    ros::init(argc,argv,"test"); 
    ros::NodeHandle nh("~");
    tf_buffer=make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tf_listener(*tf_buffer);
    nh.param<string>("map_frame_id",map_frame_id,"map");
    nh.param<string>("camera_frame_id",camera_frame_id,"camera_link");
    nh.param<string>("pointcloud_topic",pointcloud_topic,"/rslidar_points");
    nh.param<string>("image_topic",image_topic,"/image_raw");
    nh.param<string>("compressed_topic",compressed_topic,"/image_raw/compressed");
    nh.param<string>("outfile_path",outfile_path,"./");
    outfile = make_shared<std::ofstream>(outfile_path+"pose.csv", std::ios::out | std::ios::trunc);
    (*outfile)<<"timestamp"<<","<<"x"<<","<<"y"<<","<<"z"<<","<<"qx"<<","<<"qy"<<","<<"qz"<<","<<"qw"<<endl;
    vector<double> vec_rot;
    vector<double> vec_trans;
    Eigen::Matrix3d rot_init;
    Eigen::Vector3d trans_init;
    if(nh.getParam("Rot", vec_rot)){
        rot_init << MAT3_FROM_ARRAY(vec_rot);
    }
    else ros::shutdown();
    nh.param<vector<double>>("Trans",vec_trans,vector<double>(3,0));
    trans_init << VEC3_FROM_ARRAY(vec_trans);
    extrinsic_init=Eigen::Isometry3d::Identity();//欧式变换矩阵
    extrinsic_init.rotate(rot_init);//用旋转矩阵设置欧式变换矩阵的旋转部分
    extrinsic_init.pretranslate(trans_init);//设置欧式变换矩阵的平移部分
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 2, &lidarCallback);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 50, &ImageCallback);
    ros::Subscriber compressed_sub = nh.subscribe<sensor_msgs::CompressedImage>(compressed_topic, 50, &CompressedCallback);
    //动态参数服务器
	dynamic_reconfigure::Server<test_tools::DeltaExtConfig> server;
	dynamic_reconfigure::Server<test_tools::DeltaExtConfig>::CallbackType server_callback = boost::bind(&reconfigure_callback, _1);
	//回调函数与服务端绑定
	server.setCallback(server_callback);
    signal(SIGINT, SigHandle);//处理键盘Ctrl+C信号
    // thread_pool_ptr = make_shared<Common_tools::ThreadPool>(2, true, false);
    // thread_pool_ptr->commit_task(&tf_register_thread);
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        tfRegistration(extrinsic,ros::Time::now(),father_frame_id,child_frame_id);
        rate.sleep();
    }
    
    return 0;
}
