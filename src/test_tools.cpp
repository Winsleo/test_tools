#include<ros/ros.h>
#include<pcl/common/eigen.h>
#include<pcl/conversions.h>
#include"tools.hpp"
#include"tools_thread_pool.hpp"
#include<tf2_ros/transform_broadcaster.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2_eigen/tf2_eigen.h>
#include <csignal>  
//dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <test_tools/DeltaExtConfig.h>
using namespace std;
using namespace Eigen;
Eigen::Isometry3d extrinsic_init;
Eigen::Isometry3d extrinsic;
// std::shared_ptr<Common_tools::ThreadPool> thread_pool_ptr;
void SigHandle(int sig)
{
	std::cout.precision(10);//设置输出精度
    scope_color(ANSI_COLOR_CYAN_BOLD);
	std::cout<<"========================================="<<std::endl;
    std::cout<<"Modified extrinsic matrix:"<<std::endl;
	std::cout<< extrinsic.matrix() <<std::endl;
	std::cout<<"Euler angles(RPY): "<< extrinsic.matrix().block<3,3>(0,0).eulerAngles(0,1,2).transpose() <<std::endl;
	std::cout<<"Translation(XYZ):    "<< extrinsic.matrix()(0,3)<<"   "<<extrinsic.matrix()(1,3)<<"   "<<extrinsic.matrix()(2,3) <<std::endl;
	std::cout<<"========================================="<<std::endl;
	cout<<ANSI_COLOR_RESET;
}
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
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
    cout.precision(10);//设置cout的数字输出有效位数
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
string father_frame_id("world");
string child_frame_id("baselink");
int main(int argc, char **argv)
{
    ros::init(argc,argv,"test"); 
    ros::NodeHandle nh("~");
    string pointcloud_topic;
    nh.param<string>("pointcloud_topic",pointcloud_topic,"/rslidar_points");
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
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 2, &lidar_callback);
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
