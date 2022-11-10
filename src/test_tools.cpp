#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<iostream>
#include<Eigen/Eigen>
// #include<eigen3/Eigen/Core>
// #include<eigen3/Eigen/Geometry>
#include<pcl/common/eigen.h>
#include<pcl/conversions.h>
#include <tf/transform_datatypes.h>
#include<sensor_msgs/point_cloud2_iterator.h>
using namespace std;
using namespace Eigen;
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    // cout<<msg->header.frame_id<<endl;
    // if(msg->is_bigendian) cout<<"bigendian"<<"  ";
    // if(msg->is_dense) cout<<"dense"<<"  ";
    // cout<<endl;

    // ROS_INFO("%u %u %u %u %u %u",msg->height, msg->width, msg->fields.size(), msg->point_step, msg->row_step, msg->data.size());
    //robosense雷达原始数据
    //[INFO] [1663145314.620110459]: 32 1800 6 32 57600 1843200 
    //rs_to_velodyne处理之后的
    //[INFO] [1665644968.711082438]: 1 40945 5 32 1310240 1310240 
    // ROS_INFO("%s %u %u %u",msg->fields[0].name.c_str(), msg->fields[0].offset, msg->fields[0].datatype, msg->fields[0].count);
    // ROS_INFO("%s %u %u %u",msg->fields[1].name.c_str(), msg->fields[1].offset, msg->fields[1].datatype, msg->fields[1].count);
    // ROS_INFO("%s %u %u %u",msg->fields[2].name.c_str(), msg->fields[2].offset, msg->fields[2].datatype, msg->fields[2].count);
    // ROS_INFO("%s %u %u %u",msg->fields[3].name.c_str(), msg->fields[3].offset, msg->fields[3].datatype, msg->fields[3].count);
    // ROS_INFO("%s %u %u %u",msg->fields[4].name.c_str(), msg->fields[4].offset, msg->fields[4].datatype, msg->fields[4].count);
    // ROS_INFO("%s %u %u %u",msg->fields[5].name.c_str(), msg->fields[5].offset, msg->fields[5].datatype, msg->fields[5].count);
    // ROS_INFO("%s %u %u %u",msg->fields[6].name.c_str(), msg->fields[6].offset, msg->fields[6].datatype, msg->fields[6].count);
    // ROS_INFO("%s %u %u %u",msg->fields[7].name.c_str(), msg->fields[7].offset, msg->fields[7].datatype, msg->fields[7].count);
    
    // [ INFO] [1664521347.311617672]: x 0 7 1
    // [ INFO] [1664521347.311707675]: y 4 7 1
    // [ INFO] [1664521347.311745405]: z 8 7 1
    // [ INFO] [1664521347.311785224]: intensity 16 2 1
    // [ INFO] [1664521347.311828737]: ring 18 4 1
    // [ INFO] [1664521347.311869094]: timestamp 24 8 1

    //msg type : robosense 
    // sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
    // sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");
    // sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg,"z");
    // sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(*msg,"intensity");
    // sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg,"ring");
    // sensor_msgs::PointCloud2ConstIterator<double> iter_time(*msg,"timestamp");
    // cout<<"---------begin----------"<<msg->header.stamp<<endl;
    // cout.precision(15);
    // double begin_time = *iter_time;
    // for (int i=0; iter_x != iter_x.end(); ++iter_x,++iter_y,++iter_z,++iter_ring,++iter_time)
    // {
    //     if(*iter_time < begin_time) cout<<"error"<<endl;
    //     // if(i%100==0) cout<< *iter_time<<"  ";//每100个点输出一个
    //     // i++;
    // }
    // cout<<endl<<"---------end----------"<<endl;
    // ROS_INFO("x:%f y:%f z:%f intensity:%u ring:%u time:%f\n", *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_ring, *iter_time);
    //msg type : robosense

    //msg type : velodyne 
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg,"z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg,"intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg,"ring");
    sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg,"time");
    cout<<"---------begin----------"<<endl;
    cout.precision(15);
    double begin_time = *iter_time;
    for (int i=0; iter_x != iter_x.end(); ++iter_x,++iter_y,++iter_z,++iter_time)
    {
        if(*iter_time < begin_time) cout<<"error"<<endl;
        // if(i%100==0) cout<< *iter_time<<"  ";//每100个点输出一个
        // i++;
    }
    cout<<endl<<"---------end----------"<<endl;
    // ROS_INFO("x:%f y:%f z:%f intensity:%f ring:%u time:%f", *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_ring, *iter_time);
    //msg type : velodyne
    
    // msg type : pcl::PointCloud< PointPointXYZINormal>
/*     sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg,"z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg,"intensity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_curve(*msg,"curvature");
    cout<<"---------begin----------"<<endl;
    cout.precision(15);
    for (int i=0; iter_x != iter_x.end(); ++iter_x,++iter_y,++iter_z,++iter_curve)
    {
        if(i%100==0) cout<< *iter_curve<<"  ";//每100个点输出一个
        i++;
    }
    cout<<endl<<"---------end----------"<<endl; */
    // ROS_INFO("x:%f y:%f z:%f intensity:%f curvature:%f", *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_curve);
    // msg type : pcl::PointCloud< PointPointXYZINormal>
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",2,&lidar_callback);

    Eigen::Matrix3d rot_cam2lidar;//旋转矩阵
    rot_cam2lidar<<3.3876939304536846e-01, 2.0411991267696605e-02, 9.4064799417643996e-01,\
    -9.4086852267540699e-01, 8.7436721632269121e-03, 3.3865907818291047e-01,\
    -1.3120115354801065e-03, -9.9975341900370074e-01, 2.2167088580625049e-02;
    Eigen::Isometry3d T_cam2lidar=Eigen::Isometry3d::Identity();//欧式变换矩阵
    T_cam2lidar.rotate(rot_cam2lidar);//用旋转矩阵设置欧式变换矩阵的旋转部分
    T_cam2lidar.pretranslate(Eigen::Vector3d(0.16, 0.105, -0.095));//设置欧式变换矩阵的平移部分
    Eigen::Affine3d T_lidar2cam = T_cam2lidar.inverse();//欧式变换求逆，赋值为仿射变换
    cout<<T_lidar2cam.matrix()<<endl;//打印仿射变换的矩阵
    double x,y,z,roll,pitch,yaw;
    pcl::getTranslationAndEulerAngles(T_lidar2cam,x,y,z,roll,pitch,yaw);//获取仿射变换等效的x,y,z平移和欧拉角旋转(3-2-1姿态序列)
    cout<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<endl;
    Eigen::Affine3f A_lidar2cam= pcl::getTransformation(x, y, z, roll, pitch, yaw);//由x,y,z平移和roll,pitch,yaw欧拉角旋转(3-2-1姿态序列)生成对应的仿射变换
    cout<<A_lidar2cam.matrix()<<endl;

/*     Quaterniond q_cam2lidar(rot_cam2lidar);//旋转矩阵转四元数
    cout<<"Camera to Lidar quaternion:"<< q_cam2lidar.coeffs().transpose() <<endl;//coeffs的顺序是(x,y,z,w)，为实部，前三者为虚部
    Isometry3d T_cam2lidar(q_cam2lidar);//用四元数旋转构造Transform矩阵
    T_cam2lidar.pretranslate(Vector3d(0.16, 0.105, -0.095));//加上平移向量
    cout<< T_cam2lidar.matrix() <<endl;
    Isometry3d T_lidar2cam = T_cam2lidar.inverse();
    Vector3d euler_lidar2cam = T_lidar2cam.rotation().eulerAngles(2,1,0);//旋转矩阵转欧拉角(ZYX顺序，即yaw、pitch、roll顺序)
    cout<<"Lidar to Camera Euler angle:"<< euler_lidar2cam <<endl;
    cout<<"Lidar to Camera translation:"<<T_lidar2cam.translation()<<endl;
    Quaterniond q_imu2lidar(0.99999,-0.000918885,-0.00227739,-0.0036683);//构造函数是(w, x, y, z)
    q_imu2lidar.toRotationMatrix();//四元数转旋转矩阵
    Isometry3d T_imu2lidar=Isometry3d::Identity();
    T_imu2lidar.rotate(q_imu2lidar);//Transform用四元数作为旋转部分
    T_imu2lidar.pretranslate(Vector3d(-0.00134677,-0.00126342,-0.209773)); */
    
    ros::spin();
    return 0;
}
