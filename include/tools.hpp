#ifndef __TOOLS_HPP__
#define __TOOLS_HPP__
#include<cmath>
#include<vector>
#include<Eigen/Eigen>
#include"tools_color_printf.hpp"
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud2_iterator.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>

#define VEC3_FROM_ARRAY(v)  v[0],v[1],v[2]
#define MAT3_FROM_ARRAY(v)  v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define MAT4_FROM_ARRAY(v)  v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10].v[11],v[12],v[13],v[14],v[15]
enum CLOUD_TYPE{ROBOSENSE = 1, VELODYNE, OUSTER, XYZI, XYZINORMAL}; //{1, 2, 3 ,4, 5}
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      std::uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t t;
      std::uint16_t reflectivity;
      std::uint8_t  ring;
      std::uint16_t ambient;
      std::uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

//ANCHOR robosense modify
namespace robosense_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        std::uint8_t intensity;
        std::uint16_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint8_t, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

template<typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / M_PI;
}

template<typename T>
T deg2rad(T degrees)
{
  return degrees * M_PI / 180.0;
}
//0-X 1-Y 2-Z <>里代表欧拉角的旋转顺序 roll-X轴旋转角 pitch-Y轴旋转角 yaw-Z轴旋转角
template<int axis0=0,int axis1=1,int axis2=2,typename T>
inline Eigen::Matrix3d euler2matrix(T roll, T pitch, T yaw){
    std::vector<std::shared_ptr<Eigen::AngleAxisd>> eulerAngleAxis(3);
    eulerAngleAxis[0] = std::make_shared<Eigen::AngleAxisd>(roll, Eigen::Vector3d::UnitX());
    eulerAngleAxis[1] = std::make_shared<Eigen::AngleAxisd>(pitch, Eigen::Vector3d::UnitY());
    eulerAngleAxis[2] = std::make_shared<Eigen::AngleAxisd>(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d res;
    res=(*eulerAngleAxis[axis0]) * (*eulerAngleAxis[axis1]) * (*eulerAngleAxis[axis2]);
    return res;
}
//对接Eigen::Vector3d Eigen::MatrixBase<T>::eulerAngles(Eigen::Index a0, Eigen::Index a1, Eigen::Index a2)函数
template<int axis0=0,int axis1=1,int axis2=2,typename T>
inline Eigen::Matrix3d euler2matrix(const Eigen::Matrix<T,3,1> &eulerAngles){
    static Eigen::Vector3d axisXYZ[3]={Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ()};
    Eigen::Matrix3d res;
    res = Eigen::AngleAxisd(eulerAngles[0],axisXYZ[axis0]) * \
          Eigen::AngleAxisd(eulerAngles[1],axisXYZ[axis1]) * \
          Eigen::AngleAxisd(eulerAngles[2],axisXYZ[axis2]);
    return res;
}

void pointcloud_print(const sensor_msgs::PointCloud2ConstPtr& msg, int cloudType,int filterNum){
    scope_color(ANSI_COLOR_CYAN_BOLD);
    std::cout<<"frame_id:"<<msg->header.frame_id << "  header.stamp:" << msg->header.stamp.toSec()<<endl;
    if(msg->is_bigendian) std::cout<<"bigendian"<<endl;
    if(msg->is_dense) std::cout<<"dense"<<endl;
    printf("height=%u width=%u fields_num=%u point_step=%u row_step=%u data.size()=%u\n",msg->height, msg->width, msg->fields.size(), msg->point_step, msg->row_step, msg->data.size());
    //robosense雷达原始数据
    //[INFO] [1663145314.620110459]: 32 1800 6 32 57600 1843200 
    //rs_to_velodyne处理之后的
    //[INFO] [1665644968.711082438]: 1 40945 5 32 1310240 1310240 
    for(int i=0;i<msg->fields.size();i++){
        printf("fields[%d] name=%s offset=%u datatype=%u count=%u\n", i, msg->fields[i].name.c_str(), msg->fields[i].offset, msg->fields[i].datatype, msg->fields[i].count);
    }
    // [ INFO] [1664521347.311617672]: x 0 7 1
    // [ INFO] [1664521347.311707675]: y 4 7 1
    // [ INFO] [1664521347.311745405]: z 8 7 1
    // [ INFO] [1664521347.311785224]: intensity 16 2 1
    // [ INFO] [1664521347.311828737]: ring 18 4 1
    // [ INFO] [1664521347.311869094]: timestamp 24 8 1
    std::cout.precision(15);
    switch (cloudType)
    {
    case ROBOSENSE:
    {
        //msg type : robosense 
        pcl::PointCloud<robosense_ros::Point>::Ptr pcl_orig(new pcl::PointCloud<robosense_ros::Point>);
        pcl::fromROSMsg(*msg, *pcl_orig);
        std::cout<<"---------begin----------"<<endl;
        for (size_t i = 0; i < pcl_orig->size(); i++)
        {
            if(i % 100 == 0)//降采样
            {
                printf("x:%f y:%f z:%f intensity:%u ring:%u time:%f \n", pcl_orig->points[i].x, pcl_orig->points[i].y, pcl_orig->points[i].z, pcl_orig->points[i].intensity, pcl_orig->points[i].ring, pcl_orig->points[i].timestamp);
            }
        }
        std::cout<<"---------end----------"<<endl;
        
        //msg type : robosense
        break;
    }
    case VELODYNE:
    {
        //msg type : velodyne 
        pcl::PointCloud<velodyne_ros::Point>::Ptr pcl_orig(new pcl::PointCloud<velodyne_ros::Point>);
        pcl::fromROSMsg(*msg, *pcl_orig);
        std::cout<<"---------begin----------"<<endl;
        for (size_t i = 0; i < pcl_orig->size(); i++)
        {
            if(i % 100 == 0)//降采样
            {
                printf("x:%f y:%f z:%f intensity:%f time:%f ring:%u \n", pcl_orig->points[i].x, pcl_orig->points[i].y, pcl_orig->points[i].z, pcl_orig->points[i].intensity, pcl_orig->points[i].time, pcl_orig->points[i].ring);
            }
        }
        std::cout<<"---------end----------"<<endl;
        //msg type : velodyne
        break;
    }
    case XYZINORMAL:
    {
        // msg type : pcl::PointCloud< pcl::PointXYZINormal>
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_orig(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::fromROSMsg(*msg, *pcl_orig);
        std::cout<<"---------begin----------"<<endl;
        for (size_t i = 0; i < pcl_orig->size(); i++)
        {
            if(i%100==0)//每100个点输出一个
            {
                printf("x:%f y:%f z:%f intensity:%f curvature:%f \n", pcl_orig->points[i].x, pcl_orig->points[i].y, pcl_orig->points[i].z, pcl_orig->points[i].intensity, pcl_orig->points[i].curvature);
            }
        }
        std::cout<<"---------end----------"<<endl;
        // msg type : pcl::PointCloud< PointPointXYZINormal>
        break;
    }
    default:
        break;
    }
}
#endif