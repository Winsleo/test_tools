#ifndef __TOOLS_HPP__
#define __TOOLS_HPP__
#include<cmath>
#include<vector>
#include"tools_color_printf.hpp"
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud2_iterator.h>
#include<pcl/point_types.h>
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

void pointcloud_print(const sensor_msgs::PointCloud2ConstPtr& msg, int cloudType,int filterNum){
    scope_color(ANSI_COLOR_CYAN_BOLD);
    std::cout<<msg->header.frame_id<<endl;
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
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg,"z");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(*msg,"intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg,"ring");
        sensor_msgs::PointCloud2ConstIterator<double> iter_time(*msg,"timestamp");
        std::cout<<"---------begin----------"<<std::endl;
        double begin_time = *iter_time;
        for (int i=0; iter_x != iter_x.end(); ++iter_x,++iter_y,++iter_z,++iter_ring,++iter_time,i++)
        {
            if(*iter_time < begin_time) std::cerr<<"error"<<endl;
            if(i % filterNum == 0)//降采样
            {
                printf("x:%f y:%f z:%f intensity:%u ring:%u time:%f\n", *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_ring, *iter_time);
            }
        }
        std::cout<<"---------end----------"<<endl;
        
        //msg type : robosense
        break;
    }
    case VELODYNE:
    {
        //msg type : velodyne 
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg,"z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg,"intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg,"ring");
        sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg,"time");
        std::cout<<"---------begin----------"<<endl;
        double begin_time = *iter_time;
        for (int i=0; iter_x != iter_x.end(); ++iter_x,++iter_y,++iter_z,++iter_time,i++)
        {
            if(*iter_time < begin_time) std::cerr<<"error"<<endl;
            if(i % filterNum == 0)//降采样
            {
                printf("x:%f y:%f z:%f intensity:%f ring:%u time:%f", *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_ring, *iter_time);
            }
            
        }
        std::cout<<"---------end----------"<<endl;
        //msg type : velodyne
        break;
    }
    case XYZINORMAL:
    {
        // msg type : pcl::PointCloud< PointPointXYZINormal>
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg,"z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg,"intensity");
        sensor_msgs::PointCloud2ConstIterator<float> iter_curve(*msg,"curvature");
        std::cout<<"---------begin----------"<<endl;
        for (int i=0; iter_x != iter_x.end(); ++iter_x,++iter_y,++iter_z,++iter_curve,i++)
        {
            if(i%100==0)//每100个点输出一个
            {
                printf("x:%f y:%f z:%f intensity:%f curvature:%f\n", *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_curve);
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