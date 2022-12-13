#include<fstream>
#include<string>
#include<iostream>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_ros/transform_broadcaster.h>
using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"test"); 
    ros::NodeHandle nh("~");
    string csv_path;
    nh.param<string>("csv_path",csv_path,"./");
    shared_ptr<std::ifstream> in_file = make_shared<std::ifstream>(csv_path+"pose.csv", std::ios::in );
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "camera_link";
    tf2_ros::TransformBroadcaster broadcaster;
    string csv_line;
    getline(*in_file, csv_line);
    ros::Rate r(30);
    while(getline(*in_file, csv_line)){
        stringstream ss(csv_line);
        string temp;
        getline(ss, temp, ',');
        transform.header.stamp.fromSec( stod(temp) );
        getline(ss, temp, ',');
        transform.transform.translation.x = stod(temp);
        getline(ss, temp, ',');
        transform.transform.translation.y = stod(temp);
        getline(ss, temp, ',');
        transform.transform.translation.z = stod(temp);
        getline(ss, temp, ',');
        transform.transform.rotation.x = stod(temp);
        getline(ss, temp, ',');
        transform.transform.rotation.y = stod(temp);
        getline(ss, temp, ',');
        transform.transform.rotation.z = stod(temp);
        getline(ss, temp, ',');
        transform.transform.rotation.w = stod(temp);
        
        broadcaster.sendTransform(transform);
        r.sleep();
    }

    ros::shutdown();
    return 0;
}
