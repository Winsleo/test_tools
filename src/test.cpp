#include<fstream>
#include<string>
#include<iostream>
#include<thread>
#include<nav_msgs/Path.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_ros/transform_broadcaster.h>
using namespace std;
string csv_path;
shared_ptr<std::ifstream> in_file;
void pathCallback(nav_msgs::Path::ConstPtr msg){
    std::ofstream out_file(csv_path+"pose_out.csv", std::ios::trunc | std::ios::out );
    out_file.precision(20);
    for(auto pose_stamped:msg->poses){
        double timestamp = pose_stamped.header.stamp.toSec();
        stringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(20);
        ss<<timestamp;
        out_file<<std::to_string(timestamp)<<" "<<pose_stamped.pose.position.x<<" "<<pose_stamped.pose.position.y<<" "<<pose_stamped.pose.position.z<<" "<<
        pose_stamped.pose.orientation.x<<" "<<pose_stamped.pose.orientation.y<<" "<<pose_stamped.pose.orientation.z<<" "<<pose_stamped.pose.orientation.w<<endl;
    }  
    out_file.close();
}
void readPose(){
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
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"test"); 
    ros::NodeHandle nh("~");
    string path_topic;
    ros::Subscriber path_sub;
    std::shared_ptr<std::thread> read_thread;
    bool read_pose_from_csv, print_pose_from_path;
    nh.param<string>("csv_path", csv_path, "./");
    nh.param<string>("path_topic", path_topic, "/path");
    nh.param<bool>("read_pose_from_csv",read_pose_from_csv,false);
    nh.param<bool>("print_pose_from_path",print_pose_from_path,false);
    if(print_pose_from_path){
        path_sub = nh.subscribe(path_topic, 10000, &pathCallback);
    }
    if(read_pose_from_csv){
        in_file = make_shared<std::ifstream>(csv_path+"pose_in.csv", std::ios::in );  
        read_thread = make_shared<std::thread>(&readPose);
    } 

    // ros::Rate rate(10);
    // while(ros::ok()){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    ros::spin();
    read_thread->join();
    ros::shutdown();
    return 0;
}
