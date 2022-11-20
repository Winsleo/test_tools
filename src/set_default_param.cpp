#include <Eigen/Eigen>
#include <pcl/common/eigen.h>
#include <cmath>
#include <string>
//dynamic reconfigure depends
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <tools.hpp>
#include <test_tools/DeltaExtConfig.h>

std::string target_node_name("/test_tools");//动态参数服务端节点名

ros::ServiceClient set_service;

void config_callback(const test_tools::DeltaExtConfig& config){
	if(config.inverse){
		test_tools::DeltaExtConfig new_config;
		dynamic_reconfigure::Reconfigure srv;
		new_config.inverse = false;
		new_config.x = 0;
		new_config.y = 0;
		new_config.z = 0;
		new_config.roll = 0;
		new_config.pitch = 0;
		new_config.yaw = 0;
		new_config.__toMessage__(srv.request.config);
		set_service.call(srv);
	}
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "set_default_param");
	
  ros::NodeHandle private_nh(target_node_name);

  //设置初始参数 方法一：dynamic_reconfigure::client
	dynamic_reconfigure::Client<test_tools::DeltaExtConfig> client(target_node_name,&config_callback);  //订阅服务，并设置回调函数用于回读改变后的最新参数
  test_tools::DeltaExtConfig default_config;
	default_config.inverse = false;
	default_config.x = 0;
	default_config.y = 0;
	default_config.z = 0;
	default_config.roll = 0;
	default_config.pitch = 0;
	default_config.yaw = 0;
	client.setConfiguration(default_config);//用于更新参数

	// 设置初始参数 方法二：服务 ServiceClient
	set_service = private_nh.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");
	//等待服务端上线
	set_service.waitForExistence(ros::Duration(-1));//等价于ros::service::waitForService("/calibration_publisher/set_parameters");
	
	ros::spin();
  ros::shutdown();
  return 0;
}