#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <memory>

#include <string>
#include <src/common/file_resolver.h>
#include <src/common/make_unique.h>
#include <src/common/lua_parameter_dictionary.h>
#include <src/core/global_manager/global_map_manager.h>
#include <src/top/sensor_bridge.h>

#include "BagReader.h"
#include "MapPublisher.h"

using namespace sample_carto;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scample_carto");
	ros::start();

    top::BagReader bagReader("/home/liu/bag/kusatsu/lg_kusatsu_C5_1.bag", "/scan", "/odom");
    auto file_resolver = common::make_unique<common::FileResolver>(std::vector<string>{std::string("/home/liu/catkin_ws_carto/src/cartographer_ros/cartographer_ros/configuration_files")});
    const string code = file_resolver->GetFileContentOrDie("test.lua");
    common::LuaParameterDictionary parameter_dictionary(code, std::move(file_resolver));
    core::LocalMapBuilderOptions local_map_builder_options;
    local_map_builder_options.Create(parameter_dictionary.GetDictionary("trajectory_builder").get()->GetDictionary("trajectory_builder_2d").get());
	core::SparsePoseGraphOptions sparse_pose_graph_options;
    sparse_pose_graph_options.Create(parameter_dictionary.GetDictionary("map_builder").get()->GetDictionary("sparse_pose_graph").get());

    auto golbal_map_manager_ptr = std::make_shared<core::GlobalMapManager>(local_map_builder_options, sparse_pose_graph_options);
    top::SensorBridge sensor_bridge(golbal_map_manager_ptr);

	top::Publisher pub = top::Publisher(golbal_map_manager_ptr, 0.3);
	new std::thread(&top::Publisher::pcd, &pub);

    ros::Rate loop_rate(10000);

    while (ros::ok())
    {
        auto scan = bagReader.scan_raw_datas_.front();
        auto odom = bagReader.odom_raw_datas_.front();
        if (scan.header.stamp < odom.header.stamp)
        {
            auto scan_ptr = boost::make_shared<const ::sensor_msgs::LaserScan>(scan);
            sensor_bridge.HandleLaserScanMessage(scan_ptr);
            bagReader.scan_raw_datas_.pop_front();
        }
        else
        {
            auto odom_ptr = boost::make_shared<const ::nav_msgs::Odometry>(odom);
            sensor_bridge.HandleOdometryMessage(odom_ptr);
            bagReader.odom_raw_datas_.pop_front();
        }

        loop_rate.sleep();
    }

    return 0;
}