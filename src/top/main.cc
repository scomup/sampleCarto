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
    top::BagReader bagReader("/home/liu/bag/tokyo/lg_1.bag", "/scan", "/odom");
    //top::BagReader bagReader("/home/liu/bag/kusatsu/lg_kusatsu_C5_1.bag", "/scan", "/odom");
    auto file_resolver = common::make_unique<common::FileResolver>(std::vector<string>{std::string("/home/liu/workspace/sampleCarto/configuration_files")});
    const string code = file_resolver->GetFileContentOrDie("test.lua");
    common::LuaParameterDictionary parameter_dictionary(code, std::move(file_resolver));
    core::LocalMapBuilderOptions local_map_builder_options;
    local_map_builder_options.Create(parameter_dictionary.GetDictionary("trajectory_builder").get()->GetDictionary("trajectory_builder_2d").get());
	core::SparsePoseGraphOptions sparse_pose_graph_options;
    sparse_pose_graph_options.Create(parameter_dictionary.GetDictionary("map_builder").get()->GetDictionary("sparse_pose_graph").get());

    auto golbal_map_manager_ptr = std::make_shared<core::GlobalMapManager>(local_map_builder_options, sparse_pose_graph_options);
    top::SensorBridge sensor_bridge(golbal_map_manager_ptr);

	top::Publisher pub = top::Publisher(golbal_map_manager_ptr, 0.3, local_map_builder_options.submaps_options_.num_range_data_);
    std::thread t1(&top::Publisher::pcd, &pub);
    ros::Rate loop_rate(10000);

    while (ros::ok())
    {
        if (bagReader.scan_raw_datas_.size() == 0 || bagReader.odom_raw_datas_.size() == 0)
        {
            
            break;
        }
        auto scan = bagReader.scan_raw_datas_.front();
        auto odom = bagReader.odom_raw_datas_.front();
        if (scan.header.stamp < odom.header.stamp)
        {
            //static int c=0;
            //printf("%d\n",c++);
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
    //golbal_map_manager_ptr->sparse_pose_graph()->RunFinalOptimization();
    std::this_thread::sleep_for(std::chrono::seconds(10));

    pub.end();
    t1.join();

    std::cout<<"Finished!\n";

    return 0;
}