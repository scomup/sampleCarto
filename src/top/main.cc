#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <memory>

#include <string>
#include <src/common/file_resolver.h>
#include <src/common/make_unique.h>
#include <src/common/lua_parameter_dictionary.h>
#include <src/core/global_manager/global_map_manager.h>

#include "bag_reader.h"
#include "map_publisher.h"

using namespace sample_carto;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scample_carto");
    ros::start();

    std::string filename;
    if (argc >= 2)
    {
        filename = argv[1];
    }
    else
    {
        filename = "/home/liu/bag/tokyo/lg_1.bag";
    }

    top::BagReader bagReader(filename);

    auto file_resolver = common::make_unique<common::FileResolver>(std::vector<string>{std::string("/home/liu/workspace/sampleCarto/configuration_files")});
    const string code = file_resolver->GetFileContentOrDie("test.lua");
    common::LuaParameterDictionary parameter_dictionary(code, std::move(file_resolver));
    core::LocalMapBuilderOptions local_map_builder_options;
    local_map_builder_options.Create(parameter_dictionary.GetDictionary("trajectory_builder").get()->GetDictionary("trajectory_builder_2d").get());
    core::SparsePoseGraphOptions sparse_pose_graph_options;
    sparse_pose_graph_options.Create(parameter_dictionary.GetDictionary("map_builder").get()->GetDictionary("sparse_pose_graph").get());

    double use_odometry = parameter_dictionary.GetBool("use_odometry");
    double baselink_to_laser_x = parameter_dictionary.GetDouble("baselink_to_laser_x");
    double baselink_to_laser_y = parameter_dictionary.GetDouble("baselink_to_laser_y");
    double baselink_to_laser_theta = parameter_dictionary.GetDouble("baselink_to_laser_theta");

    auto golbal_map_manager_ptr = std::make_shared<core::GlobalMapManager>(local_map_builder_options, sparse_pose_graph_options);
    top::SensorBridge sensor_bridge(golbal_map_manager_ptr,
                                    baselink_to_laser_x,
                                    baselink_to_laser_y,
                                    baselink_to_laser_theta);

    top::Publisher pub =  top::Publisher(golbal_map_manager_ptr, 0.3, local_map_builder_options.submaps_options_.num_range_data_);
    std::thread t1(&top::Publisher::pcd, &pub);
    ros::Rate loop_rate(10000);

    while (ros::ok())
    {
        if (bagReader.is_finished())
        {
            break;
        }
        const rosbag::MessageInstance &message = bagReader.next_msg();
        if (bagReader.is_scan(message))
        {
            auto scan_ptr = message.instantiate<sensor_msgs::LaserScan>();
            sensor_bridge.HandleLaserScanMessage(scan_ptr);
        }
        else if (bagReader.is_odom(message))
        {
            if (use_odometry)
            {
                auto odom_ptr = message.instantiate<nav_msgs::Odometry>();
                sensor_bridge.HandleOdometryMessage(odom_ptr);
            }
        }
        loop_rate.sleep();
    }
    golbal_map_manager_ptr->sparse_pose_graph()->RunFinalOptimization();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    pub.finish();

    t1.join();

    std::cout << "Finished!\n";

    return 0;
}
