#ifndef SAMPLE_CARTO_TOP_BAG_READER_H_
#define SAMPLE_CARTO_TOP_BAG_READER_H_

#include <deque>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <src/common/make_unique.h>

namespace sample_carto
{
namespace top
{

class BagReader
{
  public:
	bool is_scan(const rosbag::MessageInstance &message);
	bool is_odom(const rosbag::MessageInstance &message);
	bool is_finished();
	BagReader(std::string bagfile);
	const rosbag::MessageInstance next_msg();
  private:
	rosbag::Bag bag_;
	rosbag::View view_;
	std::deque<std::pair<double, std::unique_ptr<rosbag::MessageInstance>>> buff_;
};
}
}

#endif
