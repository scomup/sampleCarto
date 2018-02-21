#include "bag_reader.h"


namespace sample_carto
{
namespace top
{

bool BagReader::is_scan(const rosbag::MessageInstance &message)
{
	if (message.isType<sensor_msgs::LaserScan>())
	{
		return true;
	}
	return false;
}

bool BagReader::is_odom(const rosbag::MessageInstance &message)
{
	if (message.isType<nav_msgs::Odometry>())
	{
		return true;
	}
	return false;
}

bool BagReader::is_finished()
{
	return !buff_.size();
}

BagReader::BagReader(std::string bagfile)
{
	try
	{
		bag_.open(bagfile, rosbag::bagmode::Read);
	}
	catch(rosbag::BagIOException &e)
	{
		std::cout << "Cann't open " << bagfile << ".\nPlease check the path of your bagfile.\n";
		exit(-1);
	}
	view_.addQuery(bag_);
	for (rosbag::MessageInstance m : view_)
	{
		if (is_scan(m))
		{
			auto scan_ptr = m.instantiate<sensor_msgs::LaserScan>();
			double stamp = scan_ptr->header.stamp.toSec();
			buff_.emplace_back(stamp, common::make_unique<rosbag::MessageInstance>(m));
		}
		else if (is_odom(m))
		{
			auto odom_ptr = m.instantiate<nav_msgs::Odometry>();
			double stamp = odom_ptr->header.stamp.toSec();
			buff_.emplace_back(stamp, common::make_unique<rosbag::MessageInstance>(m));
		}
	}
	std::sort(buff_.begin(), buff_.end());
}

const rosbag::MessageInstance BagReader::next_msg()
{
	rosbag::MessageInstance m = *(buff_.begin()->second);
	buff_.pop_front();
	return m;
}
}
}
