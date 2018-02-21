#ifndef SAMPLE_CARTO_TOP_MAPPUBLISHER_H_
#define SAMPLE_CARTO_TOP_MAPPUBLISHER_H_

#include "visualization_msgs/MarkerArray.h"
#include <src/core/global_manager/global_map_manager.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <src/top/sensor_bridge.h>

namespace sample_carto
{

namespace top
{

class Publisher
{
  public:
    explicit Publisher(std::shared_ptr<core::GlobalMapManager> global_map_builder_ptr,
                       double map_pub_period, int node_num_per_submap) ;
    void pcd();

    void finish();


  private:
    geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d &vector3d);

    std_msgs::ColorRGBA colorBar(double xbrt);

    void CreateCVSubmapData(int submap_id);

    void SaveContraintImage(int submap_id,int node_id, sample_carto::core::Node node, const sample_carto::transform::Rigid3d trans);

    visualization_msgs::MarkerArray GetConstraintList();

    visualization_msgs::MarkerArray GetTrajectoryNodeList();
    void makeMap();

  private:
    std::shared_ptr<core::GlobalMapManager> global_map_builder_ptr_;
    double map_pub_period_;
    nav_msgs::GetMap::Response map_;
    ros::Publisher mapPublisher_;
    ros::Publisher node_list_publisher_;
    ros::Publisher constraint_list_publisher_;
    ros::NodeHandle node_;
    int node_num_per_submap_;
    std::map<int, cv::Mat> cv_submaps_;
    bool finish_;
};

} //top
} //sample_carto

#endif //SAMPLE_CARTO_TOP_MAPPUBLISHER_H_
