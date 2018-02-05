#ifndef SAMPLE_CARTO_TOP_MAPPUBLISHER_H_
#define SAMPLE_CARTO_TOP_MAPPUBLISHER_H_

#include "visualization_msgs/MarkerArray.h"
#include <src/core/global_manager/global_map_manager.h>



namespace sample_carto {

namespace top {

class Publisher
{
  public:
    explicit Publisher(std::shared_ptr<core::GlobalMapManager>
                           global_map_builder_ptr,
                       double map_pub_period) : global_map_builder_ptr_(global_map_builder_ptr), map_pub_period_(map_pub_period)
    {
        mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        node_list_publisher_ = node_.advertise<::visualization_msgs::MarkerArray>("node_list", 1);
        constraint_list_publisher_ = node_.advertise<::visualization_msgs::MarkerArray>("constraints", 1);
    };

    geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d &vector3d)
    {
        geometry_msgs::Point point;
        point.x = vector3d.x();
        point.y = vector3d.y();
        point.z = vector3d.z();
        return point;
    }

    std_msgs::ColorRGBA colorBar(double xbrt)
    {
        assert(xbrt >= 0 && xbrt <= 1);
        xbrt = 1.0 - xbrt;
        std_msgs::ColorRGBA color;
        if (xbrt >= 0 && xbrt <= 0.25)
        {
            color.b = 1;
            color.g = (xbrt / 0.25);
            color.r = 0;
        }
        else if (xbrt > 0.25 && xbrt <= 0.5)
        {
            color.b = (2 - (xbrt / 0.25));
            color.g = 1;
            color.r = 0;
        }
        else if (xbrt > 0.5 && xbrt <= 0.75)
        {
            color.b = 0;
            color.g = 1;
            color.r = (xbrt / 0.25 * 1 - 2);
        }
        else
        {
            color.b = 0;
            color.g = (4 - (xbrt / 0.25));
            color.r = 1;
        }
        color.a = 1.0;
        return color;
    }

    visualization_msgs::MarkerArray GetConstraintList()
    {
        visualization_msgs::MarkerArray constraint_list;

        visualization_msgs::MarkerArray markers_array;
        const auto nodes = global_map_builder_ptr_->sparse_pose_graph()->GetNodes();
        const auto submaps = global_map_builder_ptr_->sparse_pose_graph()->GetAllSubmapData();
        const auto constraints = global_map_builder_ptr_->sparse_pose_graph()->constraints();

        for (const auto &constraint : constraints)
        {
            if (constraint.tag == sample_carto::core::sparse_pose_graph::Constraint::INTER_SUBMAP)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "/map";
                marker.header.stamp = ros::Time::now();
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.pose.position.z = 0.1;
                std::stringstream name;
                name << "constraints ";
                marker.id = 1;

                const auto &submap = submaps[constraint.submap_id];
                const auto &submap_pose = submap.pose;
                const auto &node_pose = nodes.at(constraint.node_id).pose;
                const sample_carto::transform::Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

                marker.points.push_back(ToGeometryMsgPoint(constraint_pose.translation()));
                marker.points.push_back(ToGeometryMsgPoint(node_pose.translation()));
                marker.color = colorBar(0.99);

                //marker.pose.position.x = node_point.x;
                //marker.pose.position.y = node_point.y;
                std::cout<<constraint.submap_id <<" to "<<constraint.node_id<<"\n";
                std::cout<<constraint_pose.translation()<<"\n";
                std::cout<<node_pose.translation()<<"\n";

                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.action = visualization_msgs::Marker::ADD;

                markers_array.markers.push_back(marker);
            }
            
        }
        std::cout<<"---------------\n";
        return markers_array;
    }

    visualization_msgs::MarkerArray GetTrajectoryNodeList()
    {
        const auto nodes = global_map_builder_ptr_->sparse_pose_graph()->GetNodes();
        visualization_msgs::MarkerArray markers_array;
        for (const auto &node_id_data : nodes)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            if (node_id_data.first % 20 == 0)
            {
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.5;
            }
            else
            {
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;
            }
            marker.pose.position.z = 0.1;
            std::stringstream name;
            name << "node " << node_id_data.first;
            marker.ns = name.str();
            marker.id = node_id_data.first;
            const ::geometry_msgs::Point node_point = ToGeometryMsgPoint(node_id_data.second.pose.translation());
            marker.color = colorBar( ((node_id_data.first /20) % 10)/10.);
            marker.points.push_back(node_point);

            marker.pose.position.x = node_point.x;
            marker.pose.position.y = node_point.y;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            
            markers_array.markers.push_back(marker);
        }
        return markers_array;
    };

    void makeMap()
    {
        
        const auto all_submap_data = global_map_builder_ptr_->sparse_pose_graph()->GetAllSubmapData();
        if (all_submap_data.size() == 0)
            return;
        std::vector<int8_t> &data = map_.map.data;

        map_.map.info.origin.position.x = -100;
        map_.map.info.origin.position.y = -100;
        map_.map.info.origin.orientation.w = 1;
        map_.map.info.resolution = 0.05;
        map_.map.info.width = 4000;
        map_.map.info.height = 4000;

        map_.map.header.frame_id = "map";
        map_.map.data.resize(map_.map.info.width * map_.map.info.height);
        map_.map.header.stamp = ros::Time::now();
        memset(&map_.map.data[0], -1, sizeof(int8_t) * map_.map.data.size());
        
        for (core::SparsePoseGraph::SubmapDataWithPose m : all_submap_data)
        {
            Eigen::Array2i offset;
            core::map::CellLimits limits;
            const auto& grid = m.submap->probability_grid();
            grid.ComputeCroppedLimits(&offset, &limits);
            auto to_optimized = m.pose * m.submap->local_pose().inverse();

            for (const Eigen::Array2i &xy_index : core::map::XYIndexRangeIterator(limits))
            {

                if (grid.IsKnown(xy_index + offset))
                {
                    double x = grid.limits().max().x() - (offset.y() + xy_index.y() + 0.5) * grid.limits().resolution();
                    double y = grid.limits().max().y() - (offset.x() + xy_index.x() + 0.5) * grid.limits().resolution();

                    auto optimized = to_optimized * transform::Rigid3d::Translation(Eigen::Vector3d(x, y, 0.));

                    int x_map = (optimized.translation().x()) / grid.limits().resolution() + map_.map.info.width / 2;
                    int y_map = (optimized.translation().y()) / grid.limits().resolution() + map_.map.info.height / 2;

                    const double p = grid.GetProbability(xy_index + offset);
                    if (data[y_map * map_.map.info.width + x_map] == -1)
                    {
                        int map_state = 0;
                        if (p > 0.51)
                            map_state = p * 100;
                        else if (p < 0.49)
                            //map_state = -(1 - p) * 100;
                            map_state = 0;
                        data[y_map * map_.map.info.width + x_map] = map_state;
                    }
                }
            }
        }
        
    }

    void pcd()
    {
        ros::Rate r(1.0 / map_pub_period_);
        ::ros::NodeHandle node_handle;
        ::ros::Publisher pcd_publisher;
        pcd_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("carto_pcd", 1);

        while (ros::ok())
        {
            makeMap();
            auto markerArray = GetTrajectoryNodeList();
            auto constraintArray = GetConstraintList();
            node_list_publisher_.publish(markerArray);
            constraint_list_publisher_.publish(constraintArray);
            mapPublisher_.publish(map_.map);
            ros::spinOnce();
            r.sleep();
        }
    }

  private:
    std::shared_ptr<core::GlobalMapManager> global_map_builder_ptr_;
    double map_pub_period_;
    nav_msgs::GetMap::Response map_;
    ros::Publisher mapPublisher_;
    ros::Publisher node_list_publisher_;
    ros::Publisher constraint_list_publisher_;
    ros::NodeHandle node_;
};

} //top
} //sample_carto

#endif //SAMPLE_CARTO_TOP_MAPPUBLISHER_H_
