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
                           p_global_trajectory_builder,
                       double map_pub_period) : p_global_trajectory_builder_(p_global_trajectory_builder), map_pub_period_(map_pub_period)
    {
        mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        trajectory_node_list_publisher_ = node_.advertise<::visualization_msgs::MarkerArray>("node_list", 1);
        constraint_list_publisher_ = node_.advertise<::visualization_msgs::MarkerArray>("constraints", 1);
    };

    void PushAndResetLineMarker(visualization_msgs::Marker *marker,
                                std::vector<visualization_msgs::Marker> *markers)
    {
        if (marker->points.size() > 1)
        {
            markers->push_back(*marker);
            ++marker->id;
        }
        marker->points.clear();
    }

    visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                      const std::string &frame_id)
    {
        visualization_msgs::Marker marker;
        marker.ns = "Trajectory " + std::to_string(trajectory_id);
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.header.stamp = ::ros::Time::now();
        marker.header.frame_id = frame_id;
        ::std_msgs::ColorRGBA result;
        result.r = 0.3f;
        result.g = 0.f;
        result.b = 1.f;
        result.a = 1.f;

        marker.color = result;
        marker.scale.x = 0.1;
        marker.pose.orientation.w = 1.;
        marker.pose.position.z = 0.05;
        return marker;
    }
    geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d &vector3d)
    {
        geometry_msgs::Point point;
        point.x = vector3d.x();
        point.y = vector3d.y();
        point.z = vector3d.z();
        return point;
    }
/*
    visualization_msgs::MarkerArray GetConstraintList()
    {
        visualization_msgs::MarkerArray constraint_list;
        int marker_id = 0;
        visualization_msgs::Marker constraint_intra_marker;
        constraint_intra_marker.id = marker_id++;
        constraint_intra_marker.ns = "Intra constraints";
        constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
        constraint_intra_marker.header.stamp = ros::Time::now();
        constraint_intra_marker.header.frame_id = "map";
        constraint_intra_marker.scale.x = 0.03;
        constraint_intra_marker.pose.orientation.w = 1.0;

        visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
        residual_intra_marker.id = marker_id++;
        residual_intra_marker.ns = "Intra residuals";
        // This and other markers which are less numerous are set to be slightly
        // above the intra constraints marker in order to ensure that they are
        // visible.
        residual_intra_marker.pose.position.z = 0.1;

        visualization_msgs::Marker constraint_inter_marker = constraint_intra_marker;
        constraint_inter_marker.id = marker_id++;
        constraint_inter_marker.ns = "Inter constraints";
        constraint_inter_marker.pose.position.z = 0.1;

        visualization_msgs::Marker residual_inter_marker = constraint_intra_marker;
        residual_inter_marker.id = marker_id++;
        residual_inter_marker.ns = "Inter residuals";
        residual_inter_marker.pose.position.z = 0.1;

        const auto all_trajectory_nodes = p_global_trajectory_builder_->sparse_pose_graph()->GetTrajectoryNodes();
        const auto all_submap_data = p_global_trajectory_builder_ -> sparse_pose_graph()->GetAllSubmapData();
        const auto constraints = p_global_trajectory_builder_ -> sparse_pose_graph()->constraints();

        for (const auto &constraint : constraints)
        {
            visualization_msgs::Marker *constraint_marker, *residual_marker;
            std_msgs::ColorRGBA color_constraint, color_residual;
            if (constraint.tag == cartographer::mapping::SparsePoseGraph::Constraint::INTRA_SUBMAP)
            {
                constraint_marker = &constraint_intra_marker;
                residual_marker = &residual_intra_marker;
                // Color mapping for submaps of various trajectories - add trajectory id
                // to ensure different starting colors. Also add a fixed offset of 25
                // to avoid having identical colors as trajectories.

                color_constraint.r = 1.f;
                color_constraint.g = ( (constraint.submap_id.submap_index+3) % 5) /5.f;
                color_constraint.b = ( (constraint.submap_id.submap_index+0) % 5) /5.f;
                color_constraint.a = ( (constraint.submap_id.submap_index+0) % 5) /5.f;

                color_residual.a = 1.0;
                color_residual.r = 1.0;
            }
            else
            {
                constraint_marker = &constraint_inter_marker;
                residual_marker = &residual_inter_marker;
                // Bright yellow
                color_constraint.a = 1.0;
                color_constraint.r = color_constraint.g = 1.0;
                // Bright cyan
                color_residual.a = 1.0;
                color_residual.b = color_residual.g = 1.0;
            }

            for (int i = 0; i < 2; ++i)
            {
                //constraint_marker->colors.push_back(color_constraint);
                residual_marker->colors.push_back(color_residual);
            }

            const auto &submap_data =
                all_submap_data[constraint.submap_id.trajectory_id]
                               [constraint.submap_id.submap_index];
            const auto &submap_pose = submap_data.pose;
            const auto &trajectory_node_pose =
                all_trajectory_nodes[constraint.node_id.trajectory_id]
                                    [constraint.node_id.node_index]
                                        .pose;
            const cartographer::transform::Rigid3d constraint_pose =
                submap_pose * constraint.pose.zbar_ij;
//
            //constraint_marker->points.push_back(
            //    ToGeometryMsgPoint(submap_pose.translation()));
            //constraint_marker->points.push_back(
            //    ToGeometryMsgPoint(constraint_pose.translation()));

            residual_marker->points.push_back(
                ToGeometryMsgPoint(constraint_pose.translation()));
            residual_marker->points.push_back(
                ToGeometryMsgPoint(trajectory_node_pose.translation()));
        }

        constraint_list.markers.push_back(constraint_intra_marker);
        constraint_list.markers.push_back(residual_intra_marker);
        constraint_list.markers.push_back(constraint_inter_marker);
        constraint_list.markers.push_back(residual_inter_marker);
        return constraint_list;
    }

    visualization_msgs::MarkerArray GetTrajectoryNodeList()
    {
        visualization_msgs::MarkerArray trajectory_node_list;
        const auto nodes = p_global_trajectory_builder_->sparse_pose_graph()->GetTrajectoryNodes();
        visualization_msgs::Marker marker = CreateTrajectoryMarker(0, "map");
        if (nodes.size() == 0)
            return trajectory_node_list;
        for (const auto &node_id_data : nodes[0])
        {
            if (node_id_data.constant_data == nullptr)
            {
                PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
                continue;
            }
            const ::geometry_msgs::Point node_point =
                ToGeometryMsgPoint(node_id_data.pose.translation());
            marker.points.push_back(node_point);
            // Work around the 16384 point limit in RViz by splitting the
            // trajectory into multiple markers.
            if (marker.points.size() == 16384)
            {
                PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
                // Push back the last point, so the two markers appear connected.
                marker.points.push_back(node_point);
            }
        }
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        return trajectory_node_list;
    };
    */

    void makeMap()
    {
        
        const auto all_submap_data = p_global_trajectory_builder_->sparse_pose_graph()->GetAllSubmapData();
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
            //auto markerArray = GetTrajectoryNodeList();
            //auto constraintArray = GetConstraintList();
            //trajectory_node_list_publisher_.publish(markerArray);
            //constraint_list_publisher_.publish(constraintArray);
            mapPublisher_.publish(map_.map);
            ros::spinOnce();
            r.sleep();
        }
    }

  private:
    std::shared_ptr<core::GlobalMapManager> p_global_trajectory_builder_;
    double map_pub_period_;
    nav_msgs::GetMap::Response map_;
    ros::Publisher mapPublisher_;
    ros::Publisher trajectory_node_list_publisher_;
    ros::Publisher constraint_list_publisher_;
    ros::NodeHandle node_;
};

} //top
} //sample_carto

#endif //SAMPLE_CARTO_TOP_MAPPUBLISHER_H_
