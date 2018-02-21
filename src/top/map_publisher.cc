#include "map_publisher.h"


namespace sample_carto
{

namespace top
{


    #define COLOR_CHANGE 10

    Publisher::Publisher(std::shared_ptr<core::GlobalMapManager> global_map_builder_ptr,
                       double map_pub_period, int node_num_per_submap) : global_map_builder_ptr_(global_map_builder_ptr),
                                                                         map_pub_period_(map_pub_period),
                                                                         node_num_per_submap_(node_num_per_submap),
                                                                         finish_(false)
    {
        mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        node_list_publisher_ = node_.advertise<::visualization_msgs::MarkerArray>("node_list", 1);
        constraint_list_publisher_ = node_.advertise<::visualization_msgs::MarkerArray>("constraints", 1);
    };
    void Publisher::pcd()
    {
        ros::Rate r(1.0 / map_pub_period_);
        ::ros::NodeHandle node_handle;
        ::ros::Publisher pcd_publisher;
        pcd_publisher = node_handle.advertise<sensor_msgs::PointCloud2>("carto_pcd", 1);

        while (ros::ok() && !finish_ )
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

    void Publisher::finish()
    {
        finish_ = true;
    }

    geometry_msgs::Point Publisher::ToGeometryMsgPoint(const Eigen::Vector3d &vector3d)
    {
        geometry_msgs::Point point;
        point.x = vector3d.x();
        point.y = vector3d.y();
        point.z = vector3d.z();
        return point;
    }

    std_msgs::ColorRGBA Publisher::colorBar(double xbrt)
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
    void Publisher::CreateCVSubmapData(int submap_id)
    {
        const auto all_submap_data = global_map_builder_ptr_->sparse_pose_graph()->GetAllSubmapData();
        if (cv_submaps_.count(submap_id) != 0)
            return;

        core::SparsePoseGraph::SubmapDataWithPose m = all_submap_data[submap_id];
        Eigen::Array2i offset;
        core::map::CellLimits limits;
        const auto &grid = m.submap->probability_grid();
        grid.ComputeCroppedLimits(&offset, &limits);
        cv::Mat cv_submap(grid.limits().cell_limits().num_y_cells, grid.limits().cell_limits().num_x_cells, CV_8UC3, cv::Scalar(200, 200, 200));
        for (const Eigen::Array2i &xy_index : core::map::XYIndexRangeIterator(limits))
        {
            Eigen::Array2i local = xy_index + offset;
            if (grid.IsKnown(local))
            {
                const double p = grid.GetProbability(xy_index + offset);
                int map_state = 0;
                if (p > 0.51)
                    map_state = (1 - p) * 200;
                else if (p < 0.49)
                    map_state = 255;

                cv_submap.at<cv::Vec3b>(local.y(), local.x()) = cv::Vec3b(map_state, map_state, map_state);
            }
        }
        cv_submaps_[submap_id] = cv_submap;
        return;
    }

    void Publisher::SaveContraintImage(int submap_id,int node_id, sample_carto::core::Node node, const sample_carto::transform::Rigid3d trans)
    {
        cv::Mat cv_submap = cv_submaps_[submap_id].clone();
        sample_carto::sensor::PointCloud points = node.constant_data->filtered_gravity_aligned_point_cloud;
        const auto all_submap_data = global_map_builder_ptr_->sparse_pose_graph()->GetAllSubmapData();

        core::SparsePoseGraph::SubmapDataWithPose m = all_submap_data[submap_id];
        //Eigen::Array2i offset;
        //core::map::CellLimits limits;
        const auto &grid = m.submap->probability_grid();

        cv::Rect rect(cv::Point(), cv_submap.size());

        //grid.ComputeCroppedLimits(&offset, &limits);
        for (auto p : points)
        {
            const Eigen::Vector3d point = trans * p.cast<double>();
            Eigen::Array2i map_xy = grid.limits().GetCellIndex(point.head<2>().cast<float>());
            if (rect.contains(cv::Point(map_xy.x(), map_xy.y())))
            {
                cv_submap.at<cv::Vec3b>(map_xy.y(), map_xy.x()) = cv::Vec3b(0, 255, 0);
            }
        }
        std::stringstream name;
        name << "constraints_" << submap_id << "_to_" << node_id << ".png";
        cv::imwrite(name.str(), cv_submap);

        return;
    }

    visualization_msgs::MarkerArray Publisher::GetConstraintList()
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
                name << "constraints_" << constraint.submap_id << "_to_" << constraint.node_id;
                //std::cout << name.str() << "\n";
                marker.ns = name.str();
                marker.id = (constraint.submap_id << 16) + constraint.node_id;
                marker.colors.push_back(colorBar(((constraint.submap_id) % COLOR_CHANGE) / (double)COLOR_CHANGE));                      // = colorBar(0.9);
                marker.colors.push_back(colorBar(((constraint.node_id / node_num_per_submap_) % COLOR_CHANGE) / (double)COLOR_CHANGE)); // = colorBar(0.9);
                const auto &submap = submaps[constraint.submap_id];
                const auto &submap_pose = submap.pose;
                const auto &node_pose = nodes.at(constraint.node_id).pose;
                const sample_carto::transform::Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

                marker.points.push_back(ToGeometryMsgPoint(constraint_pose.translation()));
                marker.points.push_back(ToGeometryMsgPoint(node_pose.translation()));

                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.action = visualization_msgs::Marker::ADD;

                markers_array.markers.push_back(marker);
                #ifdef OPENCV_DEBUG
                CreateCVSubmapData(constraint.submap_id);
                SaveContraintImage(constraint.submap_id,constraint.node_id, nodes.at(constraint.node_id), submap.submap->local_pose() * constraint.pose.zbar_ij);
                #endif
            }
        }

        return markers_array;
    }

    visualization_msgs::MarkerArray Publisher::GetTrajectoryNodeList()
    {
        const auto nodes = global_map_builder_ptr_->sparse_pose_graph()->GetNodes();
        visualization_msgs::MarkerArray markers_array;
        for (const auto &node_id_data : nodes)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            if (node_id_data.first % node_num_per_submap_ == 0)
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
            marker.color = colorBar(((node_id_data.first / node_num_per_submap_) % COLOR_CHANGE) / (double)COLOR_CHANGE);
            marker.points.push_back(node_point);

            marker.pose.position.x = node_point.x;
            marker.pose.position.y = node_point.y;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            markers_array.markers.push_back(marker);
        }

        return markers_array;
    };

    void Publisher::makeMap()
    {
        double resolution = 0.05;

        const auto all_submap_data = global_map_builder_ptr_->sparse_pose_graph()->GetAllSubmapData();
        if (all_submap_data.size() == 0)
            return;

        std::vector<std::pair<Eigen::Array2i,double>> known_points;
        Eigen::AlignedBox2i known_cells_box;

        for (core::SparsePoseGraph::SubmapDataWithPose m : all_submap_data)
        {
            Eigen::Array2i offset;
            core::map::CellLimits limits;
            const auto &grid = m.submap->probability_grid();
            grid.ComputeCroppedLimits(&offset, &limits);
            auto local_to_global = m.pose * m.submap->local_pose().inverse();

            for (const Eigen::Array2i &xy_index : core::map::XYIndexRangeIterator(limits))
            {

                if (grid.IsKnown(xy_index + offset))
                {
                    double x = grid.limits().max().x() - (offset.y() + xy_index.y() + 0.5) * grid.limits().resolution();
                    double y = grid.limits().max().y() - (offset.x() + xy_index.x() + 0.5) * grid.limits().resolution();
                    auto world_pose = local_to_global * transform::Rigid3d::Translation(Eigen::Vector3d(x, y, 0.));
                    Eigen::Array2i map_pose = Eigen::Array2i(world_pose.translation().x() / resolution, world_pose.translation().y() / resolution);
                    known_points.emplace_back(map_pose, grid.GetProbability(xy_index + offset));
                    known_cells_box.extend(map_pose.matrix());
                }
            }
        }

        std::vector<int8_t> &data = map_.map.data;
        map_.map.info.origin.position.x = known_cells_box.min().x() * 0.05;
        map_.map.info.origin.position.y = known_cells_box.min().y() * 0.05;
        map_.map.info.origin.orientation.w = 1;
        map_.map.info.resolution = resolution;
        map_.map.info.width = known_cells_box.sizes().x() + 1;
        map_.map.info.height = known_cells_box.sizes().y() + 1;

        map_.map.header.frame_id = "map";
        map_.map.data.resize(map_.map.info.width * map_.map.info.height);
        map_.map.header.stamp = ros::Time::now();
        memset(&map_.map.data[0], -1, sizeof(int8_t) * map_.map.data.size());
        for (const auto &known_point : known_points)
        {

            Eigen::Array2i map_point = known_point.first - known_cells_box.min().array();

            if (data[map_point.y() * map_.map.info.width + map_point.x()] <= 0)
            {
                int map_state = -1;
                if (known_point.second > 0.51)
                    map_state = known_point.second * 100;
                else if (known_point.second < 0.49)
                    map_state = 0;
                data[map_point.y() * map_.map.info.width + map_point.x()] = map_state;
            }
        }
    }


} //top
} //sample_carto


