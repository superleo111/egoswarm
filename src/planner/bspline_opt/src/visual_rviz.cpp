#include "ros/ros.h"
#include <iostream>
#include <Eigen/Eigen>
#include "visualization_msgs/MarkerArray.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");
    ros::Publisher  vis_opt_trajs;

    vis_opt_trajs = nh.advertise<visualization_msgs::MarkerArray>("/vis_opt_trajs", 1);
    std::vector<std::vector<Eigen::Vector3f>> vis_all_paths;
    
    VisOptTrajs(vis_all_paths, vis_opt_trajs);

}



void VisOptTrajs(const std::vector<std::vector<Eigen::Vector3f>>& vis_all_paths, ros::Publisher& vis_opt_trajs)
{
    visualization_msgs::MarkerArray delete_markers;
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.id = 0;
    delete_markers.markers.push_back(delete_marker);
    vis_opt_trajs.publish(delete_markers);
    
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(2*vis_all_paths.size());
    for (int i = 0; i < vis_all_paths.size(); i++)
    {
        // Set the marker properties for each path
        visualization_msgs::Marker& marker = marker_array.markers[i];
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "path_marker_array";
        marker.id = i;
        // marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.z = 0.1;
        marker.scale.y = 0.1;

        // Set the color for each path
        marker.color.r = (double) rand() / RAND_MAX;
        marker.color.g = (double) rand() / RAND_MAX;
        marker.color.b = (double) rand() / RAND_MAX;
        marker.color.a = 1.0;

        // Set the points for each path
        const std::vector<Eigen::Vector3f>& path_points = vis_all_paths[i];
        marker.points.resize(path_points.size());
        for (int j = 0; j < path_points.size(); j++)
        {
            geometry_msgs::Point point;
            point.x = path_points[j](0);
            point.y = path_points[j](1);
            point.z = path_points[j](2);
            marker.points[j] = point;
        }

        marker_array.markers[vis_all_paths.size() + i] = marker;
        visualization_msgs::Marker& new_marker = marker_array.markers[vis_all_paths.size() + i];
        new_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        new_marker.id = vis_all_paths.size() + i;
        new_marker.scale.x = 0.2;
        new_marker.scale.y = 0.2;
        new_marker.scale.z = 0.2;
        
    }
    visualization_msgs::MarkerArray line_marker_array;
    auto res =  Eigen::Vector3d(0.05,0.05,0.05);
    // VisMsg::pts_2dim_to_MarkerArray(vis_all_paths, line_marker_array, res, 4);

    vis_opt_trajs.publish(marker_array);
    // vis_parallel_trajs_pub_.publish(line_marker_array);

}