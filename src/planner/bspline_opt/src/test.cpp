#include <iostream>
#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

using namespace ego_planner;

void VisOptTrajs(const std::vector<std::vector<Eigen::Vector3d>> &vis_all_paths, ros::Publisher &vis_opt_trajs)
{
    visualization_msgs::MarkerArray delete_markers;
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.id = 0;
    delete_markers.markers.push_back(delete_marker);
    vis_opt_trajs.publish(delete_markers);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(2 * vis_all_paths.size());
    for (int i = 0; i < vis_all_paths.size(); i++)
    {
        // Set the marker properties for each path
        visualization_msgs::Marker &marker = marker_array.markers[i];
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
        marker.scale.x = 0.01;
        marker.scale.z = 0.01;
        marker.scale.y = 0.01;

        // Set the color for each path
        marker.color.r = (double)rand() / RAND_MAX;
        marker.color.g = (double)rand() / RAND_MAX;
        marker.color.b = (double)rand() / RAND_MAX;
        marker.color.a = 1.0;

        // Set the points for each path
        const std::vector<Eigen::Vector3d> &path_points = vis_all_paths[i];
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
        visualization_msgs::Marker &new_marker = marker_array.markers[vis_all_paths.size() + i];
        new_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        new_marker.id = vis_all_paths.size() + i;
        new_marker.scale.x = 0.2;
        new_marker.scale.y = 0.2;
        new_marker.scale.z = 0.2;
    }
    visualization_msgs::MarkerArray line_marker_array;
    auto res = Eigen::Vector3d(0.05, 0.05, 0.05);
    // VisMsg::pts_2dim_to_MarkerArray(vis_all_paths, line_marker_array, res, 4);

    while (1)
    {
        vis_opt_trajs.publish(marker_array);
    }
    // vis_opt_trajs.publish(marker_array);
    // vis_parallel_trajs_pub_.publish(line_marker_array);
}


    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
    // Eigen::MatrixXd controlpoints_of_traj;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");
    ros::Publisher vis_opt_trajs;
    ros::Time::init();

    vis_opt_trajs = nh.advertise<visualization_msgs::MarkerArray>("/vis_opt_trajs", 1);
    std::vector<std::vector<Eigen::Vector3d>> vis_all_paths;
    std::vector<Eigen::Vector3d> vis_path;
    Eigen::Vector3d a;

    std::cout << "test_bspline running" << std::endl;
    // init optimizer
    BsplineOptimizer::Ptr bspline_optimizer_;
    bspline_optimizer_.reset(new BsplineOptimizer);


    SwarmTrajData swarm_trajs_buf_; // std::vector<ego_planner::OneTrajDataOfSwarm>
    OneTrajDataOfSwarm one_traj1;
    OneTrajDataOfSwarm one_traj2;
    OneTrajDataOfSwarm one_traj3;
    Eigen::MatrixXd ctrl_pts_temp;
    double final_cost = 999999.0;
    double ts = 0.5;
    Eigen::MatrixXd traj_pts1, traj_pts2, traj_pts3;
    



    std::vector<Eigen::Vector3d> vectorList1 = {Eigen::Vector3d(10.65,2.05,0),Eigen::Vector3d(13.55,2.15,0.5),Eigen::Vector3d(16.05,2.15,1),Eigen::Vector3d(18.05,2.25,1.5),Eigen::Vector3d(19.65,2.15,2),Eigen::Vector3d(20.85,2.15,2.5),Eigen::Vector3d(21.65,2.25,3),Eigen::Vector3d(22.15,2.35,3.5),Eigen::Vector3d(22.35,2.45,4),Eigen::Vector3d(22.35,2.55,4.5),Eigen::Vector3d(22.25,2.65,5),};    
    // Eigen::MatrixXd traj_pts1(3,vectorList1.size());
    // for (int i = 0; i < vectorList1.size(); i++) {
    //     traj_pts1.col(i) = vectorList1[i];
    // }

    std::vector<Eigen::Vector3d>  start_end_derivatives;
    Eigen::Vector3d begin1(1.0,1.0,0),begin2(1.0,1.0,0),end1(0.0,0.0,0),end2(0.0,0.0,0);
    begin1 = (vectorList1[1] - vectorList1[0])*2;
    begin2 = (vectorList1[vectorList1.size()-1] - vectorList1[vectorList1.size()-2])*2;
    start_end_derivatives.push_back(begin1);
    start_end_derivatives.push_back(begin2);
    start_end_derivatives.push_back(end1);
    start_end_derivatives.push_back(end2);
    UniformBspline::parameterizeToBspline(ts, vectorList1, start_end_derivatives, traj_pts1);

    std::vector<Eigen::Vector3d> vectorList2 = {Eigen::Vector3d(10.65,2.05,0),Eigen::Vector3d(13.55,2.15,0.5),Eigen::Vector3d(16.15,2.15,1),Eigen::Vector3d(18.45,2.25,1.5),Eigen::Vector3d(20.75,2.15,2),Eigen::Vector3d(22.65,2.25,2.5),Eigen::Vector3d(24.25,2.55,3),Eigen::Vector3d(25.45,2.65,3.5),Eigen::Vector3d(26.35,2.85,4),Eigen::Vector3d(26.95,3.05,4.5),Eigen::Vector3d(27.45,3.35,5),};
    // Eigen::MatrixXd traj_pts2(3,vectorList2.size());
    // for (int i = 0; i < vectorList2.size(); i++) {
    //     traj_pts2.col(i) = vectorList2[i];
    // }
    begin1 = (vectorList2[1] - vectorList2[0])*2;
    begin2 = (vectorList2[vectorList2.size()-1] - vectorList2[vectorList2.size()-2])*2;
    start_end_derivatives[0] = begin1;
    start_end_derivatives[1] = begin2;
    UniformBspline::parameterizeToBspline(ts, vectorList2, start_end_derivatives, traj_pts2);

    std::vector<Eigen::Vector3d> vectorList3 = {Eigen::Vector3d(10.65,2.05,0),Eigen::Vector3d(13.75,2.15,0.5),Eigen::Vector3d(17.05,2.15,1),Eigen::Vector3d(20.75,2.25,1.5),Eigen::Vector3d(24.75,2.95,2),Eigen::Vector3d(28.85,4.75,2.5),Eigen::Vector3d(33.35,6.25,3),Eigen::Vector3d(38.45,5.35,3.5),Eigen::Vector3d(43.65,3.45,4),Eigen::Vector3d(49.25,2.65,4.5),Eigen::Vector3d(54.85,1.85,5),};
    // Eigen::MatrixXd traj_pts3(3,vectorList3.size());
    // for (int i = 0; i < vectorList3.size(); i++) {
    //     traj_pts3.col(i) = vectorList3[i];
    // }
    begin1 = (vectorList3[1] - vectorList3[0])*2;
    begin2 = (vectorList3[vectorList3.size()-1] - vectorList3[vectorList3.size()-2])*2;
    start_end_derivatives[0] = begin1;
    start_end_derivatives[1] = begin2;

    UniformBspline::parameterizeToBspline(ts, vectorList3, start_end_derivatives, traj_pts3);





    // Eigen::MatrixXd ctrl_pts_transpose(10, 3);
    // ctrl_pts_transpose << 10, 0, 0.5,
    //     12, 5, 1,
    //     14, 9, 1.5,
    //     16, 13, 2,
    //     18, 17, 2.5,
    //     20, 20, 3,
    //     22, 23, 3.5,
    //     24, 25, 4,
    //     26, 26, 4.5,
    //     28, 26, 5;

    // Eigen::MatrixXd traj_pts1 = ctrl_pts_transpose.transpose();

    // for (int i=0;i<ctrl_pts_transpose.rows();i++)
    // {
    //     a = ctrl_pts_transpose.row(i);
    //     vis_path.push_back(a);
    // }
    // vis_all_paths.push_back(vis_path);

    // Eigen::MatrixXd points(10, 3);
    // points << 10, 0, 0.5,
    //     10, 5, 1,
    //     10, 9, 1.5,
    //     10, 13, 2,
    //     10, 17, 2.5,
    //     10, 20, 3,
    //     10, 23, 3.5,
    //     10, 25, 4,
    //     10, 26, 4.5,
    //     10, 26, 5;
    // Eigen::MatrixXd traj_pts2 = points.transpose();

    // points << 10, 0, 0.5,
    // 12, 5, 1,
    // 14, 8, 1.5,
    // 16, 10, 2,
    // 18, 12, 2.5,
    // 20, 13, 3,
    // 22, 14, 3.5,
    // 24, 15, 4,
    // 26, 16, 4.5,
    // 28, 16, 5;
    // Eigen::MatrixXd traj_pts3 = points.transpose();

    std::cout << "traj_pts1:" << std::endl;
    std::cout << traj_pts1 << std::endl;
    std::cout << "traj_pts2:" << std::endl;
    std::cout << traj_pts2 << std::endl;
    std::cout << "traj_pts3:" << std::endl;
    std::cout << traj_pts3 << std::endl;

    ControlPoints traj;
    traj.points = traj_pts1;
    traj.size = traj_pts1.cols();


    int order = 3;
    double interval = 0.5;

    UniformBspline bspline1(traj_pts1, order, interval);
    one_traj1.position_traj_ = bspline1;
    one_traj1.probability_ = 0.5;
    one_traj1.drone_id = 0;
    swarm_trajs_buf_.push_back(one_traj1);

    UniformBspline bspline2(traj_pts2, order, interval);
    one_traj2.position_traj_ = bspline2;
    one_traj2.probability_ = 0.3;
    one_traj2.drone_id = 1;
    swarm_trajs_buf_.push_back(one_traj2);

    UniformBspline bspline3(traj_pts3, order, interval);
    one_traj3.position_traj_ = bspline3;
    one_traj3.probability_ = 0.2;
    one_traj3.drone_id = 2;
    swarm_trajs_buf_.push_back(one_traj3);



    bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_);
    bspline_optimizer_->setOrder(3);
    bspline_optimizer_->setBsplineInterval(0.5);

    // can be modified param 
    bspline_optimizer_->setDroneId(0);
    bspline_optimizer_->setControlPoints(traj_pts1);

    bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, traj, ts);
    std::cout << "final cost: " << final_cost << endl;
    std::cout<<"ctl pts:"<<endl;
    std::cout << traj_pts1<<endl;
    std::cout<<"result pts:"<<endl;
    std::cout << ctrl_pts_temp<<endl;

    vis_path.clear();
    for (int i=0;i<ctrl_pts_temp.cols();i++)
    {
        a = ctrl_pts_temp.col(i);
        vis_path.push_back(a);
    }
    vis_all_paths.push_back(vis_path);
    std::cout<<"vis size: "<<vis_path.size()<<endl;




    vis_all_paths.push_back(vectorList1);
    vis_all_paths.push_back(vectorList2);
    vis_all_paths.push_back(vectorList3);
    std::cout<<"vis size: "<<vis_all_paths.size()<<endl;
    VisOptTrajs(vis_all_paths, vis_opt_trajs);

    return 0;
}
