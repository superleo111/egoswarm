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
        // marker.color.r = (double)rand() / RAND_MAX;
        // marker.color.g = (double)rand() / RAND_MAX;
        // marker.color.b = (double)rand() / RAND_MAX;
        marker.color.a = 0.4;
        if (i>2){marker.color.a=1.0;}

        if (i==3 || i ==0) {        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;}
        if (i==4 || i ==1) {        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;}    
        if (i==5 || i ==2 ) {        marker.color.r = 1;
        marker.color.g = 0.6;
        marker.color.b = 0.2;}      

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
        new_marker.scale.x = 0.1;
        new_marker.scale.y = 0.1;
        new_marker.scale.z = 0.1;
        new_marker.color.a = 0.5;
        if (i>2){new_marker.color.a = 1;}
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



// [ERROR FIXME error is ANNOTATION in swarm uniform bspline file, is 3*n (cols num is the pts num)]
    // e.g. B-spline with N points in 3D space -> Nx3 matrix 



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

    SwarmTrajData swarm_trajs_buf_;
    OneTrajDataOfSwarm one_traj1;
    OneTrajDataOfSwarm one_traj2;
    OneTrajDataOfSwarm one_traj3;
    Eigen::MatrixXd ctrl_pts_temp;
    double final_cost = 999999.0;
    double ts = 0.5;
    // bspine param
    int order = 3;
    double interval = 0.5;
    // all traj input transform into these three Matrix 
    Eigen::MatrixXd traj_pts1, traj_pts2, traj_pts3;
    


// [TEST INSTANCE]
// below is 3 trajs for swarmcost test 
// and transform vectorlist into Bspline(traj_pts1 2 3)
    // std::vector<Eigen::Vector3d> vectorList1 = {Eigen::Vector3d(10.65,2.05,0),Eigen::Vector3d(13.55,2.15,0.5),Eigen::Vector3d(16.05,2.15,1),Eigen::Vector3d(18.05,2.25,1.5),Eigen::Vector3d(19.65,2.15,2),Eigen::Vector3d(20.85,2.15,2.5),Eigen::Vector3d(21.65,2.25,3),Eigen::Vector3d(22.15,2.35,3.5),Eigen::Vector3d(22.35,2.45,4),Eigen::Vector3d(22.35,2.55,4.5),Eigen::Vector3d(22.25,2.65,5),};    

    // std::vector<Eigen::Vector3d>  start_end_derivatives;
    // Eigen::Vector3d begin1(1.0,1.0,0),begin2(1.0,1.0,0),end1(0.0,0.0,0),end2(0.0,0.0,0);
    // begin1 = (vectorList1[1] - vectorList1[0])*2;
    // begin2 = (vectorList1[vectorList1.size()-1] - vectorList1[vectorList1.size()-2])*2;
    // start_end_derivatives.push_back(begin1);
    // start_end_derivatives.push_back(begin2);
    // start_end_derivatives.push_back(end1);
    // start_end_derivatives.push_back(end2);
    // UniformBspline::parameterizeToBspline(ts, vectorList1, start_end_derivatives, traj_pts1);

    // std::vector<Eigen::Vector3d> vectorList2 = {Eigen::Vector3d(10.65,2.05,0),Eigen::Vector3d(13.55,2.15,0.5),Eigen::Vector3d(16.15,2.15,1),Eigen::Vector3d(18.45,2.25,1.5),Eigen::Vector3d(20.75,2.15,2),Eigen::Vector3d(22.65,2.25,2.5),Eigen::Vector3d(24.25,2.55,3),Eigen::Vector3d(25.45,2.65,3.5),Eigen::Vector3d(26.35,2.85,4),Eigen::Vector3d(26.95,3.05,4.5),Eigen::Vector3d(27.45,3.35,5),};
 
    // begin1 = (vectorList2[1] - vectorList2[0])*2;
    // begin2 = (vectorList2[vectorList2.size()-1] - vectorList2[vectorList2.size()-2])*2;
    // start_end_derivatives[0] = begin1;
    // start_end_derivatives[1] = begin2;
    // UniformBspline::parameterizeToBspline(ts, vectorList2, start_end_derivatives, traj_pts2);

    // std::vector<Eigen::Vector3d> vectorList3 = {Eigen::Vector3d(10.65,2.05,0),Eigen::Vector3d(13.75,2.15,0.5),Eigen::Vector3d(17.05,2.15,1),Eigen::Vector3d(20.75,2.25,1.5),Eigen::Vector3d(24.75,2.95,2),Eigen::Vector3d(28.85,4.75,2.5),Eigen::Vector3d(33.35,6.25,3),Eigen::Vector3d(38.45,5.35,3.5),Eigen::Vector3d(43.65,3.45,4),Eigen::Vector3d(49.25,2.65,4.5),Eigen::Vector3d(54.85,1.85,5),};

    // begin1 = (vectorList3[1] - vectorList3[0])*2;
    // begin2 = (vectorList3[vectorList3.size()-1] - vectorList3[vectorList3.size()-2])*2;
    // start_end_derivatives[0] = begin1;
    // start_end_derivatives[1] = begin2;
    // UniformBspline::parameterizeToBspline(ts, vectorList3, start_end_derivatives, traj_pts3);



// [TEST INSTANCE]
// below is the trajs point set manual(for swarm test)
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
    // traj_pts1 = ctrl_pts_transpose.transpose();

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
    // traj_pts2 = points.transpose();

    // points << 10, 0, 0.5,
    // 13, 5, 1,
    // 15, 8, 1.5,
    // 16, 10, 2,
    // 18, 12, 2.5,
    // 20, 13, 3,
    // 22, 14, 3.5,
    // 24, 15, 4,
    // 26, 16, 4.5,
    // 28, 16, 5;
    // traj_pts3 = points.transpose();


//print for ctl pts check
    // std::cout << "traj_pts1:" << std::endl;
    // std::cout << traj_pts1 << std::endl;
    // std::cout << "traj_pts2:" << std::endl;
    // std::cout << traj_pts2 << std::endl;
    // std::cout << "traj_pts3:" << std::endl;
    // std::cout << traj_pts3 << std::endl;




// Matrix traj_pts to onetraj to swartrajs finally
    // UniformBspline bspline1(traj_pts1, order, interval);
    // one_traj1.position_traj_ = bspline1;
    // one_traj1.probability_ = 0.3;
    // one_traj1.drone_id = 0;
    // swarm_trajs_buf_.push_back(one_traj1);

    // UniformBspline bspline2(traj_pts2, order, interval);
    // one_traj2.position_traj_ = bspline2;
    // one_traj2.probability_ = 0.3;
    // one_traj2.drone_id = 1;
    // swarm_trajs_buf_.push_back(one_traj2);

    // UniformBspline bspline3(traj_pts3, order, interval);
    // one_traj3.position_traj_ = bspline3;
    // one_traj3.probability_ = 1.0;
    // one_traj3.drone_id = 2;
    // swarm_trajs_buf_.push_back(one_traj3);




// [VISUALIZE] matrix ctl pts 
    // vis_path.clear();
    // for (int i=0;i<traj_pts1.cols();i++)
    // {
    //     a = traj_pts1.col(i);
    //     vis_path.push_back(a);
    // }
    // vis_all_paths.push_back(vis_path);
    // std::cout<<"vis size: "<<vis_path.size()<<endl;

    // vis_path.clear();
    // for (int i=0;i<traj_pts2.cols();i++)
    // {
    //     a = traj_pts2.col(i);
    //     vis_path.push_back(a);
    // }
    // vis_all_paths.push_back(vis_path);
    // std::cout<<"vis size: "<<vis_path.size()<<endl;

    // vis_path.clear();
    // for (int i=0;i<traj_pts3.cols();i++)
    // {
    //     a = traj_pts3.col(i);
    //     vis_path.push_back(a);
    // }
    // vis_all_paths.push_back(vis_path);
    // std::cout<<"vis size: "<<vis_path.size()<<endl;



// optimization cycle(for swarm)
    // std::vector<Eigen::MatrixXd> trajs;
    // trajs.push_back(traj_pts1);
    // trajs.push_back(traj_pts2);
    // trajs.push_back(traj_pts3);

    // for (int i=0;i<3;i++)
    // {
    //     auto time0 = ros::Time::now();
    //     BsplineOptimizer::Ptr bspline_optimizer_;
    //     bspline_optimizer_.reset(new BsplineOptimizer);

    //     bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_);
    //     bspline_optimizer_->setOrder(3);
    //     bspline_optimizer_->setBsplineInterval(0.5);


    //     bspline_optimizer_->setDroneId(i);
    //     bspline_optimizer_->setControlPoints(trajs[i]);
    //     ControlPoints traj;
    //     traj.points = trajs[i];
    //     traj.size = trajs[i].cols();

    //     bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, traj, ts);
    //     std::cout << "[opt time cost]---------" << (ros::Time::now() -time0 )*1000 <<"ms"<< std::endl;

    //     std::cout<<i<<" traj-------[opt end]"<<endl;
    //     std::cout << "final cost: " << final_cost << endl;
    //     std::cout<<"ctl pts:"<<endl;
    //     // FIME remember modify this output to compare
    //     std::cout << traj_pts1<<endl;
    //     std::cout<<"result pts:"<<endl;
    //     std::cout << ctrl_pts_temp<<endl;
        
    //     std::vector<Eigen::Vector3d> opt_path;
    //     for (int i=0;i<ctrl_pts_temp.cols();i++)
    //     {
    //         a = ctrl_pts_temp.col(i);
    //         opt_path.push_back(a);
    //     }
    //     vis_all_paths.push_back(opt_path);
    //     std::cout<<"vis size: "<<opt_path.size()<<endl;

    // }





// [TEST INSTANCE] 2 trajs for feasible and smooth test
    // std::vector<Eigen::Vector3d> exec_traj_pts_xyt = {Eigen::Vector3d(5.05,100.05,0),Eigen::Vector3d(12.25,100.15,0.5),Eigen::Vector3d(19.05,100.35,1),Eigen::Vector3d(26.15,100.55,1.5),Eigen::Vector3d(32.95,100.55,2),Eigen::Vector3d(40.05,100.45,2.5),Eigen::Vector3d(48.25,100.35,3),Eigen::Vector3d(55.35,100.25,3.5),Eigen::Vector3d(63.65,100.05,4),Eigen::Vector3d(70.75,100.25,4.5),Eigen::Vector3d(78.85,100.45,5),};
    // std::vector<Eigen::Vector3d> tva_pts = {Eigen::Vector3d(0,14.1153,0),Eigen::Vector3d(0.5,13.6176,-1.11803),Eigen::Vector3d(1,13.6245,0.5),Eigen::Vector3d(1.5,13.6153,1),Eigen::Vector3d(2,13.6176,0.5),Eigen::Vector3d(2.5,13.8743,0.707107),Eigen::Vector3d(3,14.1175,0.707107),Eigen::Vector3d(3.5,14.374,0.707107),Eigen::Vector3d(4,14.3675,1.5),Eigen::Vector3d(4.5,14.1241,-0.707107),Eigen::Vector3d(5,13.8675,-0.707107),};
    // std::vector<Eigen::Vector3d> xyyaw_pts = {Eigen::Vector3d(5.05,100.05,0.00164008),Eigen::Vector3d(12.25,100.15,0.0220119),Eigen::Vector3d(19.05,100.35,0.0407764),Eigen::Vector3d(26.15,100.55,0.00477979),Eigen::Vector3d(32.95,100.55,-0.0135781),Eigen::Vector3d(40.05,100.45,-0.0287989),Eigen::Vector3d(48.25,100.35,-0.00812269),Eigen::Vector3d(55.35,100.25,-0.0267066),Eigen::Vector3d(63.65,100.05,0.0221366),Eigen::Vector3d(70.75,100.25,0.0368961),Eigen::Vector3d(78.85,100.45,0.0160921),};

    std::vector<Eigen::Vector3d> exec_traj_pts_xyt = {Eigen::Vector3d(12.55,21.55,0),Eigen::Vector3d(12.95,21.55,0.5),Eigen::Vector3d(13.35,21.55,1),Eigen::Vector3d(13.75,21.55,1.5),Eigen::Vector3d(14.15,21.65,2),Eigen::Vector3d(14.75,21.75,2.5),Eigen::Vector3d(15.45,21.85,3),Eigen::Vector3d(16.25,21.85,3.5),Eigen::Vector3d(16.95,21.85,4),Eigen::Vector3d(18.15,22.25,4.5),Eigen::Vector3d(19.55,22.95,5),};
    std::vector<Eigen::Vector3d> tva_pts = {Eigen::Vector3d(0,0.833753,0.233051),Eigen::Vector3d(0.5,0.750062,-0.167394),Eigen::Vector3d(1,0.750062,0),Eigen::Vector3d(1.5,0.750062,0),Eigen::Vector3d(2,1.02847,0.707107),Eigen::Vector3d(2.5,1.25004,0.707107),Eigen::Vector3d(3,1.50003,0.5),Eigen::Vector3d(3.5,1.52231,0.5),Eigen::Vector3d(4,2.25002,1.11803),Eigen::Vector3d(4.5,2.79337,1.41421),Eigen::Vector3d(5,3.28678,1),};
    std::vector<Eigen::Vector3d> xyyaw_pts = {Eigen::Vector3d(12.55,21.55,0.047979),Eigen::Vector3d(12.95,21.55,0.0466844),Eigen::Vector3d(13.35,21.55,0.0818546),Eigen::Vector3d(13.75,21.55,0.0818546),Eigen::Vector3d(14.15,21.65,0.330605),Eigen::Vector3d(14.75,21.75,0.14156),Eigen::Vector3d(15.45,21.85,0.142849),Eigen::Vector3d(16.25,21.85,0.0497559),Eigen::Vector3d(16.95,21.85,0.216873),Eigen::Vector3d(18.15,22.25,0.473461),Eigen::Vector3d(19.55,22.95,0.52353),};
    // vector list to bspline
    std::vector<Eigen::Vector3d> start_end_derivative;
    double start_v_norm = tva_pts[0][1];
    Eigen::Vector3d start_v(start_v_norm * cos(xyyaw_pts[0][1]), start_v_norm * sin(xyyaw_pts[0][2]), 1.0);
    start_end_derivative.emplace_back(start_v);
    double end_v_norm = tva_pts[tva_pts.size()-1][1];
    Eigen::Vector3d end_v(end_v_norm * cos(xyyaw_pts[xyyaw_pts.size()-1][2]), end_v_norm * sin(xyyaw_pts[xyyaw_pts.size()-1][2]), 1.0);
    start_end_derivative.emplace_back(end_v);
    double start_a_norm = tva_pts[0][2];
    Eigen::Vector3d start_a(start_a_norm * cos(xyyaw_pts[0][2]), start_a_norm * sin(xyyaw_pts[0][2]), 0.0);
    start_end_derivative.emplace_back(start_a);
    double end_a_norm = (tva_pts[tva_pts.size()-1][2] + tva_pts[tva_pts.size()-2][2]) / 2;
    Eigen::Vector3d end_a(end_a_norm * cos(xyyaw_pts[xyyaw_pts.size()-1][2]), end_a_norm * sin(xyyaw_pts[xyyaw_pts.size()-1][2]), 0.0);
    start_end_derivative.emplace_back(end_a);

    UniformBspline::parameterizeToBspline(0.5, exec_traj_pts_xyt, start_end_derivative, traj_pts1);
    // position vector to v/acc spline
    UniformBspline exec_traj_Bspline(traj_pts1, 3, 0.5);
    auto v_spline = exec_traj_Bspline.getDerivative();
    auto acc_spline = v_spline.getDerivative();

// extract speed and acc vector elements and print
    std::vector<geometry_msgs::Vector3> ego_planning_trajectory_xyyaw;
    std::vector<geometry_msgs::Vector3> ego_planning_trajectory_tva;
    for(double t = 0.0; t < 5.1; t += 0.5)
    {
        geometry_msgs::Vector3 xyyaw;
        geometry_msgs::Vector3 tva;
        auto pt = exec_traj_Bspline.evaluateDeBoor(t);
        auto speed = v_spline.evaluateDeBoor(t);
        auto acc = acc_spline.evaluateDeBoor(t);
        float yaw = atan2(speed[1], speed[0]);
        xyyaw.x = pt[0];
        xyyaw.y = pt[1];
        xyyaw.z = yaw;
        tva.x = t;
        tva.y = sqrt(speed[0]*speed[0] + speed[1]*speed[1]);
        // dot_product of Eigen::VectorXd speed and acc
        float dot_product = speed[0]*acc[0] + speed[1]*acc[1];
        tva.z = acc.norm() * (dot_product > 0 ? 1 : -1);
        ego_planning_trajectory_xyyaw.emplace_back(xyyaw);
        ego_planning_trajectory_tva.emplace_back(tva);
    }
    // print speed and acc vector
    std::ostringstream v_list_oss, a_list_oss;
    v_list_oss << "v_list: ";
    a_list_oss << "a_list: ";
    for(int i=0; i<ego_planning_trajectory_tva.size();i++)
    {
        a_list_oss << std::fixed << std::setprecision(1) << ego_planning_trajectory_tva[i].z << ", ";
        v_list_oss << std::fixed << std::setprecision(1) << ego_planning_trajectory_tva[i].y << ", ";
    }
    std::cout << "Bspline:" << std::endl << v_list_oss.str() << std::endl << a_list_oss.str() << std::endl;

// single traj optimize(for traj_pts1)
    BsplineOptimizer::Ptr bspline_optimizer_;
    bspline_optimizer_.reset(new BsplineOptimizer);
    // FIXME can be modified param 
    bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_);
    bspline_optimizer_->setOrder(3);
    bspline_optimizer_->setBsplineInterval(0.5);
    bspline_optimizer_->setDroneId(0);
    bspline_optimizer_->setControlPoints(traj_pts1);
    ControlPoints traj;
    traj.points = traj_pts1;
    traj.size = traj_pts1.cols();
    bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, traj, ts);

    // print the result and compare with the original pts
    std::cout << "final cost: " << final_cost << endl;
    std::cout<<"ctl pts:"<<endl;
    // FIXME remember modify this output to compare
    std::cout << traj_pts1<<endl;
    std::cout<<"result pts:"<<endl;
    std::cout << ctrl_pts_temp<<endl;


// extract the result speed and acc and print
    UniformBspline exec_traj_Bspline2(ctrl_pts_temp, 3, 0.5);
    auto v_spline2 = exec_traj_Bspline2.getDerivative();
    auto acc_spline2 = v_spline2.getDerivative();
    std::vector<geometry_msgs::Vector3> ego_planning_trajectory_xyyaw2;
    std::vector<geometry_msgs::Vector3> ego_planning_trajectory_tva2;
    for(double t = 0.0; t < 5.1; t += 0.5)
    {
        geometry_msgs::Vector3 xyyaw;
        geometry_msgs::Vector3 tva;
        auto pt = exec_traj_Bspline2.evaluateDeBoor(t);
        auto speed = v_spline2.evaluateDeBoor(t);
        auto acc = acc_spline2.evaluateDeBoor(t);
        float yaw = atan2(speed[1], speed[0]);
        xyyaw.x = pt[0];
        xyyaw.y = pt[1];
        xyyaw.z = yaw;
        tva.x = t;
        tva.y = sqrt(speed[0]*speed[0] + speed[1]*speed[1]);
        // dot_product of Eigen::VectorXd speed and acc
        float dot_product = speed[0]*acc[0] + speed[1]*acc[1];
        tva.z = acc.norm() * (dot_product > 0 ? 1 : -1);
        ego_planning_trajectory_xyyaw2.emplace_back(xyyaw);
        ego_planning_trajectory_tva2.emplace_back(tva);
    }
    std::ostringstream v_list_oss2, a_list_oss2;
    v_list_oss2 << "v_list: ";
    a_list_oss2 << "a_list: ";
    for(int i=0; i<ego_planning_trajectory_tva2.size();i++)
    {
        a_list_oss2 << std::fixed << std::setprecision(1) << ego_planning_trajectory_tva2[i].z << ", ";
        v_list_oss2 << std::fixed << std::setprecision(1) << ego_planning_trajectory_tva2[i].y << ", ";
    }
    std::cout << "Bspline:" << std::endl << v_list_oss2.str() << std::endl << a_list_oss2.str() << std::endl;

// visualize opt result 
    // for (int i=0;i<ctrl_pts_temp.cols();i++)
    // {
    //     a = ctrl_pts_temp.col(i);
    //     vis_path.push_back(a);
    // }
    // vis_all_paths.push_back(vis_path);
    // std::cout<<"vis size: "<<vis_path.size()<<endl;


//[VISUALIZE] publish to rviz
    // std::cout<<"vis size: "<<vis_all_paths.size()<<endl;
    // VisOptTrajs(vis_all_paths, vis_opt_trajs);

    return 0;
}
