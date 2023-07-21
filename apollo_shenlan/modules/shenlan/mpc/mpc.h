#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <algorithm>
 
#include "OsqpEigen/OsqpEigen.h"
#include <osqp.h>

//#include <ros/ros.h>
//#include <ackermann_msgs/AckermannDrive.h>
//#include <ackermann_msgs/AckermannDriveStamped.h>



//#include <carla_msgs/CarlaEgoVehicleControl.h>
//#include <carla_msgs/CarlaEgoVehicleStatus.h>
//#include <nav_msgs/Odometry.h>

#include "cyber/time/clock.h"
#include "cyber/time/duration.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/shenlan/mpc/proto/EgoVehicleInfo.pb.h"
#include "modules/shenlan/mpc/proto/EgoVehicleControl.pb.h"
#include "modules/shenlan/mpc/proto/EgoVehicleStatus.pb.h"
#include "modules/shenlan/mpc/proto/Trajectory.pb.h"

#include "modules/shenlan/mpc/common/traj_anal.hpp"
#include "modules/shenlan/mpc/common/cubic_spline_planner.h"

using namespace std;
using namespace Eigen;
namespace apollo{
namespace shenlan {
class MPCState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    double x = 0;
    double y = 0;
    double v = 0; 
    double theta = 0;
};

class MPC
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    // parameters
    /// algorithm param
    double du_th = 0.1;
    double dt = 0.2;
    double wheel_base = 2.5;
    int T = 5;
    int max_iter = 3;
    vector<double> Q = {10, 10, 2.5, 0.5};
    vector<double> R = {0.01, 0.01};
    vector<double> Rd = {0.01, 1.0};
    /// constraints
    double max_steer = M_PI / 4;
    double max_dsteer = M_PI / 6;
    double max_csteer = M_PI / 6 * 0.2;
    double max_speed = 55.0 / 3.6;
    double min_speed = -20.0 / 3.6;
    double max_cv = 0.2;
    double max_accel = 1.0; 

    // MPC dataset
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd C;
    MPCState xbar[240];
    Eigen::MatrixXd xref;
    Eigen::MatrixXd dref;
    Eigen::MatrixXd output;
    Eigen::MatrixXd last_output;

    // control data
    int car_id_;
    bool has_odom;
    bool receive_traj_ = false;
    bool control_a = true;
    bool feed_forward = true;
    double traj_duration_;
    double t_track = 0.0;
    //ros::Time start_time_;
    double start_time_;
    MPCState now_state;

    // ros interface
	//ros::NodeHandle node_;
    //ros::Timer cmd_timer_;
    double cmd_timer_;
    //ros::Publisher pos_cmd_pub_, vis_pub, true_odom_pub, predict_pub, ref_pub, debug_pose_pub, err_pub, init_pub;
    //ros::Subscriber odom_sub_, traj_sub_, status_sub_, trigger_sub_;

   // ackermann_msgs::AckermannDrive cmd;
    //void cmdCallback(const ros::TimerEvent &e);
    void cmdCallback();
    /*
    void trajCallback(const mpc::TrajectoryConstPtr msg);
    void odomCallback(const nav_msgs::OdometryConstPtr msg);
    void statusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr msg);
    void triggerCallback(const geometry_msgs::PoseStamped msg);
    */


    void trajCallback(const std::shared_ptr<apollo::shenlan::mpc::Trajectory> &msg);
    void odomCallback(const std::shared_ptr<localization::LocalizationEstimate>  &msg);
    void statusCallback(const std::shared_ptr<apollo::shenlan::mpc::EgoVehicleStatus> &msg);
    void triggerCallback(const std::shared_ptr<apollo::shenlan::mpc::Nav_path> &msg );
    TrajAnalyzer trajectory_analyzer_;

    // for test tracking performance
    bool in_test;
    cubic_spline_planner csp;
    vector<Eigen::Vector3d> csp_path;
    string test_traj;
    std::ofstream outfile;

    // MPC function
    void getLinearModel(const MPCState& s, double delta);
    void stateTrans(MPCState& s, double a, double delta);
    void predictMotion(void);
    void predictMotion(MPCState *b);
    void solveMPCV(void);
    void solveMPCA(void);
    void getCmd(void);

    // utils
    MPCState xopt[240];
    void normlize_theta(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }
    void smooth_yaw(void)
    {
        double dyaw = xref(3, 0) - now_state.theta;

        while (dyaw >= M_PI / 2)
        {
            xref(3, 0) -= M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }
        while (dyaw <= -M_PI / 2)
        {
            xref(3, 0) += M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }

        for (int i=0; i<T-1; i++)
        {
            dyaw = xref(3, i+1) - xref(3, i);
            while (dyaw >= M_PI / 2)
            {
                xref(3, i+1) -= M_PI * 2;
                dyaw = xref(3, i+1) - xref(3, i);
            }
            while (dyaw <= -M_PI / 2)
            {
                xref(3, i+1) += M_PI * 2;
                dyaw = xref(3, i+1) - xref(3, i);
            }
        }
    }
    double get_nearest(void)
    {
        // Eigen::Vector3d track_point = csp.get_state(t_track);
        Eigen::Vector3d track_point = csp.get_state_back(t_track);
        Eigen::Vector2d now_point(now_state.x, now_state.y);
        double min_dist = (now_point-track_point.head(2)).norm();
        double track_temp = t_track;
        double interval = -now_state.v * dt;
        
        for (double t_temp = track_temp + interval, i=1.0; i < 10; i += 1.0, t_temp += interval)
        // for (double t_temp = track_temp + 1.0, i=1.0; i < 10; i += 1.0, t_temp += 1.0)
        {
            if (t_temp > traj_duration_)
            {
                t_temp = traj_duration_;
            }
            // Eigen::Vector3d temp = csp.get_state(t_temp);
            Eigen::Vector3d temp = csp.get_state_back(t_temp);
            double dist = (temp.head(2) - now_point).norm();
         
            if (dist<min_dist)
            {
                min_dist = dist;
                t_track = t_temp;
            }
        }
        // ROS_INFO("t_track=%f,traj_duration=%f",t_track,traj_duration_);

        return t_track;
    }
    /*
    void drawFollowPath(void)
    {
        int id = 0;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 0.1;
        sphere.scale.x = 1.0;
        sphere.scale.y = 1.0;
        sphere.scale.z = 1.0;
        line_strip.scale.x = 1.0 / 2;
        geometry_msgs::Point pt;
        
        for (auto p:trajectory_analyzer_.waypoints)
        {
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.4;
            line_strip.points.push_back(pt);
        }
        vis_pub.publish(line_strip);
    }
    void drawPredictPath(MPCState *b)
    {
        int id = 0;
        double sc = 0.12;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 1;
        sphere.color.b = line_strip.color.b = 0;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        for (int i=0; i<T; i++)
        {
            // pt.x = xref(0, i);
            // pt.y = xref(1, i);
            pt.x = b[i].x;
            pt.y = b[i].y;
            pt.z = 0.4;
            line_strip.points.push_back(pt);
        }
        predict_pub.publish(line_strip);
    }
    void drawRefPath(void)
    {
        int id = 0;
        double sc = 0.12;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        for (int i=0; i<T; i++)
        {
            pt.x = xref(0, i);
            pt.y = xref(1, i);
            pt.z = 0.4;
            line_strip.points.push_back(pt);
        }
        ref_pub.publish(line_strip);
    }
    */

public:
	MPC() : csp(vector<double>{0.0, 3.0, 2.7, 1.5, 0.3, 0, 3.0, 2.7, 1.5, 0.3, 0.0}, vector<double>{0.0, 0.0, 2.0, 3.0, 4.0, 6.0, 6.0, 4.0, 3.0, 2.0, 0.0}, 0.1) {}
	// MPC() : csp(vector<double>{0.0, 4.0, 0.8, 2.67, 4.67, 1.33, 0.0, 0.0}, vector<double>{0.0, 0.0, 1.33, 4.67, 2.67, 4.0, 0.67, 0.0}, 0.13) {}
	// MPC() : csp(vector<double>{0.0, 30.0, 6.0, 20.0, 35.0, 10.0, 0.0, 0.0}, vector<double>{0.0, 0.0, 20.0, 35.0, 20.0, 30.0, 5.0, 0.0}, 1.0) {}
    void init();
	~MPC() {}
};

}
}
