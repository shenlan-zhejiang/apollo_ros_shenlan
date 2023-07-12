#pragma once

/*
#include "mpc/Trajectory.h"
#include "mpc/SingleMinco.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
*/
#include"modules/shenlan/mpc/minco_util/poly_traj_utils.hpp"
#include"modules/shenlan/mpc/proto/Trajectory.pb.h"
#include"modules/shenlan/mpc/proto/SingleMinco.pb.h"
#include"modules/shenlan/mpc/proto/nav_path.pb.h"
#include"modules/shenlan/mpc/proto/MincoTraj.pb.h"
#include "cyber/time/clock.h"
#include "cyber/time/duration.h"

#include <vector>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>

using namespace std;
using apollo::cyber::Clock;

namespace apollo {
namespace shenlan {

template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    // AERROR << "input time difference is too small";
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

inline double norm_yaw(double yaw)
{
  if (yaw > M_PI)
  {
    yaw -= 2*M_PI;
  }
  else if (yaw < -M_PI)
  {
    yaw += 2*M_PI;
  }

  return yaw;
}

inline double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= 1e-10) {
    // ADEBUG << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

inline double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol) {
  static constexpr double gr = 1.618033989;  // (sqrt(5) + 1) / 2

  double a = lower_bound;
  double b = upper_bound;

  double t = (b - a) / gr;
  double c = b - t;
  double d = a + t;

  while (std::abs(c - d) > tol) {
    if (func(c) < func(d)) {
      b = d;
    } else {
      a = c;
    }
    t = (b - a) / gr;
    c = b - t;
    d = a + t;
  }
  return (a + b) * 0.5;
}

struct TrajPoint
{
    double x;
    double y;
    double theta;
    double v;
    double a;
    double kappa;
    double s;
    double delta;
    double relative_time;

    TrajPoint(): x(0), y(0), theta(0), v(6.0), a(0), kappa(0), s(0), relative_time(0){}

    double PointDistanceSquare(const double& x_, const double& y_) const
    {
        const double dx = x - x_;
        const double dy = y - y_;

        return dx * dx + dy * dy;
    }
};

class TrajAnalyzer{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    minco_utils::MinJerkOpt jerk_opt;
    std::vector<minco_utils::Trajectory> minco_traj;
    std::vector<std::vector<minco_utils::Trajectory>> way_minco_traj;
    mpc::Trajectory traj;
    double start_time;
    std::vector<TrajPoint> waypoints;
    std::vector<std::vector<TrajPoint>> waytraj;
    std::vector<bool>reverse;
    size_t track_seg = 0;
    size_t track_point = 1;
    double total_time;

public:

    //nav_msgs::Path path;
    mpc::Nav_path  path;
    bool back_track = false;
    bool at_goal = false;

    minco_utils::State debug_pose;

    TrajAnalyzer() {}

    Eigen::Vector3d nextInitPoint()
    {
      if (track_seg < waytraj.size())
      {
        return Eigen::Vector3d(waytraj[track_seg][0].x, \
                waytraj[track_seg][0].y,\
                waytraj[track_seg][0].theta);
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    }

    void beginNextTraj()
    {
      if (track_seg < waytraj.size())
      {
        waypoints = waytraj[track_seg++];
        total_time = waypoints.back().relative_time;
        //ros::Duration d(3.0);
        cyber::Duration d(3.0);
        d.Sleep();
        start_time = Clock::NowInSeconds();
        track_point = 1;      
        at_goal = false;
      }
    }

    Eigen::Vector3d nextInitPointMinco()
    {
      if (track_seg < way_minco_traj.size())
      {
        std::vector<minco_utils::Trajectory> p = way_minco_traj[track_seg];
        Eigen::Vector2d po = p[0].getPos(0);
        return Eigen::Vector3d(po[0], po[1], p[0].getAngle(0));
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    }

    void beginNextTrajMinco()
    {
      if (track_seg < way_minco_traj.size())
      {
        waypoints = waytraj[track_seg];
        minco_traj = way_minco_traj[track_seg++];
        total_time = 0.0;
        for (size_t i=0; i<minco_traj.size(); i++)
          total_time += minco_traj[i].getTotalDuration();
        cyber::Duration d(3.0);
        d.Sleep();
        start_time = Clock::NowInSeconds();
        track_point = 1;
        at_goal = false;
      }
    }
 
    void setTraj(const std::shared_ptr<apollo::shenlan::mpc::Trajectory>& traj_)    //<<<<<<<<<<<<
    {
        traj = *traj_;
        //std::cout << "traj_type:" << traj_->traj_type() << std::endl;
        switch (traj_->traj_type())
        {
          case 0:
            { 
              start_time = traj.nav_path().header().timestamp_sec();
              double track_speed = 6.0;
              double track_acc = 0.0;
              waypoints.clear();
              TrajPoint start;
              start.a = track_acc;
              start.kappa = 0.0;
              start.relative_time = 0.0;
              start.s = 0.0;
              start.theta = 0.0;
              start.v = track_speed;
              start.x = traj.nav_path().pose()[0].position().x();
              start.y = traj.nav_path().pose()[0].position().y();
              waypoints.push_back(start);
              for (size_t i=1; i<(size_t)traj.nav_path().pose().size(); i++)
              {
                TrajPoint temp;
                Eigen::Matrix3d R = Eigen::Quaterniond(traj.nav_path().pose()[i].orientation().qw(),\
                                                      traj.nav_path().pose()[i].orientation().qx(),\
                                                      traj.nav_path().pose()[i].orientation().qy(),\
                                                      traj.nav_path().pose()[i].orientation().qz()).toRotationMatrix();
                temp.a = track_acc;
                temp.v = track_speed;
                temp.x = traj.nav_path().pose()[i].position().x();
                temp.y = traj.nav_path().pose()[i].position().y();
                double ds = sqrt(waypoints[i-1].PointDistanceSquare(temp.x, temp.y));
                temp.s = waypoints[i-1].s + ds;
                temp.relative_time = temp.s / track_speed;
                temp.theta =atan2(R.col(0)[1], R.col(0)[0]);
                temp.kappa = 0;
                waypoints.push_back(temp);
              }
              path = traj.nav_path();
            }
            break;
          case 1:
            {
              minco_traj.clear();
              reverse.clear();
              total_time = 0.0;
              for (size_t i=0; i<(size_t)traj.minco_path().trajs().size(); i++)
              {
                apollo::shenlan::mpc::SingleMinco sm = traj.minco_path().trajs()[i];
                Eigen::MatrixXd posP(2, sm.pos_pts().size() - 2);
                Eigen::VectorXd T(sm.t_pts().size());
                Eigen::MatrixXd head(2, 3),  tail(2,3);
                const int N = sm.t_pts().size();
                reverse.push_back(sm.reverse());
                int direction = sm.reverse()?-1:1;

                for (int i = 1; i < (int)sm.pos_pts().size() - 1; i++)
                {
                  posP(0, i - 1) = sm.pos_pts()[i].x();
                  posP(1, i - 1) = sm.pos_pts()[i].y();
                }
                for (int i = 0; i < (int)sm.t_pts().size(); i++)
                {
                  T(i) = sm.t_pts()[i];
                }

                head.row(0) = Eigen::Vector3d(sm.head_x().x(), sm.head_x().y(), sm.head_x().z());
                head.row(1) = Eigen::Vector3d(sm.head_y().x(), sm.head_y().y(), sm.head_y().z());
                tail.row(0) = Eigen::Vector3d(sm.tail_x().x(), sm.tail_x().y(), sm.tail_x().z());
                tail.row(1) = Eigen::Vector3d(sm.tail_y().x(), sm.tail_y().y(), sm.tail_y().z());

                jerk_opt.reset(head, tail, N);
                jerk_opt.generate(posP, T);
                minco_utils::Trajectory traj = jerk_opt.getTraj(direction);
                total_time += traj.getTotalDuration();
                minco_traj.push_back(traj);
                //std::cout << "start----:" << i << std::endl;
                //jerk_opt.print();
              }
              start_time = Clock::NowInSeconds();
            }
            break;
          default: break;
        }
    }

    std::vector<TrajPoint> getRefPoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      TrajPoint tp;
      double time_now = Clock::NowInSeconds();
      double t_cur = (time_now - start_time);
      int j=0;
      back_track = false;

      if (t_cur > total_time + 1.0)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }

      for (double t=t_cur+dt; j<T; j++, t+=dt)
      {
        double temp = t;
        bool gedit = false;
        for (size_t i=0 ;i<minco_traj.size(); i++)
        {
          if (temp-minco_traj[i].getTotalDuration() < 0)
          {
            minco_utils::State state;
            minco_traj[i].GetState(temp, state);

            debug_pose = state;

            tp.delta = minco_traj[i].getSteer(temp);
            tp.v = state.v;
            tp.x = state.x;
            tp.y = state.y;
            tp.a = state.a;
            tp.theta = norm_yaw(state.theta);
            tp.kappa = state.kappa;
            tp.s = state.s;
            at_goal = false;
            P.push_back(tp);
            gedit = true;
            // if (j==0)
            //   back_track = reverse[i];
            break;
          }
          else
          {
            temp -= minco_traj[i].getTotalDuration();
          }
        }
        if (gedit == false)
        {
          minco_utils::State state;
          minco_utils::Trajectory back_traj = minco_traj.back();
          back_traj.GetState(back_traj.getTotalDuration(), state);
          tp.v = state.v;
          tp.x = state.x;
          tp.y = state.y;
          tp.a = state.a;
          tp.delta = minco_traj.back().getSteer(back_traj.getTotalDuration());
          tp.kappa = state.kappa;
          tp.theta = norm_yaw(state.theta);
          tp.s = state.s;
          P.push_back(tp);
          // if (j==0)
          //     back_track = reverse.back();
        }
      }

      if (P[0].v < 0)
        back_track = true;
      return P;
    }

    TrajPoint getErrState(void)
    {
      double time_now = Clock::NowInSeconds();
      double t_cur = (time_now - start_time) ;
      TrajPoint tp;
      double t_temp = 0.0;

      for (size_t i=0 ;i<minco_traj.size(); i++)
      {
        if (t_cur-minco_traj[i].getTotalDuration() < 0)
        {
          minco_utils::State state;
          minco_traj[i].GetState(t_cur, state);
          tp.v = state.v;
          tp.x = state.x;
          tp.y = state.y;
          tp.theta = norm_yaw(state.theta);
          tp.relative_time = t_temp + t_cur;
          return tp;
        }
        else
        {
          t_cur -= minco_traj[i].getTotalDuration();
          t_temp += minco_traj[i].getTotalDuration();
        }
      }
      minco_utils::State state;
      minco_utils::Trajectory back_traj = minco_traj.back();
      back_traj.GetState(back_traj.getTotalDuration(), state);
      tp.v = state.v;
      tp.x = state.x;
      tp.y = state.y;
      tp.theta = norm_yaw(state.theta);
      tp.relative_time = back_traj.getTotalDuration();
      return tp;
    }

    std::vector<TrajPoint> getRefWaypoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      TrajPoint tp;
      double time_now = Clock::NowInSeconds();
      double t_cur = (time_now - start_time) ;
      // cout<<"seg="<<track_seg<<"   "<<"time="<<t_cur<<endl;
      int j = 0;
      size_t i = 1;
      
      back_track = false;

      if (t_cur > total_time + 1.0)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }
      
      for (; j<T; j++, t_cur+=dt)
      {
        bool gedit = false;
        for (; i<waypoints.size(); i++)
        {
          if (waypoints[i].relative_time > t_cur)
          {
            gedit = true;
            break;
          }
        }

        if (gedit)
        {
          Eigen::VectorXd s(Eigen::VectorXd::Zero(7));
          s[0] = waypoints[i-1].x;
          s[1] = waypoints[i-1].y;
          s[2] = waypoints[i-1].v;
          s[3] = waypoints[i-1].theta;
          s[4] = waypoints[i-1].a;
          s[5] = waypoints[i-1].delta;
          s[6] = waypoints[i-1].relative_time;
          Eigen::VectorXd s2(Eigen::VectorXd::Zero(7));
          s2[0] = waypoints[i].x;
          s2[1] = waypoints[i].y;
          s2[2] = waypoints[i].v;
          s2[3] = waypoints[i].theta;
          double dth = NormalizeAngle(s2[3] - s[3]);
          s2[3] = s[3] +dth;
          s2[4] = waypoints[i].a;
          s2[5] = waypoints[i].delta;
          s2[6] = waypoints[i].relative_time;
          Eigen::VectorXd s0 = lerp(s, waypoints[i-1].relative_time, s2, waypoints[i].relative_time, t_cur);
          tp.x = s0[0];
          tp.y = s0[1];
          tp.v = s0[2];
          tp.theta = s0[3];
          tp.a = s0[4];
          tp.delta = s0[5];
          tp.relative_time = s0[6];
          P.push_back(tp);
        }
        else
        {
          tp.x = waypoints.back().x;
          tp.y = waypoints.back().y;
          tp.v = waypoints.back().v;
          tp.theta = waypoints.back().theta;
          tp.a = waypoints.back().a;
          tp.delta = waypoints.back().delta;
          tp.relative_time = waypoints.back().relative_time;
          P.push_back(tp);
        }
      }

      if (P[0].v < 0)
        back_track = true;
      return P;
    }

    TrajPoint getErrWaystate(void)
    {
      TrajPoint tp;
      double time_now = Clock::NowInSeconds();
      double t_cur = (time_now - start_time) ;
      size_t i = 1;

      for (; i<waypoints.size(); i++)
      {
        if (waypoints[i].relative_time > t_cur)
        {
          Eigen::VectorXd s(Eigen::VectorXd::Zero(7));
          s[0] = waypoints[i-1].x;
          s[1] = waypoints[i-1].y;
          s[2] = waypoints[i-1].v;
          s[3] = waypoints[i-1].theta;
          s[4] = waypoints[i-1].a;
          s[5] = waypoints[i-1].delta;
          s[6] = waypoints[i-1].relative_time;
          Eigen::VectorXd s2(Eigen::VectorXd::Zero(7));
          s2[0] = waypoints[i].x;
          s2[1] = waypoints[i].y;
          s2[2] = waypoints[i].v;
          s2[3] = waypoints[i].theta;
          double dth = NormalizeAngle(s2[3] - s[3]);
          s2[3] = s[3] +dth;
          s2[4] = waypoints[i].a;
          s2[5] = waypoints[i].delta;
          s2[6] = waypoints[i].relative_time;
          Eigen::VectorXd s0 = lerp(s, waypoints[i-1].relative_time, s2, waypoints[i].relative_time, t_cur);
          tp.x = s0[0];
          tp.y = s0[1];
          tp.v = s0[2];
          tp.theta = s0[3];
          tp.a = s0[4];
          tp.delta = s0[5];
          tp.relative_time = s0[6];

          return tp;
        }
      }
      tp.x = waypoints.back().x;
      tp.y = waypoints.back().y;
      tp.v = waypoints.back().v;
      tp.theta = waypoints.back().theta;
      tp.a = waypoints.back().a;
      tp.delta = waypoints.back().delta;
      tp.relative_time = waypoints.back().relative_time;

      return tp;
    }

    ~TrajAnalyzer() {}
};

}
}