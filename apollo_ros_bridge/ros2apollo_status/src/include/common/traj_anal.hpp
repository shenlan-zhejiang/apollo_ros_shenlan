#pragma once

#include "mpc/Trajectory.h"
#include "minco_util/poly_traj_utils.hpp"
#include "mpc/SingleMinco.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>

using namespace std;

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
    minco_utils::MinJerkOpt jerk_opt;
    std::vector<minco_utils::Trajectory> minco_traj;
    std::vector<std::vector<minco_utils::Trajectory>> way_minco_traj;
    mpc::Trajectory traj;
    ros::Time start_time;
    std::vector<TrajPoint> waypoints;
    std::vector<std::vector<TrajPoint>> waytraj;
    std::vector<bool>reverse;
    size_t track_seg = 0;
    size_t track_point = 1;
    double total_time;

public:

    nav_msgs::Path path;
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
        ros::Duration d(3.0);
        d.sleep();
        start_time = ros::Time::now();
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
        ros::Duration d(3.0);
        d.sleep();
        start_time = ros::Time::now();
        track_point = 1;
        at_goal = false;
      }
    }

    void setTraj(const char* file)
    {
      waytraj.clear();
      way_minco_traj.clear();
      TrajPoint tp;
      std::ifstream fp;
      size_t i = 0;
      vector<double> tp_buf;

      fp.open(file,ios::in);
      if (file[16]!='m')
      {
        string trajs;
        while(getline(fp, trajs))
        {
          if (trajs[0]=='0')
          {
            continue;
          }
          else
          {
            vector<TrajPoint> wv;
            wv.clear();
            waytraj.push_back(wv);
            string traj_msg = trajs.substr(trajs.find("traj") + 5, trajs.size());
            size_t wow = traj_msg.find('!');
            while (wow != traj_msg.npos)
            {
              string state_msg = traj_msg.substr(0, wow);
              size_t dott = state_msg.find(',');
              size_t state_cnt = 0;
              while (dott != state_msg.npos)
              {
                double temp = atof(const_cast<const char*>(state_msg.substr(0, dott).c_str()));
                tp_buf.push_back(temp);
                if (dott+1>=state_msg.size())
                  break;
                state_msg = state_msg.substr(dott+1, state_msg.size());
                dott = state_msg.find(',');
              }
              tp.relative_time = tp_buf[0];
              tp.x = tp_buf[1] - 200;
              tp.y = tp_buf[2] - 100;
              tp.theta = tp_buf[3];
              
              // teb
              if (file[16]=='t')
              {
                tp.v = tp_buf[4];
                tp.a = tp_buf[5];
              }
              else    // pjso, hobca, libai
              {
                tp.kappa = tp_buf[4];
                tp.delta = tp_buf[5];
                tp.v = tp_buf[6];
                tp.a = tp_buf[7];
              }
              waytraj[i].push_back(tp);
              tp_buf.clear();
              if (wow+2>=traj_msg.size())
                break;
              traj_msg = traj_msg.substr(wow+2, traj_msg.size());
              wow = traj_msg.find('!');
            }
          }
          i++;
          cout<<"get traj: "<<i<<endl;
        }
        cout<<"Traj read down, begin tracking!"<<endl;
      }
      else
      {
        string trajs;
        while(getline(fp, trajs))
        {
          if (trajs[0]=='0')
          {
            continue;
          }
          else
          {
            vector<TrajPoint> wv;
            wv.clear();
            waytraj.push_back(wv);
            vector<minco_utils::Trajectory> one_minco_traj;
            one_minco_traj.clear();
            way_minco_traj.push_back(one_minco_traj);
            string traj_msg = trajs.substr(trajs.find("traj") + 5, trajs.size());
            size_t wow = traj_msg.find('!');

            Eigen::MatrixXd head(2, 3),  tail(2,3);
            int direction = 1;

            while (wow != traj_msg.npos)
            {
              string state_msg = traj_msg.substr(0, wow);
              size_t dott = state_msg.find(',');
              size_t state_cnt = 0;
              while (dott != state_msg.npos)
              {
                double temp = atof(const_cast<const char*>(state_msg.substr(0, dott).c_str()));
                tp_buf.push_back(temp);
                // cout<<temp<<" ";
                state_msg = state_msg.substr(dott+1, state_msg.size());
                dott = state_msg.find(',');
                if (++state_cnt==12)
                  break;
              }
              cout<<endl;
              // head.row(0) = Eigen::Vector3d(tp_buf[0], tp_buf[1], tp_buf[2]);
              // head.row(1) = Eigen::Vector3d(tp_buf[3], tp_buf[4], tp_buf[5]);
              // tail.row(0) = Eigen::Vector3d(tp_buf[6], tp_buf[7], tp_buf[8]);
              // tail.row(1) = Eigen::Vector3d(tp_buf[9], tp_buf[10], tp_buf[11]);
              head.row(0) = Eigen::Vector3d(tp_buf[0]-200, tp_buf[1], tp_buf[2]);
              head.row(1) = Eigen::Vector3d(tp_buf[3]-100, tp_buf[4], tp_buf[5]);
              tail.row(0) = Eigen::Vector3d(tp_buf[6]-200, tp_buf[7], tp_buf[8]);
              tail.row(1) = Eigen::Vector3d(tp_buf[9]-100, tp_buf[10], tp_buf[11]);

              tp_buf.clear();
              const int N = atoi(const_cast<const char*>(state_msg.substr(0, dott).c_str())) / 2;
              // cout<<"n="<<N<<endl;
              state_msg = state_msg.substr(dott+1, state_msg.size());
              dott = state_msg.find(',');
              state_cnt = 0;
              while (dott != state_msg.npos)
              {
                double temp = atof(const_cast<const char*>(state_msg.substr(0, dott).c_str()));
                tp_buf.push_back(temp);
                // cout<<" state-cnt="<<state_cnt<<"   ";
                state_msg = state_msg.substr(dott+1, state_msg.size());
                dott = state_msg.find(',');
                if (++state_cnt==3*N-1)
                {
                  // cout<<"ok"<<endl;
                  break;
                }
              }

              Eigen::MatrixXd posP(2, N - 2);
              Eigen::VectorXd T(N - 1);

              // cout<<"posP:  "<<endl;
              for (int k = 1, j=2; k < N-1; k++, j+=2)
              {
                // posP(0, k - 1) = tp.x = tp_buf[j];
                posP(0, k - 1) = tp_buf[j]-200;
                // posP(1, k - 1) = tp.y = tp_buf[j+1];
                posP(1, k - 1) = tp_buf[j+1]-100;
                // cout<<posP(0, k-1)<<"  "<<posP(1, k-1)<<endl;
              }
              // cout<<endl<<"T:   ";
              for (int k = 0, j=2*N; k < N - 1; k++, j++)
              {
                T(k) = tp_buf[j];
                // cout<< T(k)<<"  ";
              }
              jerk_opt.reset(head, tail, N-1);
              jerk_opt.generate(posP, T);
              minco_utils::Trajectory temp_traj = jerk_opt.getTraj(direction);
              direction = -direction;
              way_minco_traj[i].push_back(temp_traj);
              for (double ti=0; ti<temp_traj.getTotalDuration(); ti+=0.1)
              {
                Eigen::Vector2d tip = temp_traj.getPos(ti);
                tp.x = tip[0];
                tp.y = tip[1];
                waytraj[i].push_back(tp);
              }
              tp_buf.clear();
              if (wow+2>=traj_msg.size())
                break;
              traj_msg = traj_msg.substr(wow+2, traj_msg.size());
              wow = traj_msg.find('!');
            }
          }
          i++;
          // if (i>5)
          //   break;
          cout<<"get traj: "<<i<<endl;
        }
        cout<<"Traj read down, begin tracking!"<<endl;
      }
      at_goal = true;
      fp.close();
    }

    void setTraj(mpc::TrajectoryConstPtr traj_)
    {
        traj = *traj_;
        switch (traj_->traj_type)
        {
          case 0:
            { 
              start_time = traj.nav_path.header.stamp;
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
              start.x = traj.nav_path.poses[0].pose.position.x;
              start.y = traj.nav_path.poses[0].pose.position.y;
              waypoints.push_back(start);
              for (size_t i=1; i<traj.nav_path.poses.size(); i++)
              {
                TrajPoint temp;
                Eigen::Matrix3d R = Eigen::Quaterniond(traj.nav_path.poses[i].pose.orientation.w,\
                                                      traj.nav_path.poses[i].pose.orientation.x,\
                                                      traj.nav_path.poses[i].pose.orientation.y,\
                                                      traj.nav_path.poses[i].pose.orientation.z).toRotationMatrix();
                temp.a = track_acc;
                temp.v = track_speed;
                temp.x = traj.nav_path.poses[i].pose.position.x;
                temp.y = traj.nav_path.poses[i].pose.position.y;
                double ds = sqrt(waypoints[i-1].PointDistanceSquare(temp.x, temp.y));
                temp.s = waypoints[i-1].s + ds;
                temp.relative_time = temp.s / track_speed;
                temp.theta =atan2(R.col(0)[1], R.col(0)[0]);
                temp.kappa = 0;
                waypoints.push_back(temp);
              }
              path = traj.nav_path;
            }
            break;
          case 1:
            {
              minco_traj.clear();
              reverse.clear();
              total_time = 0.0;
              for (size_t i=0; i<traj.minco_path.trajs.size(); i++)
              {
                mpc::SingleMinco sm = traj.minco_path.trajs[i];
                Eigen::MatrixXd posP(2, sm.pos_pts.size() - 2);
                Eigen::VectorXd T(sm.t_pts.size());
                Eigen::MatrixXd head(2, 3),  tail(2,3);
                const int N = sm.t_pts.size();
                reverse.push_back(sm.reverse);
                int direction = sm.reverse?-1:1;

                for (int i = 1; i < (int)sm.pos_pts.size() - 1; i++)
                {
                  posP(0, i - 1) = sm.pos_pts[i].x;
                  posP(1, i - 1) = sm.pos_pts[i].y;
                }
                for (int i = 0; i < (int)sm.t_pts.size(); i++)
                {
                  T(i) = sm.t_pts[i];
                }

                head.row(0) = Eigen::Vector3d(sm.head_x.x, sm.head_x.y, sm.head_x.z);
                head.row(1) = Eigen::Vector3d(sm.head_y.x, sm.head_y.y, sm.head_y.z);
                tail.row(0) = Eigen::Vector3d(sm.tail_x.x, sm.tail_x.y, sm.tail_x.z);
                tail.row(1) = Eigen::Vector3d(sm.tail_y.x, sm.tail_y.y, sm.tail_y.z);

                jerk_opt.reset(head, tail, N);
                jerk_opt.generate(posP, T);
                minco_utils::Trajectory traj = jerk_opt.getTraj(direction);
                total_time += traj.getTotalDuration();
                minco_traj.push_back(traj);
              }
              start_time = ros::Time::now();
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
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
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
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
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
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
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
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
      int j = 0;
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
