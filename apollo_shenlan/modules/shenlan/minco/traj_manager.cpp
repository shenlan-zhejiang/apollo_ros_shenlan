#include "modules/shenlan/minco/plan_manage/traj_manager.h"

// #include "cyber/cyber.h"

// #include "cyber/class_loader/class_loader.h"
// #include "cyber/component/component.h"
// #include "cyber/common/util.h"

// #include "cyber/init.h"
// #include "cyber/time/time.h"
// #include "cyber/io/session.h"


#include "modules/localization/proto/localization.pb.h"
#include "modules/shenlan/mpc/proto/EgoVehicleInfo.pb.h"
#include "modules/shenlan/mpc/proto/EgoVehicleControl.pb.h"
#include "modules/shenlan/mpc/proto/EgoVehicleStatus.pb.h"
#include "modules/shenlan/mpc/proto/Trajectory.pb.h"
#include "modules/shenlan/proto/shenlan_conf.pb.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <omp.h>
#include <cmath>
#include <chrono>

using namespace chrono;

// void TrajPlanner::Init(const std::string config_path, const int car_id) 
void TrajPlanner::init(apollo::shenlan::ShenlanConf &shenlan_conf)
{

  resolution_ = shenlan_conf.mapping_conf().resolution();

  //apollo::shenlan::TrajPlannerConf trajplanner_conf;
  traj_piece_duration_ = shenlan_conf.trajplanner_conf().traj_piece_duration();//1.0; //duration of per trajectory piece
  traj_res_ = shenlan_conf.trajplanner_conf().traj_res();//8;
  dense_traj_res_ = shenlan_conf.trajplanner_conf().dense_traj_res();//20;

  //apollo::shenlan::VehicleConf vehicle_conf;
  cars_num_ = shenlan_conf.vehicle_conf().cars_num();//1;
  car_id_ = shenlan_conf.vehicle_conf().car_id();//0;
  car_width_ = shenlan_conf.vehicle_conf().car_width();//2.1;
  car_length_ = shenlan_conf.vehicle_conf().car_length();//4.8;
  car_wheelbase_ = shenlan_conf.vehicle_conf().car_wheelbase();//3.0;
  car_d_cr_ =  shenlan_conf.vehicle_conf().car_d_cr();//1.3864;

  swarm_traj_container_.resize(cars_num_);
  swarm_last_traj_container_.resize(cars_num_);
  have_received_trajs_.resize(cars_num_);
  fill(have_received_trajs_.begin(), have_received_trajs_.end(), false);
  // for(int i = 0; i < cars_num_; i++)
  // {
  //   have_received_trajs_[i] = false;
  // }

  //ploy_traj_opt_->setSurroundTrajs(&traj_container_.surround_traj);

  /*  kino a* intial  */
  kino_path_finder_ = std::make_unique<path_searching::KinoAstar>();
  kino_path_finder_->init(shenlan_conf);

  ploy_traj_opt_ = std::make_unique<plan_manage::PolyTrajOptimizer>();
  // ploy_traj_opt_->setParam(nh_, cfg_);
  ploy_traj_opt_->init(shenlan_conf);

  // kino_path_finder_->init(cfg_,nh_);
  //KinopathPub_ = nh_.advertise<visualization_msgs::Marker>("car_"+to_string(car_id_)+"_kino_trajs", 10);
  //Rectangle_poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("car_"+to_string(car_id_)+"_polyhedrons", 1);
  //minco_traj_pub_ = nh_.advertise<nav_msgs::Path>("vis/car_"+to_string(car_id_)+"_minco_traj", 2); // for visualizaion
  //MincoPathPub_ = nh_.advertise<mpc::Trajectory>("/carla/ego_vehicle/trajectory", 1);
  //MincoPathPub_ = nh_.advertise<mpc::Trajectory>("/carla/agent_" + to_string(car_id_) + "/trajectory", 1); // for control
  //TrajPathPub_ = nh_.advertise<swarm_bridge::Trajectory>("/broadcast_traj_from_planner", 1); // for swarm communication
  //wholebody_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("vis/car_"+to_string(car_id_)+"wholebodyTraj", 2);


  // cout << "333333333333333333" << endl;
  // KinopathPub = nh_.advertise<nav_msgs::Path>("/KinoPathMsg", 1);
  // DenseKinopathPub = nh_.advertise<nav_msgs::Path>("/vis_dense_kino_traj", 1);
  // DensePathVisPub = nh_.advertise<geometry_msgs::PoseArray>("/libai/vis_front_end",1);
  // DensePathPub = nh_.advertise<nav_msgs::Path>("/libai/front_end",1);

  /*debug*/

  // Debugtraj0Pub = nh_.advertise<nav_msgs::Path>("/debug/vis_traj_0", 1);
  // Debugtraj1Pub = nh_.advertise<nav_msgs::Path>("/debug/vis_traj_1", 1);
  // DebugCorridorPub = nh_.advertise<visualization_msgs::Marker>("/debug/corridor", 1);
  // DebugtrajPub =   nh_.advertise<visualization_msgs::Marker>("/debug/traj", 1);

}

// use kinodynamic a* to generate a path
bool TrajPlanner::getKinoPath(Eigen::Vector4d &end_state, bool first_search, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{
  //std::cout << "+++++++++++++getKinoPath++++++++++++++" << std::endl;  

  Eigen::Vector4d start_state;
  Eigen::Vector2d init_ctrl;

  // start_state << head_state_.vec_position, head_state_.angle, head_state_.velocity;
  start_state = start_state_;
  //std::cout << "start_state: " << start_state.transpose() << std::endl;  
  // init_ctrl << head_state_.steer, head_state_.acceleration;
  init_ctrl << start_ctrl_;
  //std::cout << "init_ctrl: " << init_ctrl.transpose() << std::endl;
  //steer and acc
  //std::cout<< "end_state: "<<end_state.transpose()<<std::endl;

  Eigen::Vector2d start_pos = start_state.head(2);
  kino_path_finder_->findNearestNode(start_pos, first_search);
  kino_path_finder_->reset();
  
  // ros::Time searcht1 = ros::Time::now();
  auto searcht1 = high_resolution_clock::now();
  //std::cout << "aaaaaaaaaaa1" << std::endl;
  int status = kino_path_finder_->search(start_state, init_ctrl, end_state, true, buf_msg);
  //std::cout << "aaaaaaaaaaa2" << std::endl;
  // ros::Time searcht2 = ros::Time::now();
  auto searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> search_ms = searcht2 - searcht1;
  // auto  = duration_cast<microseconds>(searcht2 - searcht1);
  // std::cout<<"search time: "<<(searcht2-searcht1).toSec() * 1000.0<<std::endl;
  std::cout<<"search time: "<< search_ms.count() << " ms" <<std::endl;
  if (status == path_searching::KinoAstar::NO_PATH)
  {
    std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_state, init_ctrl, end_state, false, buf_msg);
    if (status == path_searching::KinoAstar::NO_PATH)
    {
      std::cout << "[kino replan]: Can't find path." << std::endl;
      return false;
    }
    else
    {
      std::cout << "[kino replan]: retry search success." << std::endl;
    }
  }
  else
  {
    std::cout << "[kino replan]: kinodynamic search success." << std::endl;
  }

  auto time_searcht1 = high_resolution_clock::now();
  kino_path_finder_->getTruncatedposLists();
  kino_path_finder_->getSingulNodes();
  bool middle_node_success = kino_path_finder_->searchTime(kino_trajs_, start_time_);
  auto time_searcht2 = high_resolution_clock::now();
  std::chrono::duration<double, std::milli> time_searchms = time_searcht2 - time_searcht1;
  std::cout << "time serach time: " << time_searchms.count() << " ms" << std::endl;
  // kino_path_finder_->getKinoNode(kino_trajs_);  
  if(!middle_node_success)
    return false;

  return true;
}

void TrajPlanner::setInitStateAndInput(const Eigen::Vector4d& state, const double& start_time)
{ 
  std::cout << "1111111111setState1111111111" << std::endl;
  //std::cout << "state: " << std::endl << state << std::endl;
  //std::cout << "start_time: " << std::endl << start_time << std::endl;

  has_init_state_ = true;
  //std::cout << "has_init_state_: " << std::endl << has_init_state_ << std::endl;
  start_state_ = state;
  //std::cout << "start_state_: " << std::endl << start_state_ << std::endl;
  start_time_ = start_time;
  //std::cout << "start_time_: " << std::endl << start_time_ << std::endl;

  Eigen::Vector2d init_ctrl(0.0, 0.0);
  start_ctrl_ = init_ctrl;
  std::cout << "start_ctrl_: " << std::endl << init_ctrl << std::endl;
}

void TrajPlanner::setInitStateAndInput(const double& t, Eigen::Vector4d& replan_init_state)
{
  std::cout << "2222222222setState2222222222" << std::endl;

  has_init_state_ = true;
  start_time_  = t;

  int id = traj_container_.locateSingulId(t);
  

  double t_bar = t - traj_container_.singul_traj[id].start_time;
  if(t_bar > traj_container_.singul_traj[id].duration)
  {
      t_bar = traj_container_.singul_traj[id].duration;
  }
  int singul = traj_container_.singul_traj[id].traj.getSingul(t_bar);
  Eigen::Vector2d start_pos = traj_container_.singul_traj[id].traj.getPos(t_bar);
  Eigen::Vector2d start_vel = traj_container_.singul_traj[id].traj.getdSigma(t_bar);
  Eigen::Vector2d start_acc = traj_container_.singul_traj[id].traj.getddSigma(t_bar);
  double init_yaw = atan2(singul * start_vel(1), singul * start_vel(0));
  double init_vel = singul * start_vel.norm();
  Eigen::Vector4d init_state;
  //init_state << start_pos[0]-587061, start_pos[1]-4141628, init_yaw, init_vel;
  init_state << start_pos, init_yaw, init_vel;
  //std::cout << "init_state: " << std::endl << init_state << std::endl;

  Eigen::Matrix2d B;
  B << 0, -1,
       1,  0;
  double init_steer = atan(singul * (start_acc.transpose() * B * start_vel)(0, 0) * car_wheelbase_ / pow(start_vel.norm(), 3));
  double init_acc = singul * (start_vel.transpose() * start_acc)(0, 0) / start_vel.norm();
  Eigen::Vector2d init_input(init_steer, init_acc);

  //std::cout << "init_steer" << init_steer << std::endl;
  //std::cout << "init_acc" << init_acc << std::endl;
  //std::cout << "init_yaw" << init_yaw << std::endl;

  replan_init_state = init_state;
  start_state_ = init_state;
  start_ctrl_  = init_input; 
}

void TrajPlanner::displayKinoPath(std::shared_ptr<plan_utils::KinoTrajData> kino_trajs){
    return;
}
/*
void TrajPlanner::displayKinoPath(std::shared_ptr<plan_utils::KinoTrajData> kino_trajs)
{
  visualization_msgs::Marker sphere, line_strip, carMarkers;
  sphere.header.frame_id = line_strip.header.frame_id = carMarkers.header.frame_id = "map";
  sphere.header.stamp = line_strip.header.stamp = carMarkers.header.stamp = apollo::cyber::Time::Now().ToNanosecond() / 1.0e9;
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  // carMarkers.type = visualization_msgs::Marker::LINE_LIST;


  sphere.action = visualization_msgs::Marker::DELETE;
  line_strip.action = visualization_msgs::Marker::DELETE;
  // carMarkers.action = visualization_msgs::Marker::DELETE;

  KinopathPub_.publish(sphere);
  KinopathPub_.publish(line_strip);
  // path_pub.publish(carMarkers);

  sphere.action = line_strip.action = carMarkers.action = visualization_msgs::Marker::ADD;
  sphere.id = 0;
  line_strip.id = 1000;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.a = line_strip.color.a = 0.5;
  sphere.scale.x = 0.5;
  sphere.scale.y = 0.5;
  sphere.scale.z = 0.5;
  line_strip.scale.x = 0.25;

  geometry_msgs::Point pt;
  unsigned int size = kino_trajs->size();
  for (unsigned int i = 0; i < size; ++i){
    sphere.color.r = line_strip.color.r = i*1.0/(size*1.0);
    sphere.color.g = line_strip.color.g = 0.0;
    sphere.color.b = line_strip.color.b = i*1.0/(size*1.0);

    for (int k = 0; k < kino_trajs->at(i).traj_pts.size(); k++)
    {
      Eigen::Vector3d trajpt = kino_trajs->at(i).traj_pts[k];
      double yaw = kino_trajs->at(i).thetas[k];
      pt.x = trajpt(0);
      pt.y = trajpt(1);
      pt.z = 0.1;
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);

    }
  }

  KinopathPub_.publish(sphere);
  KinopathPub_.publish(line_strip);
  // path_pub.publish(carMarkers);
}
*/

void TrajPlanner::displayMincoTraj(std::shared_ptr<plan_utils::SingulTrajData> display_traj)
{
    return;
}


/*
void TrajPlanner::displayMincoTraj(std::shared_ptr<plan_utils::SingulTrajData> display_traj)
{
    auto path_msg = make_shared<apollo::shenlan::mpc::Nav_path>();
    apollo::common::Quaternion quaternion;
    quaternion.set_qw(1.0);
    quaternion.set_qx(0.0);
    quaternion.set_qy(0.0);
    quaternion.set_qz(0.0);
    apollo::common::PointENU position;
    for (unsigned int i = 0; i < display_traj->size(); ++i)
    {
        double total_duration = display_traj->at(i).duration;
        for (double t = 0; t <= total_duration; t += 0.01)
        {
            Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
            //pose.pose().position().x() = pt(0);
            //pose.pose().position().y() = pt(1);
            //pose.pose().position().z()= 0.2;
            position.set_x(pt(0));
            position.set_y(pt(0));
            position.set_z(0.2);
            auto pose = path_msg->add_pose();
            pose->mutable_position()->CopyFrom(position);
            pose->mutable_orientation()->CopyFrom(quaternion); 
        }
    }
    path_msg->mutable_header()->set_frame_id("map");

  double last_debugyaw =  display_traj->at(0).traj.getAngle(0.0);

  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.01){

      Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
      position.set_x(pt(0));
      position.set_y(pt(0));
      position.set_z(0.4);

      auto pose = path_msg->add_pose();
      pose->mutable_position()->CopyFrom(position);

      Eigen::Vector2d vel = display_traj->at(i).traj.getdSigma(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      // std::cout<<"pos: "<<pt.transpose()<<" vel: "<<vel.transpose()<<" yaw: "<<yaw<<std::endl;

      // if(fabs(yaw-last_debugyaw)>0.2){
      // }
      last_debugyaw = yaw;
    }

  }
  */

  /*
  visualization_msgs::Marker carMarkers;
  carMarkers.header.frame_id = "map";
  carMarkers.header.stamp = ros::Time::now();
  carMarkers.type = visualization_msgs::Marker::LINE_LIST;
  carMarkers.action = visualization_msgs::Marker::DELETE;
  wholebody_traj_pub_.publish(carMarkers);
  carMarkers.action = visualization_msgs::Marker::ADD;
  carMarkers.id = 21;
  carMarkers.pose.orientation.w = 1.00;
  carMarkers.ns = "trajwholepub";
  carMarkers.color.r = 1.00;
  carMarkers.color.g = 0.00;
  carMarkers.color.b = 1.00;
  carMarkers.color.a = 1.00;
  carMarkers.scale.x = 0.05;
  */

  /*
  geometry_msgs::Point pt;

  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.1){
      Eigen::Vector2d pos = display_traj->at(i).traj.getPos(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.1;
      geometry_msgs::Point point1;
      geometry_msgs::Point point2;
      geometry_msgs::Point point3;
      geometry_msgs::Point point4;
      Eigen::Matrix2d R;
      R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
      Eigen::Vector2d offset1, tmp1;
      offset1 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp1 = pos+offset1;
      point1.x = tmp1[0]; 
      point1.y = tmp1[1];
      point1.z = 0;

      Eigen::Vector2d offset2, tmp2;
      offset2 = R*Eigen::Vector2d(car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp2 = pos+offset2;
      point2.x = tmp2[0]; 
      point2.y = tmp2[1];
      point2.z = 0;

      Eigen::Vector2d offset3, tmp3;
      offset3 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,-car_width_/2.0);
      tmp3 = pos+offset3;
      point3.x = tmp3[0]; 
      point3.y = tmp3[1];
      point3.z = 0;

      Eigen::Vector2d offset4, tmp4;
      offset4 = R*Eigen::Vector2d(-car_length_/2.0+car_d_cr_,car_width_/2.0);
      tmp4 = pos+offset4;
      point4.x = tmp4[0]; 
      point4.y = tmp4[1];
      point4.z = 0;

      carMarkers.points.push_back(point1);
      carMarkers.points.push_back(point2);

      carMarkers.points.push_back(point2);
      carMarkers.points.push_back(point3);

      carMarkers.points.push_back(point3);
      carMarkers.points.push_back(point4);

      carMarkers.points.push_back(point4);
      carMarkers.points.push_back(point1);

    }

  }
  */

  //wholebody_traj_pub_.publish(carMarkers);

//}


bool TrajPlanner::checkCollisionWithObs(const double& t_now, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{
    //std::cout << "*********checkCollisionWithObs**********" << std::endl;


    bool collision = false;

    int segmentId = traj_container_.locateSingulId(t_now);
    double segment_i_start_time = traj_container_.singul_traj[segmentId].start_time;
    double segment_i_end_time = traj_container_.singul_traj[segmentId].end_time;
    int segment_i_singul = traj_container_.singul_traj[segmentId].traj.getDirection();
    if(t_now < segment_i_start_time || t_now > segment_i_end_time)
    {
        return collision;
    }
    else
    {
        double segment_i_duration = segment_i_end_time - segment_i_start_time;
        for(double t = t_now; t < segment_i_end_time - segment_i_duration / 3.0; t += 0.05)
        {
            Eigen::Vector2d pos = traj_container_.singul_traj[segmentId].traj.getPos(t - segment_i_start_time);
            Eigen::Vector2d vel = traj_container_.singul_traj[segmentId].traj.getdSigma(t - segment_i_start_time);
            double yaw = atan2(segment_i_singul * vel(1), segment_i_singul * vel(0));
            Eigen::Vector3d state; state << pos, yaw;
            kino_path_finder_->checkCollisionUsingPosAndYaw(state, collision, buf_msg);
            if(collision)
            {
                return collision;
            }
        }
    }

    return collision;
}

bool TrajPlanner::checkCollisionWithOtherCars(const double& t_now, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{
    bool collision = false;
    int num_have_received_trajs = 0;
    for(int car = 0; car < cars_num_; car++)
    {
        if(have_received_trajs_[car])
        {
            num_have_received_trajs++;
        }
    }
    if(num_have_received_trajs < cars_num_)
        return collision;
    
    double t_start = traj_container_.singul_traj[0].start_time;
    double t_end = traj_container_.singul_traj[traj_container_.singul_traj.size()-1].end_time;
    double total_duration = t_end - t_start;
    for(double t = t_now; t < t_end - total_duration / 3; t += 0.1)
    {
        collision = ploy_traj_opt_->checkCollisionWithSurroundCars(t);
        if(collision)
        {
            break;
        }
    }
    return collision;
}

bool TrajPlanner::RunMINCOParking(const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{

  // traj_container_.clearSingul();
  Eigen::MatrixXd flat_finalState(2, 3),  flat_headState(2,3);
  Eigen::VectorXd ego_piece_dur_vec;
  Eigen::MatrixXd ego_innerPs;
  cout<<"begin to run minco"<<endl;
  //nav_msgs::Path debug_msg0,debug_msg1;
  display_hPolys_.clear();

  // double worldtime =  head_state_.time_stamp;
  double worldtime = start_time_;
  double basetime = 0.0;

  /*try to merge optimization process*/
  std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
  std::vector<int> singul_container;
  Eigen::VectorXd duration_container;
  std::vector<Eigen::MatrixXd> waypoints_container;
  std::vector<Eigen::MatrixXd> iniState_container, finState_container;
  duration_container.resize(kino_trajs_.size());

  for(unsigned int i = 0; i < kino_trajs_.size(); i++)
  {
    double timePerPiece = traj_piece_duration_;
    //int segment_idx = i;
    plan_utils::FlatTrajData kino_traj = kino_trajs_.at(i);
    singul_container.push_back(kino_traj.singul);
    std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
    plan_utils::MinJerkOpt initMJO;
    plan_utils::Trajectory initTraj;
    int piece_nums;
    double initTotalduration = 0.0;
    for(const auto pt : pts)
    {
      initTotalduration += pt[2];
    }
    piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),2);
    timePerPiece = initTotalduration / piece_nums;
    ego_piece_dur_vec.resize(piece_nums);
    ego_piece_dur_vec.setConstant(timePerPiece);
    duration_container[i] = timePerPiece * piece_nums /** 1.2*/;
    ego_innerPs.resize(2, piece_nums-1);
    std::vector<Eigen::Vector3d> statelist;
    double res_time = 0;
    for(int i = 0; i < piece_nums; i++ )
    {
      int resolution;
      if(i==0||i==piece_nums-1)
      {
        resolution = dense_traj_res_;
      }
      else
      {
        resolution = traj_res_;
      }
      for(int k = 0; k <= resolution; k++)
      {
        // double t = basetime+res_time + 1.0*k/resolution*ego_piece_dur_vec[i];
        // Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
        double t = res_time + 1.0 * k / resolution * ego_piece_dur_vec[i];
        Eigen::Vector3d pos = kino_path_finder_->CalculateInitPos(t, kino_traj.singul);
        statelist.push_back(pos);
        if(k==resolution && i!=piece_nums-1)
        {
          ego_innerPs.col(i) = pos.head(2);
        }
      }
      res_time += ego_piece_dur_vec[i];
    }
    std::cout<<"s: "<<kino_traj.singul<<"\n";
    
    
    //double tm1 = apollo::cyber::Time::Now().ToNanosecond() / 1.0e9;;
    auto corridor_t1 = std::chrono::high_resolution_clock::now();
    getRectangleConst(statelist, buf_msg);
    sfc_container.push_back(hPolys_);
    display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
    auto corridor_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> corridor_ms = corridor_t2 - corridor_t1;
    std::cout << "corridor time: " << corridor_ms.count() << " ms" << std::endl;
    
    waypoints_container.push_back(ego_innerPs);
    iniState_container.push_back(kino_traj.start_state);
    finState_container.push_back(kino_traj.final_state);
    basetime += initTotalduration;

  }

  std::cout<<"try to optimize!\n";

  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container,
                                                        waypoints_container, duration_container,
                                                        sfc_container, singul_container, worldtime, 0.0);

  std::cout<<"optimize ended!\n";

  if (flag_success)
  {
    traj_container_.clearSingul();
    std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
    for(unsigned int i = 0; i < kino_trajs_.size(); i++)
    {
      traj_container_.addSingulTraj( (*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]), worldtime, car_id_); // todo time
      std::cout<<"init duration: "<<duration_container[i]<<std::endl;
      std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
      std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
      std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost()<<std::endl;
      worldtime = traj_container_.singul_traj.back().end_time;
    }
    return true;
  }
  else
  {
    cerr<<"[PolyTrajManager] Planning fails! "<<endl;
    return false;
    // return kWrongStatus;
  }
}

void TrajPlanner::broadcastTraj2SwarmBridge(){
    return;
}
/*
void TrajPlanner::broadcastTraj2SwarmBridge()
{
    //ros::Time t0 = ros::Time().fromSec(start_time_);
    double t0 = apollo::cyber::Time::ToSecond(start_time_);
    //swarm_bridge::Trajectory traj_msg;
    for(int i = 0; i < (int)kino_trajs_.size(); i++)
    {
        swarm_bridge::SingleMinco sm;
        Eigen::MatrixXd poses = traj_container_.singul_traj[i].traj.getPositions();
        Eigen::VectorXd ts = traj_container_.singul_traj[i].traj.getDurations();
        int direction = traj_container_.singul_traj[i].traj.getDirection();
        // Eigen::MatrixXd init = iniState_container[i];
        Eigen::MatrixXd init = kino_trajs_.at(i).start_state;
        // Eigen::MatrixXd fina = finState_container[i];
        Eigen::MatrixXd fina = kino_trajs_.at(i).final_state;
        sm.head_x.x = init.row(0)[0];
        sm.head_x.y = init.row(0)[1];
        sm.head_x.z = init.row(0)[2];
        sm.head_y.x = init.row(1)[0];
        sm.head_y.y = init.row(1)[1];
        sm.head_y.z = init.row(1)[2];

        sm.tail_x.x = fina.row(0)[0];
        sm.tail_x.y = fina.row(0)[1];
        sm.tail_x.z = fina.row(0)[2];
        sm.tail_y.x = fina.row(1)[0];
        sm.tail_y.y = fina.row(1)[1];
        sm.tail_y.z = fina.row(1)[2]; 

        sm.reverse = kino_trajs_.at(i).singul == 1?false:true;
        sm.start_time = t0;
        t0 = t0 + ros::Duration().fromSec(ts.sum());
        for (int i=0; i<poses.cols(); i++)
        {
            geometry_msgs::Point temp;
            temp.x = poses(0, i);
            temp.y = poses(1, i);
            temp.z = 0;
            sm.pos_pts.push_back(temp);
        }
        for (int i=0; i<ts.size(); i++)
        {
            sm.t_pts.push_back(ts(i));
        }
        traj_msg.minco_path.trajs.push_back(sm);       
    }
    //traj_msg.traj_type = 1;
    //traj_msg.car_id = car_id_;
    //TrajPathPub_.publish(traj_msg);
}
*/

void TrajPlanner::getRectangleConst(std::vector<Eigen::Vector3d> statelist, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{
    hPolys_.clear();
    double resolution = resolution_;
    double step = resolution * 1.0;
    double limitBound = 10.0;

    for(const auto state : statelist)
    {
        Eigen::Vector2d pos = state.head(2);
        double yaw = state(2);
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw),
                 sin(yaw), cos(yaw);
        
        Eigen::Vector4d distance2center;
        distance2center << car_width_ / 2.0, car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0, car_length_ / 2.0 - car_d_cr_;
        Eigen::Vector4d have_stopped_expanding;
        have_stopped_expanding << 1.0, 1.0, 1.0, 1.0;

        Eigen::MatrixXd hPoly;
        hPoly.resize(4, 4);
        while(have_stopped_expanding.norm() != 0)
        {
            for(int i = 0; i < 4; i++)
            {
            
                Eigen::Vector2d point1, point2, newpoint1, newpoint2; 
                bool isocc = false;               
                switch(i)
                {
                case 0: // dy
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0) + step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0) + step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 1: // dx
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
                    point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, distance2center(0));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(distance2center(1) + step, -distance2center(2));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 - car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 2: // -dy
                    point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2) - step);
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2) - step);

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_width_ / 2.0 > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                case 3: // -dx
                    point1 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
                    point2 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
                    newpoint1 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, -distance2center(2));
                    newpoint2 = pos + ego_R * Eigen::Vector2d(-distance2center(3) - step, distance2center(0));

                    kino_path_finder_->checkCollisionUsingLine(point1, newpoint1, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint1, newpoint2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    kino_path_finder_->checkCollisionUsingLine(newpoint2, point2, isocc, buf_msg);
                    if(isocc)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }

                    distance2center(i) += step;
                    if(distance2center(i) - car_length_ / 2.0 + car_d_cr_ > limitBound)
                    {
                        have_stopped_expanding(i) = 0.0;
                        break;
                    }
                    break;

                }
            }
        }
        Eigen::Vector2d point1, norm1;
        point1 = pos + ego_R * Eigen::Vector2d(distance2center(1), distance2center(0));
        norm1 << -sin(yaw), cos(yaw);
        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;

        Eigen::Vector2d point2, norm2;
        point2 = pos + ego_R * Eigen::Vector2d(distance2center(1), -distance2center(2));
        norm2 << cos(yaw), sin(yaw);
        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;

        Eigen::Vector2d point3, norm3;
        point3 = pos + ego_R * Eigen::Vector2d(-distance2center(3), -distance2center(2));
        norm3 << sin(yaw), -cos(yaw);
        hPoly.col(2).head<2>() = norm3;
        hPoly.col(2).tail<2>() = point3;

        Eigen::Vector2d point4, norm4;
        point4 = pos + ego_R * Eigen::Vector2d(-distance2center(3), distance2center(0));
        norm4 << -cos(yaw), -sin(yaw);
        hPoly.col(3).head<2>() = norm4;
        hPoly.col(3).tail<2>() = point4;   

        hPolys_.push_back(hPoly);     

    }
}

void TrajPlanner::setMapFree(double& time_now)
{
    int num_have_received_trajs = 0;
    for(int car = 0; car < cars_num_; car++)
    {
        if(have_received_trajs_[car])
        {
            num_have_received_trajs++;
        }
    }
    if(num_have_received_trajs < cars_num_)
        return;
    std::vector<Eigen::Vector2d> pos_vec;
    std::vector<double> yaw_vec;

    for(int car = 0; car < cars_num_; car++)
    {
        
        int i_th_segment = swarm_traj_container_[car].locateSingulId(time_now);
        double i_th_segment_starttime = swarm_traj_container_[car].singul_traj[i_th_segment].start_time;
        double i_th_segment_endtime = swarm_traj_container_[car].singul_traj[i_th_segment].end_time;
        // if(time_now < i_th_segment_starttime || time_now > i_th_segment_endtime)
        //     return;
        if(time_now > i_th_segment_endtime)
            return;

        Eigen::Vector2d pos;
        double yaw;
        if(time_now < i_th_segment_starttime)
        {
            int i_th_last_segment = swarm_last_traj_container_[car].locateSingulId(time_now);
            double i_th_segment_last_start_time = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].start_time;
            double i_th_segment_last_end_time = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].end_time;
            if(time_now < i_th_segment_last_start_time || time_now > i_th_segment_last_end_time)
                return;
            
            pos = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].traj.getPos(time_now - i_th_segment_last_start_time);
            yaw = swarm_last_traj_container_[car].singul_traj[i_th_last_segment].traj.getAngle(time_now - i_th_segment_last_start_time);

            pos_vec.push_back(pos);
            yaw_vec.push_back(yaw);
        }
        else
        {
            pos = swarm_traj_container_[car].singul_traj[i_th_segment].traj.getPos(time_now - i_th_segment_starttime);
            yaw = swarm_traj_container_[car].singul_traj[i_th_segment].traj.getAngle(time_now - i_th_segment_starttime);

            pos_vec.push_back(pos);
            yaw_vec.push_back(yaw);            
        }

    }

    kino_path_finder_->setFreeSpaces(pos_vec, yaw_vec);
}
