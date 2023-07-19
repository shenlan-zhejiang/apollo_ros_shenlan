#include "modules/shenlan/minco/minco_component.h"

namespace apollo {
namespace shenlan {

bool MincoShenlanComponent::Init() 
{
  ACHECK(GetProtoConfig(&shenlan_conf)) << "Unable to load shenlan conf file";

  RFSM.init(shenlan_conf);
  RFSM.mapping_ptr_ = std::make_shared<apollo::shenlan::MappingProcess>();
  RFSM.mapping_ptr_->init(shenlan_conf);
  RFSM.planner_ptr_ = std::make_shared<TrajPlanner>();
  RFSM.planner_ptr_->setMap(RFSM.mapping_ptr_);
  RFSM.planner_ptr_->init(shenlan_conf);
  //std::cout << "0000000000" << std::endl;

//   cyber::TimerOption opt1;
//   opt1.oneshot = false;
//   opt1.callback = boost::bind(&MincoShenlanComponent::execFSMCallback, this);
//   opt1.period = 20;
//   exec_timer_ = std::make_shared<cyber::Timer>();
//   exec_timer_->SetTimerOption(opt1);
//   exec_timer_->Start();
  //std::cout << "1111" << std::endl;

//   cyber::TimerOption opt2;
//   opt2.oneshot = false;
//   opt2.callback = boost::bind(&MincoShenlanComponent::checkCollisionCallback, this);
//   opt2.period = 100;
//   safety_timer_ = std::make_shared<cyber::Timer>();
//   safety_timer_->SetTimerOption(opt2);
//   safety_timer_->Start();
//   std::cout << "2222" << std::endl;

  pc_writer_ = node_->CreateWriter<drivers::PointCloud>("/apollo/shenlan/mapping/pc_transformed");
  map_writer_ = node_->CreateWriter<drivers::PointCloud>("/apollo/shenlan/mapping/gird_map");
  traj_writer_ = node_->CreateWriter<apollo::shenlan::mpc::Trajectory>("/apollo/shenlan/minco/mpc_trajectory");
  //adc_writer_ = node_->CreateWriter<apollo::planning::ADCTrajectory>("/apollo/shenlan/minco/ADCTrajectory");
  adc_writer_ = node_->CreateWriter<apollo::planning::ADCTrajectory>("/apollo/planning");
  
  last_seq = -1;

  return true;
}

// bool MincoShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg_, const std::shared_ptr<drivers::PointCloud> &pcl_msg) 
// {
//   // std::cout << "************Proc************" << std::endl;

//   std::cout << pcl_msg->header().sequence_num() << " odom " << odom_msg_->header().sequence_num() << std::endl;
//   if (odom_msg_->header().sequence_num() == 1613777) {
//     odom_msg = std::make_shared<localization::LocalizationEstimate>(*odom_msg_); //for debug, using odom_msg
//     return true;
//   }
//   if (pcl_msg->header().sequence_num() != 3472) {
//     return true;
//   }

//   if (last_seq == (int)(pcl_msg->header().sequence_num())) {
//     return true;
//   }


bool MincoShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<drivers::PointCloud> &pcl_msg) 
{

  //std::cout << "0000" << std::endl;
  CreateMapCallback(odom_msg, pcl_msg);
  //std::cout << "1111" << std::endl;
  OdomCallback(odom_msg);
  //std::cout << "2222" << std::endl;
  execFSMCallback();
  //std::cout << "3333" << std::endl;
  checkCollisionCallback();
  //std::cout << "4444" << std::endl;
  //RFSM.print();
  //exit(0);
  return true;
}


void MincoShenlanComponent::CreateMapCallback(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<drivers::PointCloud> &pcl_msg)
{

  last_seq = (int)(pcl_msg->header().sequence_num());
  //std::cout << "handle new pcl" << std::endl;
  //std::cout << pcl_msg->header().sequence_num() << " odom " << odom_msg->header().sequence_num() << std::endl;

  RFSM.mapping_ptr_->have_odom_ = true;
  RFSM.mapping_ptr_->local_map_valid_ = true;

  RFSM.mapping_ptr_->curr_posi_[0] = odom_msg->pose().position().x();// - 587061;
  RFSM.mapping_ptr_->curr_posi_[1] = odom_msg->pose().position().y();// - 4141628;
  RFSM.mapping_ptr_->curr_posi_[2] = odom_msg->pose().position().z();

  RFSM.mapping_ptr_->curr_q_.w() = odom_msg->pose().orientation().qw();
  RFSM.mapping_ptr_->curr_q_.x() = odom_msg->pose().orientation().qx();
  RFSM.mapping_ptr_->curr_q_.y() = odom_msg->pose().orientation().qy();
  RFSM.mapping_ptr_->curr_q_.z() = odom_msg->pose().orientation().qz();

  RFSM.mapping_ptr_->curr_twist_[0] = odom_msg->pose().linear_velocity().x();
  RFSM.mapping_ptr_->curr_twist_[1] = odom_msg->pose().linear_velocity().y();
  RFSM.mapping_ptr_->curr_twist_[2] = odom_msg->pose().linear_velocity().z();

  //Eigen::Vector3d Position_XYZ(odom_msg->pose().position().x() - 587061, odom_msg->pose().position().y() - 4141628, odom_msg->pose().position().z());
  Eigen::Vector3d Position_XYZ(odom_msg->pose().position().x(), odom_msg->pose().position().y(), odom_msg->pose().position().z());
  Eigen::Quaterniond quaternion(odom_msg->pose().orientation().qw(), odom_msg->pose().orientation().qx(), 
                                odom_msg->pose().orientation().qy(), odom_msg->pose().orientation().qz());
  Eigen::Matrix3d Rotation_matrix;
  
  // Rotation_matrix = quaternion.toRotationMatrix();

  Eigen::Quaterniond quaternion_(0.7071, 0, 0, 0.7071);
  Rotation_matrix = quaternion.toRotationMatrix() * quaternion_.toRotationMatrix();

  RFSM.mapping_ptr_->center_position_ = Position_XYZ + Rotation_matrix * RFSM.mapping_ptr_->lidar2car_;

  std::shared_ptr<drivers::PointCloud> laserCloudTransformed = std::make_shared<drivers::PointCloud>();

  for(auto iter : pcl_msg->point())
  {
      Eigen::Vector3d LaserCloudIn_XYZ(iter.x(), iter.y(), iter.z());
      Eigen::Vector3d LaserCloudTransformed_XYZ = Rotation_matrix * LaserCloudIn_XYZ + RFSM.mapping_ptr_->center_position_;

      Eigen::Vector3d pc_position = LaserCloudTransformed_XYZ - RFSM.mapping_ptr_->center_position_;
      if(pc_position(2) < RFSM.mapping_ptr_->obs_low_ || pc_position(2) > RFSM.mapping_ptr_->obs_high_) {
          continue;
      }
      if(pc_position(0) * pc_position(0) + pc_position(1) * pc_position(1) < RFSM.mapping_ptr_->obs_circle_) {
          continue;
      }

      apollo::drivers::PointXYZIT *point = laserCloudTransformed->add_point();
      point->set_x(LaserCloudTransformed_XYZ(0));
      point->set_y(LaserCloudTransformed_XYZ(1));
      point->set_z(LaserCloudTransformed_XYZ(2));
      //point->set_intensity(iter.intensity());
      //point->set_timestamp(iter.timestamp() * 1e9);
  }

  RFSM.mapping_ptr_->number_of_points_ = laserCloudTransformed->point().size();

  // set header
  std::shared_ptr<drivers::PointCloud> msg = laserCloudTransformed;
  const auto timestamp = pcl_msg->header().timestamp_sec();
  msg->set_height(1);
  msg->set_width(msg->point_size() / msg->height());
  msg->set_is_dense(false);
  msg->mutable_header()->set_sequence_num(pcl_msg->header().sequence_num());
  msg->mutable_header()->set_frame_id("map_shenlan");
  msg->mutable_header()->set_timestamp_sec(timestamp);
  msg->mutable_header()->set_lidar_timestamp(timestamp * 1e9);
  msg->set_measurement_time(timestamp);

  pc_writer_->Write(msg);

  RFSM.mapping_ptr_->local_range_min_ = RFSM.mapping_ptr_->center_position_ - RFSM.mapping_ptr_->sensor_range_;
  RFSM.mapping_ptr_->local_range_max_ = RFSM.mapping_ptr_->center_position_ + RFSM.mapping_ptr_->sensor_range_;
  RFSM.mapping_ptr_->raycastProcess(RFSM.mapping_ptr_->center_position_, laserCloudTransformed);

  ///*
  std::shared_ptr<drivers::PointCloud> map_ = std::make_shared<drivers::PointCloud>();
  globalOccPc(map_);
  map_writer_->Write(map_);
  //*/
}

void MincoShenlanComponent::globalOccPc(const std::shared_ptr<drivers::PointCloud> &msg)
{
    // for (int x = 0; x < RFSM.mapping_ptr_->global_map_size_[0]; ++x)
    // {
    //     for (int y = 0; y < RFSM.mapping_ptr_->global_map_size_[1]; ++y)
    //     {
    //         if (RFSM.mapping_ptr_->occupancy_buffer_2d_.at(y * RFSM.mapping_ptr_->global_map_size_[0] + x) > 0.5)
    //         {
    //             Eigen::Vector2i idx(x, y);
    //             Eigen::Vector2d pos;
    //             RFSM.mapping_ptr_->indexToPos2d(idx, pos);
    //             apollo::drivers::PointXYZIT *point = msg->add_point();
    //             point->set_x(pos[0]);
    //             point->set_y(pos[1]);
    //             point->set_z(0);
    //         }
    //     }
    // }

    for (int x = 0; x < RFSM.mapping_ptr_->global_map_size_[0]; ++x)
      for (int y = 0; y < RFSM.mapping_ptr_->global_map_size_[1]; ++y)
          for (int z = 0; z < RFSM.mapping_ptr_->global_map_size_[2]; ++z)
          {
              if (RFSM.mapping_ptr_->occupancy_buffer_[x * RFSM.mapping_ptr_->grid_size_y_multiply_z_ + y * RFSM.mapping_ptr_->global_map_size_(2) + z] > RFSM.mapping_ptr_->min_occupancy_log_)
              {
                  Eigen::Vector3i idx(x, y, z);
                  Eigen::Vector3d pos;
                  RFSM.mapping_ptr_->indexToPos(idx, pos);
                  apollo::drivers::PointXYZIT *point = msg->add_point();
                  point->set_x(pos[0]);
                  point->set_y(pos[1]);
                  point->set_z(pos[2]);
              }
          }

    auto timestamp = apollo::cyber::Time::Now().ToSecond();
    msg->set_height(1);
    msg->set_width(msg->point_size() / msg->height());
    msg->set_is_dense(false);
    msg->mutable_header()->set_sequence_num(seq_num_map);
    msg->mutable_header()->set_frame_id("map_shenlan");
    msg->mutable_header()->set_timestamp_sec(timestamp);
    msg->mutable_header()->set_lidar_timestamp(timestamp * 1e9);
    msg->set_measurement_time(timestamp);
    seq_num_map += 1;
}

void MincoShenlanComponent::checkCollisionCallback()
{
    //std::cout << "checkcolli:" << apollo::cyber::Time::Now().ToNanosecond() << std::endl;
    //std::cout << "checkcolli:" << last_seq << std::endl;
    //double time_now = ros::Time::now().toSec();
    double time_now = apollo::cyber::Time::Now().ToSecond();
    // set other cars' position of map is free
    RFSM.planner_ptr_->setMapFree(time_now);

    // check collision with static obstacles
    if(RFSM.exec_state_ == RFSM.EXEC_TRAJ)
        RFSM.collision_with_obs_ = RFSM.planner_ptr_->checkCollisionWithObs(time_now);

    // check collision with surround cars
    if(RFSM.exec_state_ == RFSM.EXEC_TRAJ)
        RFSM.collision_with_othercars_ = RFSM.planner_ptr_->checkCollisionWithOtherCars(time_now);
}

void MincoShenlanComponent::ParkingCallback(const std::shared_ptr<apollo::localization::Pose> &msg)
{
    return;
    std::cout << "Triggered parking mode!" << std::endl;
    // end_pt_ << msg.pose.position.x, msg.pose.position.y, 
    //            tf::getYaw(msg.pose.orientation), 1.0e-2;
    RFSM.end_pt_ << RFSM.target_x_, RFSM.target_y_, 
               RFSM.target_yaw_, 1.0e-2;
    std::cout << "end_pt: " << RFSM.end_pt_.transpose() << std::endl;
    
    RFSM.have_target_ = true;
    // Eigen::Vector4d init_state;  init_state << cur_pos_, cur_yaw_, cur_vel_;
    // planner_ptr_->setInitState(init_state);
    // planner_ptr_->setParkingEnd(end_pt_);
    // planner_ptr_->getKinoPath(end_pt_);
    // planner_ptr_->displayKinoPath(planner_ptr_->display_kino_path());
    // planner_ptr_->RunMINCOParking();
    // planner_ptr_->displayPolyH(planner_ptr_->display_hPolys());
    // planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());
}

void MincoShenlanComponent::OdomCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg)
{
    //std::cout << "+++++++++++++odomcb++++++++++++++++ " << std::endl;

    Eigen::Vector3d center_pos(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    //std::cout << "center_pos: " << center_pos << std::endl;

    Eigen::Vector3d pos2center(-RFSM.car_d_cr_, 0, 0);
    //std::cout << "pos2center: " << pos2center << std::endl;

    Eigen::Quaterniond quaternion(msg->pose().orientation().qw(), msg->pose().orientation().qx(), 
                                  msg->pose().orientation().qy(), msg->pose().orientation().qz());

    Eigen::Quaterniond quaternion_(0.7071, 0, 0, 0.7071);

    Eigen::Matrix3d R = quaternion.toRotationMatrix() * quaternion_.toRotationMatrix();
    Eigen::Vector3d pos = center_pos + R * pos2center;
    
    RFSM.cur_pos_ = pos.head(2);

    double vx = 0;
    vx = msg->pose().linear_velocity().x();    
    double vy = 0;
    vy = msg->pose().linear_velocity().y();
    RFSM.cur_vel_ = std::sqrt(vx * vx + vy * vy);
    //std::cout << "RFSM.cur_vel_: " << RFSM.cur_vel_ << std::endl;
   
    //Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(0,1,2);
    RFSM.cur_yaw_ = msg->pose().heading();//eulerAngle(2);
    //std::cout << "RFSM.cur_yaw_: " << RFSM.cur_yaw_ << std::endl;


/*
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(cur_pos_(0), cur_pos_(1), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, cur_yaw_);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "map", "car_"+to_string(car_id_)+"_pos"));
*/
}

void MincoShenlanComponent::execFSMCallback()
{
    //std::cout << "execFSM:" << apollo::cyber::Time::Now().ToNanosecond() << std::endl;
    //exec_timer_->Stop();
    int ret = RFSM.execFSM();
    if (ret == 1) {
      auto traj_msg = std::make_shared<apollo::shenlan::mpc::Trajectory>();
      calcTraj2Controller(traj_msg);
      traj_writer_->Write(traj_msg);

      auto adc_msg = std::make_shared<apollo::planning::ADCTrajectory>();
      calcMinco2ADC(adc_msg);
      adc_writer_->Write(adc_msg);
    }
    //exec_timer_->Start();
}

void MincoShenlanComponent::calcTraj2Controller(const std::shared_ptr<apollo::shenlan::mpc::Trajectory> &traj_msg)
{
    double t0 = RFSM.planner_ptr_->start_time_;
    apollo::shenlan::mpc::MincoTraj minco;
    for(int i = 0; i < (int)RFSM.planner_ptr_->kino_trajs_.size(); i++)
    {
        auto sm = minco.add_trajs();
        Eigen::MatrixXd poses = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getPositions();
        Eigen::VectorXd ts = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getDurations();
        //int direction = traj_container_.singul_traj[i].traj.getDirection();
        // Eigen::MatrixXd init = iniState_container[i];
        Eigen::MatrixXd init = RFSM.planner_ptr_->kino_trajs_.at(i).start_state;
        // Eigen::MatrixXd fina = finState_container[i];
        Eigen::MatrixXd fina = RFSM.planner_ptr_->kino_trajs_.at(i).final_state;
        sm->mutable_head_x()->set_x(init.row(0)[0]);
        sm->mutable_head_x()->set_y(init.row(0)[1]);
        sm->mutable_head_x()->set_z(init.row(0)[2]);
        sm->mutable_head_y()->set_x(init.row(1)[0]);
        sm->mutable_head_y()->set_y(init.row(1)[1]);
        sm->mutable_head_y()->set_z(init.row(1)[2]);

        sm->mutable_tail_x()->set_x(fina.row(0)[0]);
        sm->mutable_tail_x()->set_y(fina.row(0)[1]);
        sm->mutable_tail_x()->set_z(fina.row(0)[2]);
        sm->mutable_tail_y()->set_x(fina.row(1)[0]);
        sm->mutable_tail_y()->set_y(fina.row(1)[1]);
        sm->mutable_tail_y()->set_z(fina.row(1)[2]);

        sm->set_reverse(RFSM.planner_ptr_->kino_trajs_.at(i).singul == 1?false:true);
        sm->set_start_time(t0);
        //t0 = t0 + apollo::cyber::Duration().fromSec(ts.sum()); //ros::Duration().fromSec(ts.sum()); 
        t0 = t0 + ts.sum();
        for (int i=0; i<poses.cols(); i++)
        {
            auto temp = sm->add_pos_pts();
            //geometry_msgs::Point temp;
            temp->set_x(poses(0, i));
            temp->set_y(poses(1, i));
            temp->set_z(0);
        }
        for (int i=0; i<ts.size(); i++)
        {
            sm->add_t_pts(ts(i));
        }
    }
    traj_msg->mutable_minco_path()->CopyFrom(minco);
    traj_msg->set_traj_type(1);
}


void MincoShenlanComponent::calcMinco2ADC(const std::shared_ptr<apollo::planning::ADCTrajectory> &traj_msg)
{
    auto timestamp = apollo::cyber::Time::Now().ToSecond();
    traj_msg->mutable_header()->set_timestamp_sec(timestamp);
    traj_msg->mutable_header()->set_module_name("planning");
    traj_msg->mutable_header()->set_sequence_num(seq_num_adc);
    seq_num_adc += 1;
    // traj_msg->set_total_path_length(0);
    // traj_msg->set_total_path_time(0);
    traj_msg->set_gear(apollo::canbus::Chassis::GEAR_DRIVE);
    traj_msg->mutable_engage_advice()->set_advice(apollo::common::EngageAdvice::READY_TO_ENGAGE);
    traj_msg->mutable_estop()->set_is_estop(0);

    double t0 = RFSM.planner_ptr_->start_time_;
    for(int i = 0; i < (int)RFSM.planner_ptr_->kino_trajs_.size(); i++)
    {
        Eigen::MatrixXd positions = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getPositions();
        Eigen::VectorXd durations = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getDurations();
        double totalduration = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getTotalDuration();
        traj_msg->set_total_path_time(totalduration);
        t0 = t0 + durations.sum();

        double angle = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getAngle(durations(i));
        double vel = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getVel(durations(i));
        double acc = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getAcc(durations(i));
        double curv = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getCurv(durations(i));
        // double curv_ = 0;
        // double _curv = 0;
        // _curv = curv - curv_;

        //std::cout << "t0" << t0 << std::endl;
        //std::cout << poses.cols() << "|" << ts.size() << std::endl;
        //auto rt = t0;

        // Eigen::Vector2d vel_ = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getdSigma(durations.sum());
        // double vel = vel_.norm();

        for (int j=0; j<positions.cols(); j++)
        {
            auto traj_point = traj_msg->add_trajectory_point();
            apollo::common::PathPoint point;
            //geometry_msgs::Point temp;

            point.set_x(positions(0, j));
            point.set_y(positions(1, j));
            
            // double x = positions(0, j);
            // double x_ = 0;
            // double y = positions(1, j);
            // double y_ = 0;
            // s = std::sqrt((x - x_) * (x - x_) + (y - y_) * (y - y_));

            point.set_z(0);

            point.set_theta(angle); //localization.pose.heading = The heading is zero when the car is facing East and positive when facing North.

            point.set_kappa(curv); //curvature = tan(radians(carsteer / 100 * degrees(max_steer_)) / carsteer_ratio) / carwheel_base = tan()

            //double curvature_change_rate;
            //curvature_change_rate = (curv - curv_)/(vel * 0.01);
            //point.set_dkappa(curvature_change_rate); //curvature_change_rate = (curvature - carcurvature) / (carspeed * 0.01)

            // double s = 0;
            // s += vel * durations(j);
            // point.set_s(s); //cars += carspeed * 0.01
            // traj_msg->set_total_path_length(s);

            traj_point->mutable_path_point()->CopyFrom(point);

            traj_point->set_v(vel);
            traj_point->set_a(acc); //linear_acceleration_vrf = Linear acceleration of the VRP in the vehicle reference frame. Right/forward/up in meters per square second.
            //traj_point->set_relative_time(durations.sum()); //relative_time = data['time'][i] - data['time'][closestpoint] - now + starttime
            //if (i < ts.size())
            //    rt += ts(i); 
        }
        //curv_ = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getCurv(t0);
        break; // there is only one traj in ADCtrajectory.
    }

    //this snippet comes from displayMincoTraj in traj_manager.h
    /*
    auto minco_traj = RFSM.planner_ptr_->trajectory();
    for (unsigned int i = 0; i < minco_traj->size(); ++i)
    {
        double total_duration = minco_traj->at(i).duration;
        for (double t = 0; t <= total_duration; t += 0.01)
        {
            Eigen::Vector2d pt = minco_traj->at(i).traj.getPos(t);
            apollo::common::PathPoint *point = traj_msg->add_path_point();
            point->set_x(pt[0]);
            point->set_y(pt[1]);
            point->set_z(0.2);
        }
        break; // there is only one path_point in ADCtrajectory.
    }
    */
}

}  // namespace shenlan
}  // namespace apollo