#include "modules/shenlan/minco_mapping/minco_mapping_component.h"

namespace apollo {
namespace shenlan {

bool MincoMappingShenlanComponent::Init() 
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
//   opt1.callback = boost::bind(&MincoMappingShenlanComponent::execFSMCallback, this);
//   opt1.period = 20;
//   exec_timer_ = std::make_shared<cyber::Timer>();
//   exec_timer_->SetTimerOption(opt1);
//   exec_timer_->Start();
//   std::cout << "1111" << std::endl;

//   cyber::TimerOption opt2;
//   opt2.oneshot = false;
//   opt2.callback = boost::bind(&MincoMappingShenlanComponent::checkCollisionCallback, this);
//   opt2.period = 100;
//   safety_timer_ = std::make_shared<cyber::Timer>();
//   safety_timer_->SetTimerOption(opt2);
//   safety_timer_->Start();
//   std::cout << "2222" << std::endl;

  pc_writer_ = node_->CreateWriter<drivers::PointCloud>("/apollo/shenlan/mapping/pointcloud");
  map_writer_ = node_->CreateWriter<drivers::PointCloud>("/apollo/shenlan/mapping/gird_map");
  kino_writer_ = node_->CreateWriter<apollo::shenlan::NavPath>("/apollo/shenlan/minco/kino_traj");
  minco_writer_ = node_->CreateWriter<apollo::shenlan::NavPath>("/apollo/shenlan/minco/minco_traj");
  traj_writer_ = node_->CreateWriter<apollo::shenlan::mpc::Trajectory>("/apollo/shenlan/minco/mpc_trajectory");
  adc_writer_ = node_->CreateWriter<apollo::planning::ADCTrajectory>("/apollo/planning");

  last_seq = -1;

  return true;
}

// bool MincoMappingShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg_, const std::shared_ptr<drivers::PointCloud> &pcl_msg) 
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


bool MincoMappingShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<drivers::PointCloud> &pcl_msg) 
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


void MincoMappingShenlanComponent::CreateMapCallback(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<drivers::PointCloud> &pcl_msg)
{
  // planner make hz at 1/50ms
  last_seq = (int)(pcl_msg->header().sequence_num());
  
  //std::cout << "handle new pcl" << std::endl;
  //std::cout << pcl_msg->header().sequence_num() << " odom " << odom_msg->header().sequence_num() << std::endl;

  RFSM.mapping_ptr_->have_odom_ = true;
  RFSM.mapping_ptr_->local_map_valid_ = true;

  RFSM.mapping_ptr_->curr_posi_[0] = odom_msg->pose().position().x();
  RFSM.mapping_ptr_->curr_posi_[1] = odom_msg->pose().position().y();
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

  Eigen::Quaterniond quaternion_(RFSM.mapping_ptr_->lidar2imu_qw_, RFSM.mapping_ptr_->lidar2imu_qx_, RFSM.mapping_ptr_->lidar2imu_qy_, RFSM.mapping_ptr_->lidar2imu_qz_);
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

  std::shared_ptr<drivers::PointCloud> map_ = std::make_shared<drivers::PointCloud>();
  globalOccPc(map_);
  map_writer_->Write(map_);
}

void MincoMappingShenlanComponent::globalOccPc(const std::shared_ptr<drivers::PointCloud> &msg)
{   
    // // 2D GRIDMAP VISUALIZATION 
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

    // 3D GRIDMAP VISUALIZATION 
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

void MincoMappingShenlanComponent::OdomCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg)
{
    // std::cout << "+++++++++++++odomcb++++++++++++++++ " << std::endl;

    Eigen::Vector3d center_pos(msg->pose().position().x(), msg->pose().position().y(), msg->pose().position().z());
    // std::cout << "center_pos: " << center_pos << std::endl;

    // @ROCY: rear axis center in apollo is y, in ros is x
    Eigen::Vector3d pos2center(-RFSM.car_d_cr_x_, -RFSM.car_d_cr_y_, 0);
    // std::cout << "pos2center: " << pos2center << std::endl;

    Eigen::Quaterniond quaternion(msg->pose().orientation().qw(), msg->pose().orientation().qx(), 
                                  msg->pose().orientation().qy(), msg->pose().orientation().qz());

    Eigen::Quaterniond quaternion_(RFSM.imu2car_qw_, RFSM.imu2car_qx_, RFSM.imu2car_qy_, RFSM.imu2car_qz_);

    Eigen::Matrix3d R = quaternion.toRotationMatrix() * quaternion_.toRotationMatrix();

    Eigen::Vector3d pos = center_pos + R * pos2center;
    
    RFSM.cur_pos_ = pos.head(2);

    double cur_vx_ = msg->pose().linear_velocity().x();
    double cur_vy_ = msg->pose().linear_velocity().y();
    RFSM.cur_vel_ = std::sqrt(cur_vx_ * cur_vx_ + cur_vy_ * cur_vy_);
    // std::cout << "RFSM.cur_vel_: " << RFSM.cur_vel_ << std::endl;

    // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(0,1,2);
    RFSM.cur_yaw_ = msg->pose().heading();//eulerAngle(2);
    //std::cout << "RFSM.cur_yaw_: " << RFSM.cur_yaw_ << std::endl;

    // @ROCY: use for setInitStateAndInput(replan_start_time, replan_init_state, cur_yaw_, cur_vel_, start_pos_, start_vel_, start_acc_);
    Eigen::Vector3d linear_velocity(msg->pose().linear_velocity().x(), msg->pose().linear_velocity().y(), msg->pose().linear_velocity().z());
    Eigen::Vector3d linear_acceleration(msg->pose().linear_acceleration().x(), msg->pose().linear_acceleration().y(), msg->pose().linear_acceleration().z());
    RFSM.start_pos_ = center_pos.head(2);
    RFSM.start_vel_ = linear_velocity.head(2);
    RFSM.start_acc_ = linear_acceleration.head(2);

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

void MincoMappingShenlanComponent::execFSMCallback()
{
    // std::cout << "execFSM:" << apollo::cyber::Time::Now().ToNanosecond() << std::endl;
    // exec_timer_->Stop();
    int ret = RFSM.execFSM();
    if (ret == 1) {
      auto kino_msg = std::make_shared<apollo::shenlan::NavPath>();
      displayKinoPath(kino_msg);
      kino_writer_->Write(kino_msg);

      auto minco_msg = std::make_shared<apollo::shenlan::NavPath>();
      displayMincoTraj(minco_msg);
      minco_writer_->Write(minco_msg);

      auto traj_msg = std::make_shared<apollo::shenlan::mpc::Trajectory>();
      calcTraj2Controller(traj_msg);
      traj_writer_->Write(traj_msg);

      auto adc_msg = std::make_shared<apollo::planning::ADCTrajectory>();
      calcMinco2ADC(adc_msg);
      adc_writer_->Write(adc_msg);
    }
    // exec_timer_->Start();
}

void MincoMappingShenlanComponent::ParkingCallback(const std::shared_ptr<apollo::localization::Pose> &msg)
{
    return;
    std::cout << "Triggered parking mode!" << std::endl;
    // end_pt_ << msg.pose.position.x, msg.pose.position.y, 
    //            tf::getYaw(msg.pose.orientation), 1.0e-2;
    RFSM.end_pt_ << RFSM.target_x_, RFSM.target_y_, RFSM.target_yaw_, 1.0e-2;
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

void MincoMappingShenlanComponent::displayKinoPath(const std::shared_ptr<apollo::shenlan::NavPath> &kino_msg)
{
    apollo::common::Quaternion quaternion;
    quaternion.set_qw(1.0);
    quaternion.set_qx(0.0);
    quaternion.set_qy(0.0);
    quaternion.set_qz(0.0);
    apollo::common::PointENU position;
    //std::cout << "111111111111kino_size: " << RFSM.planner_ptr_->kino_trajs_.size() << std::endl;
    for (unsigned int i = 0; i < RFSM.planner_ptr_->kino_trajs_.size(); ++ i)
    {
        // std::cout << "111111111111kino_pts_size: " <<  RFSM.planner_ptr_->kino_trajs_.at(i).traj_pts.size() << std::endl;
        for (size_t k = 0; k <  RFSM.planner_ptr_->kino_trajs_.at(i).traj_pts.size(); k ++)
        {
            Eigen::Vector3d pt =  RFSM.planner_ptr_->kino_trajs_.at(i).traj_pts[k];
            //std::cout << "111111111111kino_pt: " << pt << std::endl;
            position.set_x(pt(0));
            position.set_y(pt(1));
            position.set_z(0.1);
            auto pose = kino_msg->add_pose();
            pose->mutable_position()->CopyFrom(position);
            pose->mutable_orientation()->CopyFrom(quaternion); 
        }
    }
    auto timestamp = apollo::cyber::Time::Now().ToSecond();
    kino_msg->mutable_header()->set_timestamp_sec(timestamp);
    kino_msg->mutable_header()->set_frame_id("map");
    kino_msg->mutable_header()->set_sequence_num(seq_num_adc);
    seq_num_kino += 1;
}

void MincoMappingShenlanComponent::displayMincoTraj(const std::shared_ptr<apollo::shenlan::NavPath> &minco_msg)
{
    apollo::common::Quaternion quaternion;
    quaternion.set_qw(1.0);
    quaternion.set_qx(0.0);
    quaternion.set_qy(0.0);
    quaternion.set_qz(0.0);
    apollo::common::PointENU position;
    //std::cout << "222222222222minco_size: " << RFSM.planner_ptr_->traj_container_.singul_traj.size() << std::endl;
    for (unsigned int i = 0; i < RFSM.planner_ptr_->traj_container_.singul_traj.size(); ++i)
    {
        // std::cout << "222222222222minco_duration: " << RFSM.planner_ptr_->traj_container_.singul_traj.at(i).duration << std::endl;
        // for (double t = 0; t <= RFSM.planner_ptr_->traj_container_.singul_traj.at(i).duration; t += 0.01)
        for (double t = 0; t <= RFSM.planner_ptr_->traj_container_.singul_traj.at(i).duration; t += 0.2)
        {
            Eigen::Vector2d pt = RFSM.planner_ptr_->traj_container_.singul_traj.at(i).traj.getPos(t);
            // std::cout << "222222222222minco_pt: " << pt << std::endl;
            position.set_x(pt(0));
            position.set_y(pt(1));
            position.set_z(0.1);
            auto pose = minco_msg->add_pose();
            pose->mutable_position()->CopyFrom(position);
            pose->mutable_orientation()->CopyFrom(quaternion); 
        }
    }
    auto timestamp = apollo::cyber::Time::Now().ToSecond();
    minco_msg->mutable_header()->set_timestamp_sec(timestamp);
    minco_msg->mutable_header()->set_frame_id("map");
    minco_msg->mutable_header()->set_sequence_num(seq_num_adc);
    seq_num_minco += 1;
}

void MincoMappingShenlanComponent::calcTraj2Controller(const std::shared_ptr<apollo::shenlan::mpc::Trajectory> &traj_msg)
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

void MincoMappingShenlanComponent::calcMinco2ADC(const std::shared_ptr<apollo::planning::ADCTrajectory> &traj_msg)
{
    // double t0 = RFSM.planner_ptr_->start_time_;
    double t1 = 0;// s = 0;

    auto timestamp = apollo::cyber::Time::Now().ToSecond();
    traj_msg->mutable_header()->set_timestamp_sec(timestamp);
    traj_msg->mutable_header()->set_module_name("planning");
    traj_msg->mutable_header()->set_sequence_num(seq_num_adc);
    
    for(int i = 0; i < (int)RFSM.planner_ptr_->kino_trajs_.size(); i++)
    {
        if (RFSM.planner_ptr_->kino_trajs_.at(i).singul == 1)
            traj_msg->set_gear(apollo::canbus::Chassis::GEAR_DRIVE);
        else
            traj_msg->set_gear(apollo::canbus::Chassis::GEAR_REVERSE);
    }
    
    traj_msg->mutable_engage_advice()->set_advice(apollo::common::EngageAdvice::READY_TO_ENGAGE);
    traj_msg->mutable_estop()->set_is_estop(0);
    
    // there is only one traj in ADCtrajectory
    for(int i = 0; i < (int)RFSM.planner_ptr_->kino_trajs_.size(); i++)
    {
        // @ROCY: get evert point position
        Eigen::MatrixXd positions = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getPositions();
        //std::cout << "positions: " << positions << std::endl;

        // @ROCY: get every point duration
        Eigen::VectorXd durations = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getDurations();
        //std::cout << "durations: " << durations << std::endl;

        // @ROCY: set_total_path_time
        // double totalduration = RFSM.planner_ptr_->traj_container_.singul_traj.at(i).duration;
        double totalduration = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getTotalDuration();
        traj_msg->set_total_path_time(totalduration);

        // @ROCY: set_total_path_length
        // traj_msg->set_total_path_length(0);
        
        // @ROCY: REMOVE LAST POINT FOR SAFETY
        for (int j = 0; j < positions.cols() -1; j ++)
        {
            // t0 = t0 + durations(j);
            t1 = t1 + durations(j);
            // std::cout << "t1: " << t1 << std::endl;
            apollo::common::PathPoint point;

            // @ROCY: set point positon
            point.set_x(positions(0, j));
            point.set_y(positions(1, j));
            point.set_z(0);

            // @ROCY: set_s = accumulated distance from beginning of the path
            // s = s + (positions(j) - positions(j+1)).abs() 
            // point.set_s(s)

            // @ROCY: set_theta = direction on the x-y plane
            double angle = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getAngle(t1);
            // std::cout << "angle: " << angle << std::endl;
            point.set_theta(angle); //localization.pose.heading = The heading is zero when the car is facing East and positive when facing North.
            
            // @ROCY: set_kappa = curvature on the x-y planning
            double curv = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getCurv(t1);
            // std::cout << "curv: " << curv << std::endl;
            point.set_kappa(curv); //curvature = tan(radians(carsteer / 100 * degrees(max_steer_)) / carsteer_ratio) / carwheel_base = tan()

            // @ROCY: set_ddkappa = derivative of kappa w.r.t s.(in planner, dkappa is v)
            // point.set_dkappa(dcurv); 

            // @ROCY: SET DDKAPPA = derivative of derivative of kappa w.r.t s.(in planner, dkappa is a)
            // point.set_ddkappa(ddcurv); 
          
            auto traj_point = traj_msg->add_trajectory_point();

            // @ROCY: set path point
            traj_point->mutable_path_point()->CopyFrom(point);

            // @ROCY: set_v = linear velocity
            double vel = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getVel(t1);
            // std::cout << "vel: " << vel << std::endl;
            traj_point->set_v(vel);

            // @ROCY: set_a = linear acceleration
            double acc = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj. getAcc(t1);
            // std::cout << "acc: " << acc << std::endl;
            traj_point->set_a(acc); 
 
            // @ROCY: set_relative_time = relative time from beginning of the trajectory
            traj_point->set_relative_time(t1);

            // @ROCY: set_da = longitudinal jerk
            // traj_point->set_da();

            // @ROCY: set_steer = The angle between vehicle front wheel and vehicle longitudinal axis
            Eigen::Vector2d vel_ = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getdSigma(t1);
            Eigen::Vector2d acc_ = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getddSigma(t1);
            int singul = RFSM.planner_ptr_->kino_trajs_.at(i).singul;
            Eigen::Matrix2d B;
            B << 0, -1,
                 1,  0;
            double steer = atan(singul * (acc_.transpose() * B * vel_)(0, 0) * RFSM.car_wheelbase_ / pow(vel_.norm(), 3));
            traj_point->set_steer(steer);
        }
        
        // there is only one traj in ADCtrajectory
        break; 
    }
    seq_num_adc += 1;
}

void MincoMappingShenlanComponent::checkCollisionCallback()
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

}  // namespace shenlan
}  // namespace apollo