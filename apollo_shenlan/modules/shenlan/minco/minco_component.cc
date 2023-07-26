#include "modules/shenlan/minco/minco_component.h"

namespace apollo {
namespace shenlan {

bool MincoShenlanComponent::Init() 
{
    std::cout << "MincoShenlanComponent::Init" << std::endl;
    ACHECK(GetProtoConfig(&shenlan_conf)) << "Unable to load shenlan conf file";

    RFSM.init(shenlan_conf);
    RFSM.planner_ptr_ = std::make_shared<TrajPlanner>();
    RFSM.planner_ptr_->init(shenlan_conf);
    // std::cout << "0000000000" << std::endl;

    // cyber::TimerOption opt1;
    // opt1.oneshot = false;
    // opt1.callback = boost::bind(&MincoShenlanComponent::execFSMCallback, this);
    // opt1.period = 20;
    // exec_timer_ = std::make_shared<cyber::Timer>();
    // exec_timer_->SetTimerOption(opt1);
    // exec_timer_->Start();
    // std::cout << "1111" << std::endl;

    // cyber::TimerOption opt2;
    // opt2.oneshot = false;
    // opt2.callback = boost::bind(&MincoShenlanComponent::checkCollisionCallback, this);
    // opt2.period = 100;
    // safety_timer_ = std::make_shared<cyber::Timer>();
    // safety_timer_->SetTimerOption(opt2);
    // safety_timer_->Start();
    // std::cout << "2222" << std::endl;

    adc_writer_ = node_->CreateWriter<apollo::planning::ADCTrajectory>("/apollo/planning");
    
    last_seq = -1;

    fp = fopen("test00033444.txt", "rb");
    fread(&current_seq, sizeof(uint32_t), 1, fp);
    //fread(current_data, sizeof(double), 360000, fp);

    return true;
}

bool diff(char *a, char *b, int size)
{
    for (int idx = 0; idx < size; idx++) {
        if (a[idx] != b[idx]) { return true;}
    }
    return false;
}

bool MincoShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg) 
{
  //int size__ = buf_msg->occupancy_buffer_2d_size();
  
  // uint32_t seq = odom_msg->header().sequence_num();
  //std::cout << seq << " ==== " << size__ << " diff:" << diff((char*)current_data, (char*)buf_msg->mutable_occupancy_buffer_2d()->mutable_data(), sizeof(double) * size__) << std::endl;
  //return true;
  //std::cout << "occupancy_buffer_2d_size:" << buf_msg->occupancy_buffer_2d_size() << std::endl;
  //std::cout << "+++++++++" << seq << "   " << current_seq << std::endl;
  // if (current_seq != seq) return true;
  // std::cout << "=========" << seq << "   " << current_seq << std::endl;
  //std::cout << "0000" << std::endl;
  //CreateMapCallback(odom_msg, buf_msg);
  //std::cout << "1111" << std::endl;
  OdomCallback(odom_msg);
  //std::cout << "2222" << std::endl;
  int size = buf_msg->occupancy_buffer_2d_size();

  //int size = buf_msg->occupancy_buffer_2d().size();
  auto buf_msg_ = std::make_shared<apollo::shenlan::OccupancyBuffer>();
  buf_msg_->mutable_occupancy_buffer_2d()->Resize(size, 0.0);

  fread(buf_msg_->mutable_occupancy_buffer_2d()->mutable_data(), sizeof(double), size, fp);
  fread(&current_seq, sizeof(uint32_t), 1, fp);
  execFSMCallback(buf_msg);
  //std::cout << "3333" << std::endl;
  checkCollisionCallback(buf_msg);
  //std::cout << "4444" << std::endl;
  //RFSM.print();

  return true;
}


/*
bool MincoShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg) 
{
  uint32_t seq = odom_msg->header().sequence_num();
  //std::cout << "occupancy_buffer_2d_size:" << buf_msg->occupancy_buffer_2d_size() << std::endl;
  //std::cout << "+++++++++" << seq << "   " << current_seq << std::endl;
  if (current_seq != seq) return true;
  std::cout << "=========" << seq << "   " << current_seq << std::endl;
  //std::cout << "0000" << std::endl;
  //CreateMapCallback(odom_msg, buf_msg);
  //std::cout << "1111" << std::endl;
  OdomCallback(odom_msg);
  //std::cout << "2222" << std::endl;
  int size = buf_msg->occupancy_buffer_2d_size();

  //int size = buf_msg->occupancy_buffer_2d().size();
  auto buf_msg_ = std::make_shared<apollo::shenlan::OccupancyBuffer>();
  buf_msg_->mutable_occupancy_buffer_2d()->Resize(size, 0.0);

  fread(buf_msg_->mutable_occupancy_buffer_2d()->mutable_data(), sizeof(double), size, fp);
  fread(&current_seq, sizeof(uint32_t), 1, fp);
  execFSMCallback(buf_msg_);
  //std::cout << "3333" << std::endl;
  checkCollisionCallback(buf_msg_);
  //std::cout << "4444" << std::endl;
  //RFSM.print();
  if (__count__simon == 5) {
    fclose(fp);
    std::cout << "exit()" << std::endl;
    exit(0);
  } else {
    __count__simon++;
  }
  return true;
}
*/

/*
bool MincoShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg) 
{
    // std::cout << "MincoShenlanComponent::Proc" << std::endl;

    OdomCallback(odom_msg);

    execFSMCallback(buf_msg);

    checkCollisionCallback(buf_msg);

    return true;
}
*/
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
}

void MincoShenlanComponent::execFSMCallback(const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{
    //std::cout << "execFSM:" << apollo::cyber::Time::Now().ToNanosecond() << std::endl;
    //exec_timer_->Stop();
    int ret = RFSM.execFSM(buf_msg);
    if (ret == 1) {
      auto adc_msg = std::make_shared<apollo::planning::ADCTrajectory>();
      calcMinco2ADC(adc_msg);
      adc_writer_->Write(adc_msg);
    }
    //exec_timer_->Start();
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
}

void MincoShenlanComponent::checkCollisionCallback(const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{
    //std::cout << "checkcolli:" << apollo::cyber::Time::Now().ToNanosecond() << std::endl;
    //std::cout << "checkcolli:" << last_seq << std::endl;
    //double time_now = ros::Time::now().toSec();
    double time_now = apollo::cyber::Time::Now().ToSecond();
    // set other cars' position of map is free
    RFSM.planner_ptr_->setMapFree(time_now);

    // check collision with static obstacles
    if(RFSM.exec_state_ == RFSM.EXEC_TRAJ)
        RFSM.collision_with_obs_ = RFSM.planner_ptr_->checkCollisionWithObs(time_now, buf_msg);

    // check collision with surround cars
    if(RFSM.exec_state_ == RFSM.EXEC_TRAJ)
        RFSM.collision_with_othercars_ = RFSM.planner_ptr_->checkCollisionWithOtherCars(time_now, buf_msg);
}

}  // namespace shenlan
}  // namespace apollo