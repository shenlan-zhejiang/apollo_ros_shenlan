#include "modules/shenlan/minco/minco_component.h"

namespace apollo {
namespace shenlan {

bool MincoShenlanComponent::Init() 
{
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

    kino_writer_ = node_->CreateWriter<apollo::shenlan::NavPath>("/apollo/shenlan/minco/kino_traj");
    minco_writer_ = node_->CreateWriter<apollo::shenlan::NavPath>("/apollo/shenlan/minco/minco_traj");
    adc_writer_ = node_->CreateWriter<apollo::planning::ADCTrajectory>("/apollo/planning");

    last_seq = -1;

    return true;
}

bool MincoShenlanComponent::Proc(const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg) 
{
    // std::cout << "MincoShenlanComponent::Proc" << std::endl;

    OdomCallback(odom_msg);

    execFSMCallback(buf_msg);

    checkCollisionCallback(buf_msg);

    return true;
}

void MincoShenlanComponent::OdomCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg)
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

    // @ROCY: for setInitStateAndInput(replan_start_time, replan_init_state, cur_yaw_, cur_vel_, start_pos_, start_vel_, start_acc_);
    Eigen::Vector3d linear_velocity(msg->pose().linear_velocity().x(), msg->pose().linear_velocity().y(), msg->pose().linear_velocity().z());
    Eigen::Vector3d linear_acceleration(msg->pose().linear_acceleration().x(), msg->pose().linear_acceleration().y(), msg->pose().linear_acceleration().z());
    RFSM.start_pos_ = center_pos.head(2);
    RFSM.start_vel_ = linear_velocity.head(2);
    RFSM.start_acc_ = linear_acceleration.head(2);

    // @ROCY: for trajs visualization
    RFSM.cur_z = msg->pose().position().z();
}

void MincoShenlanComponent::execFSMCallback(const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_msg)
{
    // std::cout << "execFSM:" << apollo::cyber::Time::Now().ToNanosecond() << std::endl;
    // exec_timer_->Stop();
    int ret = RFSM.execFSM(buf_msg);
    if (ret == 1) {
      auto kino_msg = std::make_shared<apollo::shenlan::NavPath>();
      displayKinoPath(kino_msg);
      kino_writer_->Write(kino_msg);

      auto minco_msg = std::make_shared<apollo::shenlan::NavPath>();
      displayMincoTraj(minco_msg);
      minco_writer_->Write(minco_msg);
      
      auto adc_msg = std::make_shared<apollo::planning::ADCTrajectory>();
      calcMinco2ADC(adc_msg);
      adc_writer_->Write(adc_msg);
    }
    // exec_timer_->Start();
}

void MincoShenlanComponent::displayKinoPath(const std::shared_ptr<apollo::shenlan::NavPath> &kino_msg)
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
        //std::cout << "111111111111kino_pts_size: " <<  RFSM.planner_ptr_->kino_trajs_.at(i).traj_pts.size() << std::endl;
        //for (size_t k = 0; k <  RFSM.planner_ptr_->kino_trajs_.at(i).traj_pts.size(); k++)
        for (size_t k = 0; k <  RFSM.planner_ptr_->kino_trajs_.at(i).traj_pts.size(); k ++)
        {
            Eigen::Vector3d pt =  RFSM.planner_ptr_->kino_trajs_.at(i).traj_pts[k];
            //std::cout << "111111111111kino_pt: " << pt << std::endl;
            position.set_x(pt(0));
            position.set_y(pt(1));
            position.set_z(RFSM.cur_z);
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

void MincoShenlanComponent::displayMincoTraj(const std::shared_ptr<apollo::shenlan::NavPath> &minco_msg)
{
    apollo::common::Quaternion quaternion;
    quaternion.set_qw(1.0);
    quaternion.set_qx(0.0);
    quaternion.set_qy(0.0);
    quaternion.set_qz(0.0);
    apollo::common::PointENU position;
    //std::cout << "222222222222minco_size: " << RFSM.planner_ptr_->traj_container_.singul_traj.size() << std::endl;
    for (unsigned int i = 0; i < RFSM.planner_ptr_->traj_container_.singul_traj.size(); ++ i)
    {
        //std::cout << "222222222222minco_duration: " << RFSM.planner_ptr_->traj_container_.singul_traj.at(i).duration << std::endl;
        for (double t = 0; t <= RFSM.planner_ptr_->traj_container_.singul_traj.at(i).duration; t += 0.2)
        {
            Eigen::Vector2d pt = RFSM.planner_ptr_->traj_container_.singul_traj.at(i).traj.getPos(t);
            //std::cout << "222222222222minco_pt: " << pt << std::endl;
            position.set_x(pt(0));
            position.set_y(pt(1));
            position.set_z(RFSM.cur_z);
            auto pose = minco_msg->add_pose();
            pose->mutable_position()->CopyFrom(position);
            pose->mutable_orientation()->CopyFrom(quaternion);

            double heading = RFSM.planner_ptr_->traj_container_.singul_traj.at(i).traj.getAngle(t);
            pose->set_heading(heading);
        }
    }
    auto timestamp = apollo::cyber::Time::Now().ToSecond();
    minco_msg->mutable_header()->set_timestamp_sec(timestamp);
    minco_msg->mutable_header()->set_frame_id("map");
    minco_msg->mutable_header()->set_sequence_num(seq_num_adc);
    seq_num_minco += 1;
}

void MincoShenlanComponent::calcMinco2ADC(const std::shared_ptr<apollo::planning::ADCTrajectory> &traj_msg)
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
            double acc = RFSM.planner_ptr_->traj_container_.singul_traj[i].traj.getAcc(t1);
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