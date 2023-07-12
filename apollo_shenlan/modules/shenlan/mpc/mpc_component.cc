#include "modules/shenlan/mpc/mpc_component.h"

namespace apollo {
namespace shenlan {

bool MpcShenlanComponent::Init() {
  mpc.init();
  return true;
}

bool MpcShenlanComponent::Proc(const std::shared_ptr<apollo::shenlan::mpc::EgoVehicleStatus> &status) {
    //trajCallback(traj);
    statusCallback(status);
    return true;
}

void MpcShenlanComponent::odomCallback(const std::shared_ptr<localization::LocalizationEstimate> &msg)      //nav_msgs::OdometryConstPtr 
{
    ;
}

void MpcShenlanComponent::trajCallback(const shared_ptr<apollo::shenlan::mpc::Trajectory> &msg)       //mpc::TrajectoryConstPtr msg
{
    //std::cout << "seq: " << msg->header().sequence_num() << std::endl;
    mpc.trajectory_analyzer_.setTraj(msg);
    mpc.receive_traj_ = true;
}

static void write_initg_aux(std::shared_ptr<cyber::Writer<apollo::shenlan::mpc::Nav_path>> writer_, Eigen::Vector3d& initp)
{
    auto initg = std::make_shared<mpc::Nav_path>();
    auto pose = initg->add_pose();
    initg->mutable_header()->set_frame_id("map");

    apollo::common::PointENU position;
    position.set_x(initp[0]);
    position.set_y(initp[1]);
    position.set_z(0.1);
    pose->mutable_position()->CopyFrom(position);

    Eigen::Matrix3d R;
    R<<cos(initp[2]),-sin(initp[2]),0,\
        sin(initp[2]),cos(initp[2]),0,\
        0,0,1;
    Eigen::Quaterniond q(R);
    
    apollo::common::Quaternion quaternion;
    quaternion.set_qw(q.w());
    quaternion.set_qx(q.x());
    quaternion.set_qy(q.y());
    quaternion.set_qz(q.z());
    pose->mutable_orientation()->CopyFrom(quaternion);
    //init_pub.publish(initg);
}

void MpcShenlanComponent::triggerCallback(const shared_ptr<apollo::shenlan::mpc::Nav_path>& msg)     //geometry_msgs::PoseStamped msg
{
    if (mpc.test_traj[16]=='m')
    {
        mpc.in_test = false;
        Eigen::Vector3d initp = mpc.trajectory_analyzer_.nextInitPointMinco();
        write_initg_aux(nullptr, initp);
        mpc.trajectory_analyzer_.beginNextTrajMinco();
        mpc.receive_traj_ = true;
    }
    else
    {
        Eigen::Vector3d initp = mpc.trajectory_analyzer_.nextInitPoint();
        write_initg_aux(nullptr, initp);
        mpc.trajectory_analyzer_.beginNextTraj(); 
        mpc.receive_traj_ = true;
    }
    
}

void MpcShenlanComponent::statusCallback(const shared_ptr<apollo::shenlan::mpc::EgoVehicleStatus> &msg)    //carla_msgs::CarlaEgoVehicleStatusConstPtr msg
{
    mpc.has_odom = true;
    mpc.now_state.x = msg->pose().position().x();
    mpc.now_state.y = msg->pose().position().y();
    Eigen::Quaterniond q(msg->pose().orientation().qw(),
                msg->pose().orientation().qx(),
                msg->pose().orientation().qy(),
                msg->pose().orientation().qz());
    Eigen::Matrix3d R(q);
    mpc.now_state.theta = atan2(R.col(0)[1],R.col(0)[0]);
    //mpc.now_state.v = (double)msg->velocity;


    apollo::common::PointENU position;
    position.set_x(msg->pose().position().x());
    position.set_y(msg->pose().position().y());
    position.set_z(0.1);

    apollo::common::Quaternion quaternion;
    quaternion.set_qw(msg->pose().orientation().qw());
    quaternion.set_qx(msg->pose().orientation().qx());
    quaternion.set_qy(msg->pose().orientation().qy());
    quaternion.set_qz(msg->pose().orientation().qz());

    apollo::localization::Pose pose;
    pose.mutable_position()->CopyFrom(position);
    pose.mutable_orientation()->CopyFrom(quaternion);

    auto pose_ = std::make_shared<apollo::localization::Pose>(msg->pose());
    pose_->mutable_position()->set_z(1234);

    auto odom = std::make_shared<localization::LocalizationEstimate>();
    odom->mutable_header()->set_timestamp_sec(msg->header().timestamp_sec());
    //odom->set_allocated_pose(pose_);
    odom->mutable_pose()->CopyFrom(*pose_);
    //true_odom_pub.publish(odom);

    std::cout<<"header:"<<msg->header().sequence_num()<<std::endl;
    std::cout<<"acceleration:"<<msg->acceleration()<<std::endl;
    std::cout<<"curvature:"<<msg->curvature()<<std::endl;
    std::cout<<"now_state.x:"<<mpc.now_state.x <<std::endl;
    std::cout<<"now_state.y:"<<mpc.now_state.y <<std::endl;
    std::cout<<"now_state.theta:"<<mpc.now_state.theta <<std::endl;
    std::cout<<std::endl;
}

}  // namespace planning
}  // namespace apollo
