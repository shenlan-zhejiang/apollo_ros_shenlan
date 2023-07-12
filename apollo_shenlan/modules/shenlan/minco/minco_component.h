#pragma once

#include<cstdlib>
#include<iostream>
#include<memory>
#include<string>
#include<vector>


#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/drive_state.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/common/util.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"

#include "modules/shenlan/minco/plan_manage/replan_fsm.h"
#include "modules/shenlan/mpc/proto/Trajectory.pb.h"

namespace apollo{
namespace shenlan{

class MincoShenlanComponent final : public cyber::Component<localization::LocalizationEstimate, drivers::PointCloud> 
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        bool Init() override;
        bool Proc( const std::shared_ptr<localization::LocalizationEstimate> &odm_, const std::shared_ptr<drivers::PointCloud> &pc_) override;

        std::string Name() const {return "minco_shenlan";}
    private:
        apollo::shenlan::ReplanFSM RFSM;
        apollo::shenlan::ShenlanConf shenlan_conf;
          
        std::shared_ptr<cyber::Timer> exec_timer_, safety_timer_;

        void CreateMapCallback( const std::shared_ptr<localization::LocalizationEstimate> &odm_, const std::shared_ptr<drivers::PointCloud> &pc_);
        void ParkingCallback(const std::shared_ptr<apollo::localization::Pose> &msg);
        void OdomCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg);
        //void OdomCallback(const nav_msgs::Odometry& msg);
        //void SwarmTrajCallback(const swarm_bridge::Trajectory& traj_msg);
        void execFSMCallback();
        void checkCollisionCallback();

        std::shared_ptr<cyber::Writer<drivers::PointCloud>> pc_writer_;
        std::shared_ptr<cyber::Writer<drivers::PointCloud>> map_writer_;
        std::shared_ptr<cyber::Writer<apollo::shenlan::mpc::Trajectory>> traj_writer_;
        std::shared_ptr<cyber::Writer<apollo::planning::ADCTrajectory>> adc_writer_;

        int seq_num_map = 0;
        void globalOccPc(const std::shared_ptr<drivers::PointCloud> &msg);
        void calcTraj2Controller(const std::shared_ptr<apollo::shenlan::mpc::Trajectory> &msg);

        int seq_num_adc = 0;
        void calcMinco2ADC(const std::shared_ptr<apollo::planning::ADCTrajectory> &traj_msg);

        int last_seq;
        
        //std::shared_ptr<localization::LocalizationEstimate> odom_msg;
        
};

CYBER_REGISTER_COMPONENT(MincoShenlanComponent);

}
}
