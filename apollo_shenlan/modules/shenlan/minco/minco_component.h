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
#include "modules/shenlan/proto/shenlan_pb.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/common/util.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"

#include "modules/shenlan/minco/plan_manage/replan_fsm.h"
// #include "modules/shenlan/mpc/proto/Trajectory.pb.h"

namespace apollo{
namespace shenlan{

class MincoShenlanComponent final : public cyber::Component<localization::LocalizationEstimate, apollo::shenlan::OccupancyBuffer> 
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        bool Init() override;
        bool Proc( const std::shared_ptr<localization::LocalizationEstimate> &odm_, const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &buf_) override;

        std::string Name() const {return "minco_shenlan";}
    private:
        apollo::shenlan::ReplanFSM RFSM;
        apollo::shenlan::ShenlanConf shenlan_conf;
          
        std::shared_ptr<cyber::Timer> exec_timer_, safety_timer_;

        void OdomCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg);
        void execFSMCallback(const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &msg);
        void checkCollisionCallback(const std::shared_ptr<apollo::shenlan::OccupancyBuffer> &msg);

        std::shared_ptr<cyber::Writer<apollo::planning::ADCTrajectory>> adc_writer_;
        std::shared_ptr<cyber::Writer<apollo::shenlan::NavPath>> kino_writer_;
        std::shared_ptr<cyber::Writer<apollo::shenlan::NavPath>> minco_writer_;

        int seq_num_adc = 0;
        void calcMinco2ADC(const std::shared_ptr<apollo::planning::ADCTrajectory> &traj_msg);

        int seq_num_kino = 0;
        void displayKinoPath(const std::shared_ptr<apollo::shenlan::NavPath> &kino_msg);
        
        int seq_num_minco = 0;
        void displayMincoTraj(const std::shared_ptr<apollo::shenlan::NavPath> &minco_msg);

        int last_seq;
                
};

CYBER_REGISTER_COMPONENT(MincoShenlanComponent);

}
}
