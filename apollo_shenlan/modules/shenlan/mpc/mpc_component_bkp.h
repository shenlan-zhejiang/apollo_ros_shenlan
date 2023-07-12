#pragma once

#include<cstdlib>
#include<iostream>
#include<memory>
#include<string>
#include<vector>

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"

#include "mpc.h"


namespace apollo{
namespace shenlan{

class MpcShenlanComponent final : public cyber::Component<apollo::shenlan::mpc::Nav_path, apollo::shenlan::mpc::Trajectory, localization::LocalizationEstimate,apollo::shenlan::mpc::EgoVehicleStatus>
{
    public:

    bool Init() override;
    bool Proc(const std::shared_ptr<apollo::shenlan::mpc::Nav_path> &path, 
    const std::shared_ptr<apollo::shenlan::mpc::Trajectory> &traj, 
    const std::shared_ptr<localization::LocalizationEstimate> &odom,
    const std::shared_ptr<apollo::shenlan::mpc::EgoVehicleStatus> &status
    ) override;

    std::string Name() const {return "shenlan_mpc";}

    private:
    void triggerCallback(const shared_ptr<apollo::shenlan::mpc::Nav_path> &msg);
    void trajCallback(const shared_ptr<apollo::shenlan::mpc::Trajectory> &msg);
    void odomCallback(const shared_ptr<localization::LocalizationEstimate> &msg);
    void statusCallback(const shared_ptr<apollo::shenlan::mpc::EgoVehicleStatus> &msg);
    std::shared_ptr<cyber::Writer<drivers::PointCloud>> pc_writer_;
    std::shared_ptr<cyber::Writer<drivers::PointCloud>> map_writer_;
    MPC mpc;

    unsigned path_seq = 0;
    unsigned traj_seq = 0;
    unsigned odom_seq = 0;
    unsigned status_seq = 0;

};
CYBER_REGISTER_COMPONENT(MpcShenlanComponent);

}
}