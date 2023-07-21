#ifndef _REPLAN_FSM_H_
#define _REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>
//#include <tf/transform_broadcaster.h>

#include "modules/shenlan/mapping/mapping.h"
#include "modules/shenlan/minco/plan_manage/traj_manager.h"
#include "modules/shenlan//proto/shenlan_conf.pb.h"
// #include "swarm_bridge/Trajectory.h" //????
// #include <plan_env/grid_map.h>
// #include <traj_utils/Bspline.h>
// #include <traj_utils/MultiBsplines.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <traj_utils/DataDisp.h>
// #include <plan_manage/planner_manager.h>
// #include <traj_utils/planning_visualization.h>

const double TIME_BUDGET = 0.06;

namespace apollo {
namespace shenlan {
class ReplanFSM
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public:

    enum FSM_EXEC_STATE
    {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        EMERGENCY_STOP,
        SEQUENTIAL_START
    };

    enum TARGET_TYPE
    {
        MANNUAL_TARGET = 1,
        PRESET_TARGET = 2,
        REFERENCE_PATH = 3
    };


    std::shared_ptr<apollo::shenlan::MappingProcess> mapping_ptr_;
    std::shared_ptr<TrajPlanner> planner_ptr_;
    //TrajPlanner::Ptr planner_ptr_;
     
    bool have_target_, collision_with_obs_, collision_with_othercars_;
    Eigen::Vector4d init_state_;
    Eigen::Vector4d end_pt_;
    Eigen::Vector2d cur_pos_;
    double cur_yaw_, cur_vel_;
    int car_id_;
    double car_d_cr_;
    double start_world_time_;
    double target_x_, target_y_, target_yaw_, target_vel_;

    FSM_EXEC_STATE exec_state_;

    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    int execFSM();

public:
    ReplanFSM()
    {
    }
    ~ReplanFSM()
    {
    }

    void init(apollo::shenlan::ShenlanConf &shenlan_conf);
    void print()
    {
        //std::cout << "cur_pos:" << std::endl << cur_pos_ << std::endl;      
        //std::cout << "cur_state:" << std::endl << cur_yaw_ << " " << cur_vel_ << " " << exec_state_ << std::endl;
        //std::cout << "init_state: " << std::endl << init_state_ << std::endl;
        //planner_ptr_->print();
    }
    std::string odom_topic_ = "map";
    
};
}
}

#endif