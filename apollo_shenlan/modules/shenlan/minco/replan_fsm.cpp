#include "modules/shenlan/minco/plan_manage/replan_fsm.h"
#include <chrono>
#include "Eigen/Eigen"
#include "modules/shenlan/proto/shenlan_conf.pb.h"

namespace apollo {
namespace shenlan {

void ReplanFSM::init(apollo::shenlan::ShenlanConf &shenlan_conf)
{
    exec_state_ = ReplanFSM::FSM_EXEC_STATE::INIT;
    // apollo::shenlan::ReplanFSMConf replanfsm_conf;
    // apollo::shenlan::MappingConf mapping_conf;
    target_x_ = shenlan_conf.mapping_conf().origin_x() + shenlan_conf.replanfsm_conf().target_x();//587061-50.0;
    target_y_ = shenlan_conf.mapping_conf().origin_y() + shenlan_conf.replanfsm_conf().target_y();//4141628-50.0;
    target_yaw_ = shenlan_conf.replanfsm_conf().target_yaw();//1.57;
    target_vel_ = shenlan_conf.replanfsm_conf().target_vel();//0.1
    end_pt_ << target_x_, target_y_, target_yaw_, target_vel_;

    have_target_ = true;
    collision_with_obs_ = false;
    collision_with_othercars_ = false;

    //apollo::shenlan::VehicleConf vehicle_conf;
    car_d_cr_ = shenlan_conf.vehicle_conf().car_d_cr();//1.3864;
    car_id_ = shenlan_conf.vehicle_conf().car_id();//0;

    //nh_.param("mapping/odometry_topic", odom_topic_, odom_topic_);
}

void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    std::cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << std::endl;    
}

int ReplanFSM::execFSM() {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 50)
    {
    //   printFSMExecState();
    //   if (!have_odom_)
    //     cout << "no odom." << endl;
      if (!have_target_)
        std::cout << "wait for goal or trigger." << std::endl;
      fsm_num = 0;
    }

    //std::cout << "execFSM--:" << ros::Time::now() << std::endl;
    //std::cout << "execFSM:" << exec_state_ << " fsm_num:" << fsm_num << " " /*<< last_seq */<< std::endl;
    
    int ret = 0;

    switch (exec_state_)
    {
        case INIT:
        {
            std::cout << "case0 INIT" << std::endl;
            // if(!have_odom_)
            // {
            //     goto force_return;
            //     // return;
            // }

            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }

        case WAIT_TARGET:
        {
            std::cout << "case1 WAIT_TARGET" << std::endl;
            if(!have_target_ /*|| !have_trigger_*/)
                return ret;

            else
            {
                changeFSMExecState(SEQUENTIAL_START, "FSM");
            }
            break;
        }

        case SEQUENTIAL_START:
        {
            std::cout << "case6 SEQUENTIAL_START" << std::endl;
            // Eigen::Vector4d init_state;
            init_state_ << cur_pos_, cur_yaw_, cur_vel_;

            //double start_time = ros::Time::now().toSec() + TIME_BUDGET;
            double start_time = apollo::cyber::Time::Now().ToSecond() + TIME_BUDGET;

            start_world_time_ = start_time;
            
            //ros::Time t1 = ros::Time::now();
            auto t1 = apollo::cyber::Time::Now().ToSecond();

            planner_ptr_->setInitStateAndInput(init_state_, start_time);

            planner_ptr_->setParkingEnd(end_pt_);

            planner_ptr_->getKinoPath(end_pt_, true);

            // planner_ptr_->displayPolyH(planner_ptr_->display_hPolys());
            planner_ptr_->displayKinoPath(planner_ptr_->display_kino_path());

            planner_ptr_->RunMINCOParking();

            planner_ptr_->broadcastTraj2SwarmBridge();

            //ros::Time t2 = ros::Time::now();
            auto t2 = apollo::cyber::Time::Now().ToSecond();
            //std::cout << "case6 t2:" << t2 << std::endl;

            double time_spent_in_planning = (t2 - t1);

            if(TIME_BUDGET > time_spent_in_planning)
            {
                //ros::Duration(TIME_BUDGET - time_spent_in_planning).sleep(); ??????
            }
            else
            {
                std::cout << "Out of time budget!" << std::endl;;
            }
            //planner_ptr_->publishTraj2Controller();
            ret = 1;
            
            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());

            changeFSMExecState(EXEC_TRAJ, "FSM");

            break;
        }

        // case GEN_NEW_TRAJ:
        // {
        //    std::cout << "case2 GEN_NEW_TRAJ" << std::endl;
        // }

        case REPLAN_TRAJ:
        {
            std::cout << "case3 REPLAN_TRAJ" << std::endl;
            // ros::Time t_now = ros::Time::now();
            auto t_now = apollo::cyber::Time::Now().ToSecond();
            double replan_start_time = t_now + TIME_BUDGET;
            start_world_time_ = replan_start_time;
            
            auto t1 = apollo::cyber::Time::Now().ToSecond();
            Eigen::Vector4d replan_init_state;
            planner_ptr_->setInitStateAndInput(replan_start_time, replan_init_state);
            init_state_ = replan_init_state;

            planner_ptr_->setParkingEnd(end_pt_);
            if(!planner_ptr_->getKinoPath(end_pt_, false))
            {
                while(true)
                {
                    double t_cur = apollo::cyber::Time::Now().ToSecond();
                    if(t_cur > replan_start_time + 0.2)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                // ros::Duration(0.5).sleep();
                break;
            }
            planner_ptr_->displayKinoPath(planner_ptr_->display_kino_path());
            if(!planner_ptr_->RunMINCOParking())
            {
                while(true)
                {
                    double t_cur = apollo::cyber::Time::Now().ToSecond();
                    if(t_cur > replan_start_time + 0.1)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
                break;
            }
            planner_ptr_->broadcastTraj2SwarmBridge();
            auto t2 = apollo::cyber::Time::Now().ToSecond();
            double time_spent_in_planning = (t2 - t1);
            if(TIME_BUDGET > time_spent_in_planning)
            {
                // ros::Duration(TIME_BUDGET - time_spent_in_planning).sleep();
                while(true)
                {
                    double t_cur = apollo::cyber::Time::Now().ToSecond();
                    if(t_cur > replan_start_time)
                    {
                        break;
                    }
                    else
                    {
                        planner_ptr_->setMapFree(t_cur);
                    }
                }
            }
            else
            {
                //ROS_ERROR("Out of time budget!");
                std::cout << "Out of time budget!" << std::endl;
            }
            //planner_ptr_->publishTraj2Controller();
            ret = 1;
            
            // planner_ptr_->displayPolyH(planner_ptr_->display_hPolys());
            
            planner_ptr_->displayMincoTraj(planner_ptr_->trajectory());
            changeFSMExecState(EXEC_TRAJ, "FSM");

            break;
        }

        case EXEC_TRAJ:
        {
            std::cout << "case4 EXEC_TRAJ" << std::endl;
            auto t_now = apollo::cyber::Time::Now().ToSecond();
            if(((cur_pos_ - init_state_.head(2)).norm() > 10.0 || (t_now - start_world_time_) > 2.5) && (cur_pos_ - end_pt_.head(2)).norm() > 5.0 /*&& !collision_with_othercars_*/)
            {
                //std::cout << "cur_pos_ - init_state_.head(2)).norm() > 10.0: "<< std::endl << (cur_pos_ - init_state_.head(2)).norm() << " " << cur_pos_<< " " << init_state_.head(2) << std::endl;
                //std::cout << "t_now - start_world_time_ > 2.5: "<< std::endl << t_now - start_world_time_<< " " << t_now << " " << start_world_time_ << std::endl;
                //std::cout << "cur_pos_ - end_pt_.head(2)).norm() > 5.0: "<< std::endl << (cur_pos_ - end_pt_.head(2)).norm()<< " " << cur_pos_<< " " << end_pt_.head(2) << std::endl;
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }

            if((collision_with_obs_ || collision_with_othercars_) && t_now - start_world_time_ > TIME_BUDGET /*+ 0.3*/) // make sure the new trajectory have been executed and then replan
            {
                //std::cout << "t_now - start_world_time_ > TIME_BUDGET: " << t_now - start_world_time_ <<std::endl;
                changeFSMExecState(REPLAN_TRAJ, "FSM");
                collision_with_obs_ = false;
                collision_with_othercars_ = false;
                break;
            }

           // reach end
            if((cur_pos_ - end_pt_.head(2)).norm() < 0.5 && abs(cur_yaw_ - end_pt_(2) < 0.15 && abs(cur_vel_) < 0.2))
            {
                changeFSMExecState(WAIT_TARGET, "FSM");
                have_target_ = false;
                return ret;
            }

            break;
        }

        // case EMERGENCY_STOP:
        // {
        //    std::cout << "case5 EMERGENCY_STOP" << std::endl;
        // }

        default:
           return ret;
    }

    return ret;
}
}
}