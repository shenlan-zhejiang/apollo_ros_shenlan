#ifndef _TRAJ_MANAGER_H_
#define _TRAJ_MANAGER_H_

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>

#include "modules/shenlan/mapping/mapping.h"
// #include "mpc/Trajectory.h"
// #include "swarm_bridge/Trajectory.h"
#include "modules/shenlan//proto/shenlan_conf.pb.h"

#include "modules/shenlan/minco/plan_manage/traj_optimizer.h"

// #include "decomp_util/ellipsoid_decomp.h"
// #include "decomp_ros_utils/data_ros_utils.h"

// #include "plan_utils/CorridorBuilder2d.hpp"
#include "modules/shenlan/minco/path_searching/kino_astar.h"
//#include "tf/tf.h"
//#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/costmap_2d.h>
//#include <nav_core/base_local_planner.h>
//#include <tf2_ros/transform_listener.h>

//#include <pluginlib/class_loader.hpp>
//#include <geometry_msgs/PoseArray.h>
//#include <visualization_msgs/Marker.h>
using std::vector;

  
class TrajPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public:

    ///////////////////////////////////////////////////////////////
    TrajPlanner(){}
    ~TrajPlanner(){}
    typedef std::shared_ptr<TrajPlanner> Ptr;

    //ros::NodeHandle nh_;
    std::shared_ptr<apollo::shenlan::MappingProcess> map_ptr_;

    // void init(const std::string config_path, const int car_id);
    //void init(const ros::NodeHandle& nh);
    void init(apollo::shenlan::ShenlanConf &shenlan_conf);
    void setMap(std::shared_ptr<apollo::shenlan::MappingProcess>& ptr);
    void RunOnceParking();
    bool RunMINCOParking();
    void broadcastTraj2SwarmBridge();

    // void set_map_interface(map_utils::TrajPlannerMapItf* map_itf);
    // void set_initial_state(const State& state);
    void setInitStateAndInput(const Eigen::Vector4d& state, const double& start_time);
    void setInitStateAndInput(const double& t, Eigen::Vector4d& replan_init_state);
    //void setSwarmTrajs(const apollo::shenlan::swarm_bridge::Trajectory& traj_msg);
    void setMapFree(double& time_now);
    // void release_initial_state(){has_init_state_ = false;};
    // Vehicle ego_vehicle() const { return ego_vehicle_; }
    // vec_E<vec_E<Vehicle>> forward_trajs() const { return forward_trajs_; }
    // vec_E<Polyhedron2D> polys() const { return polys_; }
    // vec_Vec2f display_vec_obs() const { return vec_obs_; }
    std::vector<Eigen::MatrixXd> display_hPolys() const { return display_hPolys_; }
    // points visualizations
    // Eigen::MatrixXd display_InnterPs() const {return display_InnterPs_;}
    // Eigen::MatrixXd display_OptimalPs() const {return display_OptimalPs_;}
    // plan_utils::SurroundTrajData display_surround_trajs() const {return surround_trajs;};

    std::shared_ptr<plan_utils::SingulTrajData> trajectory() const {
      return std::shared_ptr<plan_utils::SingulTrajData>(
        new plan_utils::SingulTrajData(traj_container_.singul_traj));
    }
    std::shared_ptr<plan_utils::KinoTrajData> display_kino_path() const {
      return std::shared_ptr<plan_utils::KinoTrajData>(
        new plan_utils::KinoTrajData(kino_trajs_)); 
    }

    // const plan_utils::SingulTrajData* get_trajectory()
    // {
    //   // return &traj_container_.singul_traj;
    //   // const plan_utils::SingulTrajData* const singul_traj = &traj_container_.singul_traj;
    //   return &traj_container_.singul_traj;
    // }
    
    
    void setParkingEnd(Eigen::Vector4d end_pt){
      end_pt = end_pt_;
      have_parking_target_ = true;
    }
    //set moving obstalces, only used in the parking scene
    // void set_surPoints(std::vector<std::vector<common::State>> surpoints){
    //   sur_discretePoints = surpoints;

    // }
    // decimal_t time_cost() const { return time_cost_; }

    // planning::minco::Config cfg_;
    plan_utils::TrajContainer traj_container_;
    double traj_piece_duration_;
    int traj_res_, dense_traj_res_;

    bool checkCollisionWithObs(const double& t_now);
    bool checkCollisionWithOtherCars(const double& t_now);

    bool getKinoPath(Eigen::Vector4d &end_state, bool first_search);
    void displayKinoPath(std::shared_ptr<plan_utils::KinoTrajData> kino_trajs);
    void displayPolyH(const std::vector<Eigen::MatrixXd> hPolys);
    void displayMincoTraj(std::shared_ptr<plan_utils::SingulTrajData> display_traj);

    //for apollo
    double start_time_;
    plan_utils::KinoTrajData kino_trajs_;

private:

    void ReadConfig(const std::string config_path);
    /**
     * @brief transform all the states in a batch
     */

    int continous_failures_count_{0};
    int car_id_;
    //_____________________________________
    /* kinodynamic a star path and parking*/
    std::unique_ptr<path_searching::KinoAstar> kino_path_finder_;
    //plan_utils::KinoTrajData kino_trajs_;

    
    // void getKinoPath(Eigen::Vector4d &end_state);
    // ErrorType RunMINCOParking();
    bool enable_urban_ = false;

    /*teb local planner, for benchmark hzc*/
    /*
    std::shared_ptr<pluginlib::ClassLoader<nav_core::BaseLocalPlanner>> blp_loader_;
    boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* buffer;
    tf2_ros::TransformListener* tf;*/

    std::unique_ptr<plan_manage::PolyTrajOptimizer> ploy_traj_opt_;
    std::vector<plan_utils::TrajContainer> swarm_traj_container_;
    std::vector<plan_utils::TrajContainer> swarm_last_traj_container_;
    std::vector<bool> have_received_trajs_;

    // vehicle state
    // State head_state_;
    Eigen::Vector4d start_state_;
    Eigen::Vector2d start_ctrl_;
    bool has_init_state_ = false;
    bool is_init;
    int cars_num_;
    double car_d_cr_, car_length_, car_width_, car_wheelbase_;

    //_____________
    /* Map related */
    std::vector<Eigen::MatrixXd> hPolys_, display_hPolys_;
    // vec_E<Polyhedron2D> polys_;
    // vec_Vec2f vec_obs_;
    


    // vec_E<State> ref_states_;
    // vec_E<plan_utils::Trajectory> ref_trajactory_list_;
    // vec_E<vec_E<Vehicle>> forward_trajs_;   
    // std::vector<LateralBehavior> forward_behaviors_;
    // std::vector<LateralBehavior> valid_behaviors_; 
    // vec_E<State> final_ref_states_;
    // plan_utils::SurroundTrajData surround_trajs;




    Eigen::MatrixXd display_InnterPs_, display_OptimalPs_;
    // std::vector<std::vector<common::State>> sur_discretePoints; // only used in the parking scene, moving obstacles





    // vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;

    // ErrorType ConvertSurroundTraj(plan_utils::SurroundTrajData* surround_trajs_ptr, int index);
    // ErrorType ConverSurroundTrajFromPoints(std::vector<std::vector<common::State>> sur_trajs,plan_utils::SurroundTrajData* surround_trajs_ptr);


    // ErrorType UpdateTrajectoryWithCurrentBehavior();
    // ErrorType RunMINCO();
    
    // Eigen::MatrixXd state_to_flat_output(const State& state);

    // void printLateralBehavior(LateralBehavior lateral_behavior);

    // ErrorType UpdateObsGrids();
    
    // ErrorType getSikangConst(plan_utils::Trajectory &Traj,
    //                          Eigen::MatrixXd &innerPs,  
    //                          Eigen::VectorXd &piece_dur_vec);
    // using galaxy to generate corridor                         
    // ErrorType getGalaxyConst(plan_utils::Trajectory &Traj,
    //                          Eigen::MatrixXd &innerPs,  
    //                          Eigen::VectorXd &piece_dur_vec); 
    //using rectangles to represent corridor
    void getRectangleConst(std::vector<Eigen::Vector3d> statelist);

    // bool checkShapeInside(Eigen::MatrixXd &hPoly, std::vector<Eigen::Vector2d> vertices);
    // bool checkPosInside(Eigen::MatrixXd &hPoly,Eigen::Vector2d pos);
    // void addBoundBox(Eigen::Vector2d &first_point, double angle, int i);

    //ros::Publisher KinopathPub_;
    //ros::Publisher Rectangle_poly_pub_;
    //ros::Publisher minco_traj_pub_;
    //ros::Publisher MincoPathPub_;
    //ros::Publisher TrajPathPub_;
    //ros::Publisher wholebody_traj_pub_;
    /*for benchmark*/
    
    //ros::Publisher KinopathPub;//for obca
    //ros::Publisher DensePathVisPub;//for libai
    //ros::Publisher DensePathPub;//for libai

    /*debug*/
    //ros::Publisher Debugtraj0Pub;
    //ros::Publisher Debugtraj1Pub;
    //ros::Publisher DebugCorridorPub;
    //ros::Publisher DebugtrajPub;
    /*vis dense hybridastar*/
    //ros::Publisher DenseKinopathPub;
    
    /*if parking*/
    bool have_parking_target_ = false;
    Eigen::Vector4d end_pt_;
    //double start_time_;
    /*vehicle param*/



};



#endif
