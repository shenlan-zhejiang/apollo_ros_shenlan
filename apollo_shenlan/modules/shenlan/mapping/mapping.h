#ifndef _MAPPING_H
#define _MAPPING_H

#include <iostream>
#include <math.h>
#include <queue>

#include "Eigen/Eigen"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/shenlan/proto/shenlan_conf.pb.h"

namespace apollo {
namespace shenlan {

#define INVALID_IDX -1

class MappingProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    MappingProcess() {};
    ~MappingProcess() {}; 

    void init(apollo::shenlan::ShenlanConf &shenlan_conf); //double _resolution_, double _lidar_height_, Eigen::Vector3d& _origin_, Eigen::Vector3d& _map_size_

    bool odomValid() { return have_odom_; }
    bool mapValid() { return (global_map_valid_ || local_map_valid_); }
    void LoadGlobalMap();

    /*-----------------------get data------------------------*/
    double getResolution() {  return resolution_;  }
    int getVoxelState(const Eigen::Vector3d &pos);
    int getVoxelState(const Eigen::Vector3i &id);
    int getVoxelState2d(const Eigen::Vector2d &pos);
    int getVoxelState2d(const Eigen::Vector2i &id);
    void setFreeSpacesForMapping(const std::vector<Eigen::MatrixXd>& vec_freespaces);
    Eigen::Vector3d get_curr_posi() { return curr_posi_; }
    Eigen::Vector3d get_curr_twist() { return curr_twist_; }
    Eigen::Vector3d get_curr_acc() { return curr_acc_; }
    Eigen::Quaterniond get_curr_q_() { return curr_q_; }

    bool isInMap(const Eigen::Vector3d &pos);
    bool isInMap(const Eigen::Vector3i &id);
    bool isInMap2d(const Eigen::Vector2i &pos);
    bool isInMap2d(const Eigen::Vector2d &id);
    bool isInLocalMap(const Eigen::Vector3d &pos);
    bool isInLocalMap(const Eigen::Vector3i &id);
    void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
    void posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
    void indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
    
//private:
    /*-----------current states-------------------------------*/
    Eigen::Vector3d curr_posi_, curr_twist_, curr_acc_;
    Eigen::Quaterniond curr_q_;

    /************parameters******************************/
    Eigen::Vector3d origin_, map_size_, pred_map_size_;
    Eigen::Vector3d min_range_, max_range_;
    Eigen::Vector3i global_map_size_;  //global_map_size_ represents the number of the grids in each dircetion of the global map
    Eigen::Vector3d sensor_range_;
    Eigen::Vector3d center_position_, lidar2car_;
    double resolution_, resolution_inv_, lidar_height_;
    double obs_low_, obs_high_, obs_circle_;
    double min_ray_length_, max_ray_length_;
    Eigen::Vector3d local_range_min_, local_range_max_;
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_;
    double min_occupancy_log_;
    int grid_size_y_multiply_z_;
    int pred_size_y_multiply_z_;
    int buffer_size_, buffer_size_2d_;
    int number_of_points_; //This parameter represents the number of points in each frame of the pointcloud
    bool use_pred_;
    bool have_odom_;
    bool global_map_valid_, local_map_valid_;
    bool has_global_cloud_;
    bool have_received_freespaces_;

    std::vector<double> occupancy_buffer_; //This buffer stores the states of each grid in the "global" map
    std::vector<double> occupancy_buffer_2d_;

    /*----------------map fusion---------------------------*/
    std::vector<int> cache_all_, cache_hit_;
    std::vector<int> cache_traverse_, cache_rayend_;
    std::queue<Eigen::Vector3i> cache_voxel_;
    int raycast_num_;
    std::vector<Eigen::MatrixXd> vec_freespaces_;
    int car_id_, cars_num_;

    void raycastProcess(const Eigen::Vector3d& t_wc, const std::shared_ptr<drivers::PointCloud>& laserCloudTransformed);
    int setCacheOccupancy(const Eigen::Vector3d& pt_w, int occ);
};

}
}
#endif
