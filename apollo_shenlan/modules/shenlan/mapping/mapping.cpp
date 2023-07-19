#include "modules/shenlan/mapping/mapping.h"
#include "modules/shenlan/mapping/raycast.h"

namespace apollo {
namespace shenlan {

void MappingProcess::init(apollo::shenlan::ShenlanConf &shenlan_conf) //double _resolution_, double _lidar_height_, Eigen::Vector3d& _origin_, Eigen::Vector3d& _map_size_
{
    /*----init---*/
    //apollo::shenlan::VehicleConf vehicle_conf;
    cars_num_ = shenlan_conf.vehicle_conf().cars_num();//1;
    car_id_ = shenlan_conf.vehicle_conf().car_id();//0;

    //apollo::shenlan::MappingConf mapping_conf;
    origin_(0) = shenlan_conf.mapping_conf().origin_x() + shenlan_conf.mapping_conf().map_origin_x();//587061-100;
    origin_(1) = shenlan_conf.mapping_conf().origin_y() + shenlan_conf.mapping_conf().map_origin_y();//4141628-100;
    origin_(2) = shenlan_conf.mapping_conf().origin_z() + shenlan_conf.mapping_conf().map_origin_z();//-0.1+0.1;
    map_size_(0) = shenlan_conf.mapping_conf().map_size_x();//200.0;
    map_size_(1) = shenlan_conf.mapping_conf().map_size_y();//200.0;
    map_size_(2) = shenlan_conf.mapping_conf().map_size_z();//10.0;
    resolution_ = shenlan_conf.mapping_conf().resolution();//0.3;
    min_ray_length_ = shenlan_conf.mapping_conf().min_ray_length();//0.0;
    max_ray_length_ = shenlan_conf.mapping_conf().max_ray_length();//30.0;
    prob_hit_log_ = shenlan_conf.mapping_conf().prob_hit_log();//1.2;
    prob_miss_log_ = shenlan_conf.mapping_conf().prob_miss_log();//-0.3;
    clamp_min_log_ = shenlan_conf.mapping_conf().clamp_min_log();//-2.0;
    clamp_max_log_ = shenlan_conf.mapping_conf().clamp_max_log();//2.0;
    min_occupancy_log_ = shenlan_conf.mapping_conf().min_occupancy_log();//1.39;
    lidar_height_ = shenlan_conf.mapping_conf().lidar_height();//3.0;
    obs_low_ = shenlan_conf.mapping_conf().obs_low();
    obs_high_ = shenlan_conf.mapping_conf().obs_high();
    obs_circle_ = shenlan_conf.mapping_conf().obs_circle();
    
    // odom_topic_ = odom_topic_;
    // lidar_topic_ = lidar_topic_;
    // map_frame_id_ = map_frame_id_;
    // map_pub_topic_ = map_pub_topic_;

    /*-------------------   settings   ------------------------*/
    have_odom_ = false;
    global_map_valid_ = false;
    local_map_valid_ = false;
    has_global_cloud_ = false;
    have_received_freespaces_ = false;

    resolution_inv_ = 1 / resolution_;
    for(int i=0;i<3;i++) 
    {
        global_map_size_(i) = ceil(map_size_(i) / resolution_);
    }

    lidar2car_ << 0.0, 0.0, lidar_height_;

    min_range_ = origin_; //???????????????still don't know what it is used for??????????????
    max_range_ = origin_ + map_size_;

    //inititalize size of buffer
    grid_size_y_multiply_z_ = global_map_size_(1) * global_map_size_(2);
    buffer_size_ = global_map_size_(0) * grid_size_y_multiply_z_; //The size of the global map
    buffer_size_2d_ = global_map_size_(0) * global_map_size_(1);
    occupancy_buffer_.resize(buffer_size_);
    occupancy_buffer_2d_.resize(buffer_size_2d_);
    
    cache_all_.resize(buffer_size_);
    cache_hit_.resize(buffer_size_);
    cache_rayend_.resize(buffer_size_);
    cache_traverse_.resize(buffer_size_);
    raycast_num_ = 0;

    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), -1.0);
    fill(occupancy_buffer_2d_.begin(), occupancy_buffer_2d_.end(), -1.0);
    fill(cache_all_.begin(), cache_all_.end(), 0);
    fill(cache_hit_.begin(), cache_hit_.end(), 0);
    fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
    fill(cache_traverse_.begin(), cache_traverse_.end(), -1);

    std::cout << "MappingProcess Initialized!\n" << std::endl;;

}

void MappingProcess::raycastProcess(const Eigen::Vector3d& t_wc, const std::shared_ptr<drivers::PointCloud>& laserCloudTransformed) //t_wc is the position of the lidar
{
    //std::cout << "+++++++raycastProcess+++++++" << std::endl;

    if(number_of_points_ == 0)
        return;

    int cache_count = 0;
    raycast_num_ += 1;
    //std::cout << raycast_num_ << std::endl;

    int set_cache_idx;
    /*----------iterate over all points of a frame of pointcloud----------*/
    for(int i=0; i<number_of_points_; i++)  
    {
        Eigen::Vector3d pt_w(laserCloudTransformed->point(i).x(), 
                             laserCloudTransformed->point(i).y(), 
                             laserCloudTransformed->point(i).z());

        bool inside_car = false;
        
        if(have_received_freespaces_)
        {
            for(int car = 0; car < cars_num_; car++)
            {
                if(car == car_id_)
                    continue;

                Eigen::MatrixXd normalVec_and_points = vec_freespaces_[car];

                if((Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(0).tail(2))).dot(normalVec_and_points.col(0).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(1).tail(2))).dot(normalVec_and_points.col(1).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(2).tail(2))).dot(normalVec_and_points.col(2).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(3).tail(2))).dot(normalVec_and_points.col(3).head(2)) <= 0)
                {
                    inside_car = true;
                    break;
                }
                
            }
            
        }

        if(inside_car)
            continue;

        double length = (pt_w - t_wc).norm();
        // std::cout << length << std::endl;
        // std::cout << min_ray_length_ << std::endl;
        // std::cout << max_ray_length_ << std::endl;

        if (length < min_ray_length_)
            continue;
        else if (length > max_ray_length_)
        {
            pt_w = (pt_w - t_wc) / length * max_ray_length_ + t_wc;
            cache_count++;
            set_cache_idx = setCacheOccupancy(pt_w, 0);
            //std::cout << cache_count << "===1==" << cache_voxel_.size() << std::endl;
        }
        else {
            set_cache_idx = setCacheOccupancy(pt_w, 1);
            cache_count++;
            //std::cout << cache_count << "===2==" << cache_voxel_.size() << std::endl;
        }

        if(set_cache_idx != INVALID_IDX)
        {
            if(cache_rayend_[set_cache_idx] == raycast_num_)
            {
                continue;
            }
            else   
                cache_rayend_[set_cache_idx] = raycast_num_;
        }

        RayCaster raycaster;
        bool need_ray = raycaster.setInput(pt_w / resolution_, t_wc / resolution_);
        if(!need_ray)
            continue;
        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
        Eigen::Vector3d ray_pt;
        if(!raycaster.step(ray_pt))
            continue;
        while(raycaster.step(ray_pt))
        {
            Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
            set_cache_idx = setCacheOccupancy(tmp, 0);
            cache_count++;
            //std::cout << cache_count << "===3==" << cache_voxel_.size() << std::endl;
            if(set_cache_idx != INVALID_IDX)
            {
                if(cache_traverse_[set_cache_idx] == raycast_num_)
                    break;
                else
                    cache_traverse_[set_cache_idx] = raycast_num_;
            }
        }
    }

    // std::cout << cache_voxel_.front() << std::endl;

    while(!cache_voxel_.empty())
    {
        Eigen::Vector3i idx = cache_voxel_.front();
        int idx_ctns = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + idx(2);
        
        // std::cout << idx(0) << std::endl;
        // std::cout << idx(1) << std::endl;
        // std::cout << idx(2) << std::endl;
        //std::cout << grid_size_y_multiply_z_ << std::endl;
        //std::cout << global_map_size_(2) << std::endl;

        cache_voxel_.pop();

        //std::cout << idx_ctns << std::endl;

        double log_odds_update =
            cache_hit_[idx_ctns] >= cache_all_[idx_ctns] - cache_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;
        cache_hit_[idx_ctns] = cache_all_[idx_ctns] = 0;

        if ((log_odds_update >= 0 && occupancy_buffer_[idx_ctns] >= clamp_max_log_) ||
            (log_odds_update <= 0 && occupancy_buffer_[idx_ctns] <= clamp_min_log_))
            continue;

        //------------------- With the "if" below, the map in the past is also stored ----------------------------//
        // if(occupancy_buffer_[idx_ctns] <= min_occupancy_log_)
        // {
            occupancy_buffer_[idx_ctns] =
                std::min(std::max(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_min_log_), clamp_max_log_);
                // std::min(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_max_log_);

            if(occupancy_buffer_[idx_ctns] > min_occupancy_log_)
            {
                int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
                occupancy_buffer_2d_[idx_ctns_2d] = 1;
            }
            else
            {
                int number_of_freespaces_in_z = 0;
                for(int z = 0; z < global_map_size_(2); z++)
                {
                    int idx_ctns_3d = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + z;
                    if(occupancy_buffer_[idx_ctns_3d] < min_occupancy_log_)
                    {
                        number_of_freespaces_in_z++;
                    }
                    else
                    {
                        break;
                    }
                }
                if(number_of_freespaces_in_z == global_map_size_(2))
                {
                    int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
                    occupancy_buffer_2d_[idx_ctns_2d] = 0;
                }
            }
        // }

    }
}

int MappingProcess::setCacheOccupancy(const Eigen::Vector3d& pos, int occ)
{
    if (occ != 1 && occ != 0)
    {
        return INVALID_IDX;
    }

    Eigen::Vector3i id;
    posToIndex(pos, id);

    if (!isInMap(id))
    {
        return INVALID_IDX;
    }
    //std::cout << "valid_idx:" << id << std::endl;

    int idx_ctns = id(0) * grid_size_y_multiply_z_ + id(1) * global_map_size_(2) + id(2);

    cache_all_[idx_ctns] += 1;

    if (cache_all_[idx_ctns] == 1)
    {
        cache_voxel_.push(id);
    }

    if (occ == 1)
        cache_hit_[idx_ctns] += 1;

    return idx_ctns;    
}

inline bool MappingProcess::isInMap(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    return isInMap(idx);
}

inline bool MappingProcess::isInMap(const Eigen::Vector3i &id)
{
    return ((id[0] | (global_map_size_[0] - 1 - id[0]) | id[1] | (global_map_size_[1] - 1 - id[1]) | id[2]| (global_map_size_[2] - 1 - id[2])) >= 0);
};

inline bool MappingProcess::isInLocalMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInLocalMap(idx);
}

inline bool MappingProcess::isInLocalMap(const Eigen::Vector3i &id)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min_, min_id);
  posToIndex(local_range_max_, max_id);
  min_id(0) = std::max(0, min_id(0));
  min_id(1) = std::max(0, min_id(1));
  min_id(2) = std::max(0, min_id(2));
  max_id(0) = std::min(global_map_size_[0], max_id(0));
  max_id(1) = std::min(global_map_size_[1], max_id(1));
  max_id(2) = std::min(global_map_size_[2], max_id(2));
  return (((id[0] - min_id[0]) | (max_id[0] - id[0]) | (id[1] - min_id[1]) | (max_id[1] - id[1]) | (id[2] - min_id[2]) | (max_id[2] - id[2])) >= 0);
};

bool MappingProcess::isInMap2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i idx;
    posToIndex2d(pos, idx);
    return isInMap2d(idx);
}

bool MappingProcess::isInMap2d(const Eigen::Vector2i &id)
{
    if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
    {
        return false;
    }
    else
        return true;
};

void MappingProcess::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
{
    for(int i = 0; i < 2; i++)
        id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void MappingProcess::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
{
    // pos = origin_;
    for(int i = 0; i < 2; i++)
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);//using namespace message_filters;
}

int MappingProcess::getVoxelState2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    posToIndex2d(pos, id);
    if(!isInMap2d(id))
        return -1;
    // todo: add local map range

    return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)] > 0.5 ? 1 : 0;
}

void MappingProcess::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for(int i = 0; i < 3; i++)
        id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void MappingProcess::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
    pos = origin_;
    for(int i = 0; i < 3; i++)
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
}

int MappingProcess::getVoxelState(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    if(!isInMap(id))
        return -1;
    if(!isInLocalMap(id))
        return 0;

    return occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * global_map_size_(2) + id(2)] > min_occupancy_log_ ? 1 : 0;
}

void MappingProcess::setFreeSpacesForMapping(const std::vector<Eigen::MatrixXd>& vec_freespaces)
{
    vec_freespaces_ = vec_freespaces;
    have_received_freespaces_ = true;
}

}
}
