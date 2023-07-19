#include "modules/shenlan/mapping/mapping_component.h"
#include "cyber/time/clock.h"

using apollo::cyber::Clock;

namespace apollo {
namespace shenlan {

using apollo::cyber::io::Session;
using apollo::localization::LocalizationEstimate;
using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;

bool MappingShenlanComponent::Init() 
{
  ACHECK(GetProtoConfig(&shenlan_conf)) << "Unable to load shenlan conf file";

  AINFO << "UDP bridge sender init, startin...";

  mp_.init(shenlan_conf);

  pc_writer_ = node_->CreateWriter<drivers::PointCloud>("/apollo/shenlan/mapping/pc_transformed");
  map_writer_ = node_->CreateWriter<drivers::PointCloud>("/apollo/shenlan/mapping/gird_map");
  
  last_seq = -1;

  return true;
}

bool MappingShenlanComponent::Proc( const std::shared_ptr<localization::LocalizationEstimate> &odom_msg, const std::shared_ptr<drivers::PointCloud> &pcl_msg) 
{
  if (last_seq == (int)(pcl_msg->header().sequence_num())) {
    return true;
  }
  last_seq = (int)(pcl_msg->header().sequence_num());
  std::cout << "handle new pcl" << std::endl;

  mp_.have_odom_ = true;
  mp_.local_map_valid_ = true;

  mp_.curr_posi_[0] = odom_msg->pose().position().x();
  mp_.curr_posi_[1] = odom_msg->pose().position().y();
  mp_.curr_posi_[2] = odom_msg->pose().position().z();
//   //svl    
//   mp_.curr_posi_[0] = odom_msg->pose().position().x() - 587061;
//   mp_.curr_posi_[1] = odom_msg->pose().position().y() - 4141628;
//   mp_.curr_posi_[2] = odom_msg->pose().position().z();
//   //huzhou
//   mp_.curr_posi_[0] = odom_msg->pose().position().x() - 218385;
//   mp_.curr_posi_[1] = odom_msg->pose().position().y() - 3418220;
//   mp_.curr_posi_[2] = odom_msg->pose().position().z() - 12;
//   //beijing
//   mp_.curr_posi_[0] = odom_msg->pose().position().x() + 67814;
//   mp_.curr_posi_[1] = odom_msg->pose().position().y() - 4457770;
//   mp_.curr_posi_[2] = odom_msg->pose().position().z() - 33;
//   //nanjing
//   mp_.curr_posi_[0] = odom_msg->pose().position().x() - 103962;
//   mp_.curr_posi_[1] = odom_msg->pose().position().y() - 3534899;
//   mp_.curr_posi_[2] = odom_msg->pose().position().z() - 10;

  mp_.curr_q_.w() = odom_msg->pose().orientation().qw();
  mp_.curr_q_.x() = odom_msg->pose().orientation().qx();
  mp_.curr_q_.y() = odom_msg->pose().orientation().qy();
  mp_.curr_q_.z() = odom_msg->pose().orientation().qz();

  mp_.curr_twist_[0] = odom_msg->pose().linear_velocity().x();
  mp_.curr_twist_[1] = odom_msg->pose().linear_velocity().y();
  mp_.curr_twist_[2] = odom_msg->pose().linear_velocity().z();

  Eigen::Vector3d Position_XYZ(odom_msg->pose().position().x(), odom_msg->pose().position().y(), odom_msg->pose().position().z());
  // //svl
  // Eigen::Vector3d Position_XYZ(odom_msg->pose().position().x() - 587061, odom_msg->pose().position().y() - 4141628, odom_msg->pose().position().z());
//   //huzhou 
//   Eigen::Vector3d Position_XYZ(odom_msg->pose().position().x() - 218385, odom_msg->pose().position().y() - 3418220, odom_msg->pose().position().z() - 12);
//   //beijing
//   Eigen::Vector3d Position_XYZ(odom_msg->pose().position().x() + 67814, odom_msg->pose().position().y() - 4457770, odom_msg->pose().position().z() - 33);
//   //nanjing
//   Eigen::Vector3d Position_XYZ(odom_msg->pose().position().x() - 103962, odom_msg->pose().position().y() - 3534899, odom_msg->pose().position().z() - 10);
  
  Eigen::Quaterniond quaternion(odom_msg->pose().orientation().qw(), odom_msg->pose().orientation().qx(), 
                                odom_msg->pose().orientation().qy(), odom_msg->pose().orientation().qz());
  Eigen::Matrix3d Rotation_matrix;
  
  // Rotation_matrix = quaternion.toRotationMatrix();

  Eigen::Quaterniond quaternion_(0.7071, 0, 0, 0.7071);
  Rotation_matrix = quaternion.toRotationMatrix() * quaternion_.toRotationMatrix();

  mp_.center_position_ = Position_XYZ + Rotation_matrix * mp_.lidar2car_;

  std::shared_ptr<drivers::PointCloud> laserCloudTransformed = std::make_shared<drivers::PointCloud>();

  for(auto iter : pcl_msg->point())
  {
      Eigen::Vector3d LaserCloudIn_XYZ(iter.x(), iter.y(), iter.z());
      Eigen::Vector3d LaserCloudTransformed_XYZ = Rotation_matrix * LaserCloudIn_XYZ + mp_.center_position_;

      Eigen::Vector3d pc_position = LaserCloudTransformed_XYZ - mp_.center_position_;
      if(pc_position(2) < mp_.obs_low_ || pc_position(2) > mp_.obs_high_) {
          continue;
      }
      if(pc_position(0) * pc_position(0) + pc_position(1) * pc_position(1) < mp_.obs_circle_) {
          continue;
      }

      PointXYZIT *point = laserCloudTransformed->add_point();
      point->set_x(LaserCloudTransformed_XYZ(0));
      point->set_y(LaserCloudTransformed_XYZ(1));
      point->set_z(LaserCloudTransformed_XYZ(2));
      //point->set_intensity(iter.intensity());
      //point->set_timestamp(iter.timestamp() * 1e9);
  }

  mp_.number_of_points_ = laserCloudTransformed->point().size();

  // set header
  std::shared_ptr<drivers::PointCloud> msg = laserCloudTransformed;
  const auto timestamp = pcl_msg->header().timestamp_sec();
  msg->set_height(1);
  msg->set_width(msg->point_size() / msg->height());
  msg->set_is_dense(false);
  msg->mutable_header()->set_sequence_num(pcl_msg->header().sequence_num());
  msg->mutable_header()->set_frame_id("shenlan_mapping");
  msg->mutable_header()->set_timestamp_sec(timestamp);
  msg->mutable_header()->set_lidar_timestamp(timestamp * 1e9);
  msg->set_measurement_time(timestamp);

  pc_writer_->Write(msg);

  mp_.local_range_min_ = mp_.center_position_ - mp_.sensor_range_;
  mp_.local_range_max_ = mp_.center_position_ + mp_.sensor_range_;
  mp_.raycastProcess(mp_.center_position_, laserCloudTransformed);

  std::shared_ptr<drivers::PointCloud> map_ = std::make_shared<drivers::PointCloud>();
  globalOccPc(map_);
  map_writer_->Write(map_);

  return true;
}
 
void MappingShenlanComponent::globalOccPc(const std::shared_ptr<drivers::PointCloud> &msg)
{
    // for (int x = 0; x < mp_.global_map_size_[0]; ++x)
    // {
    //     for (int y = 0; y < mp_.global_map_size_[1]; ++y)
    //     {
    //         if (mp_.occupancy_buffer_2d_.at(y * mp_.global_map_size_[0] + x) > 0.5)
    //         {
    //             Eigen::Vector2i idx(x, y);
    //             Eigen::Vector2d pos;
    //             mp_.indexToPos2d(idx, pos);
    //             PointXYZIT *point = msg->add_point();
    //             point->set_x(pos[0]);
    //             point->set_y(pos[1]);
    //             point->set_z(0);
    //         }
    //     }
    // }

    for (int x = 0; x < mp_.global_map_size_[0]; ++x)
      for (int y = 0; y < mp_.global_map_size_[1]; ++y)
          for (int z = 0; z < mp_.global_map_size_[2]; ++z)
          {
              if (mp_.occupancy_buffer_[x * mp_.grid_size_y_multiply_z_ + y * mp_.global_map_size_(2) + z] > mp_.min_occupancy_log_)
              {
                  Eigen::Vector3i idx(x, y, z);
                  Eigen::Vector3d pos;
                  mp_.indexToPos(idx, pos);
                  PointXYZIT *point = msg->add_point();
                  point->set_x(pos[0]);
                  point->set_y(pos[1]);
                  point->set_z(pos[2]);
              }
          }

    auto timestamp = Clock::NowInSeconds();
    msg->set_height(1);
    msg->set_width(msg->point_size() / msg->height());
    msg->set_is_dense(false);
    msg->mutable_header()->set_sequence_num(seq_num_map);
    msg->mutable_header()->set_frame_id("map_shenlan");
    msg->mutable_header()->set_timestamp_sec(timestamp);
    msg->mutable_header()->set_lidar_timestamp(timestamp * 1e9);
    msg->set_measurement_time(timestamp);
    seq_num_map += 1;
}
 
}
}
