#pragma once

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"

#include "modules/shenlan/mapping/mapping.h"

namespace apollo {
namespace shenlan {

class MappingShenlanComponent final : public cyber::Component<localization::LocalizationEstimate, drivers::PointCloud> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  MappingShenlanComponent()
      : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

  bool Init() override;
  bool Proc( const std::shared_ptr<localization::LocalizationEstimate> &odm_, const std::shared_ptr<drivers::PointCloud> &pc_) override;

  std::string Name() const { return "mapping_shenlan"; }

 private:
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  apollo::shenlan::ShenlanConf shenlan_conf;

  std::shared_ptr<cyber::Writer<drivers::PointCloud>> pc_writer_;
  std::shared_ptr<cyber::Writer<drivers::PointCloud>> map_writer_;
  MappingProcess mp_;

  int seq_num_map = 0;
  void globalOccPc(const std::shared_ptr<drivers::PointCloud> &msg);

  int last_seq;
};

CYBER_REGISTER_COMPONENT(MappingShenlanComponent);

}  // namespace bridge
}  // namespace apollo
