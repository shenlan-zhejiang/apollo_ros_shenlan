#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

namespace apollo {
namespace shenlan {

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

class RayCaster
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */)
  {
  }
  ~RayCaster()
  {
  }

  bool setInput(const Eigen::Vector3d& start, const Eigen::Vector3d& end/* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool step(Eigen::Vector3d& ray_pt);
};

}
}
#endif  // RAYCAST_H_