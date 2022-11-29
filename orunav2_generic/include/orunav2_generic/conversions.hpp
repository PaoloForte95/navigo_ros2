#pragma once

#include <orunav2_generic/interfaces.hpp>

namespace orunav2_generic {

std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > getStdVectorEigenVector2dFromPoint2dContainerInterface(const Point2dContainerInterface &pts) { 
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ret;
  for (size_t i = 0; i < pts.sizePoint2d(); i++) {
    ret.push_back(pts.getPoint2d(i));
  }
  return ret;
}

}
