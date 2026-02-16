/**
 * @file types.cpp
 * @brief Implementation of core types
 */

#include "vina_slam/core/types.hpp"

namespace vina_slam {
namespace core {

Plane::Plane() {
  plane_var.setZero();
}

} // namespace core
} // namespace vina_slam
