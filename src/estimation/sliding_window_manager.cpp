/**
 * @file sliding_window_manager.cpp
 * @brief Implementation of sliding window management
 */

#include "vina_slam/estimation/sliding_window_manager.hpp"
#include <malloc.h>

namespace vina_slam {
namespace estimation {

SlidingWindowManager::SlidingWindowManager(const rclcpp::Node::SharedPtr& node_in) : node(node_in) {
}

SlidingWindowManager& SlidingWindowManager::instance(const rclcpp::Node::SharedPtr& node_in) {
  static SlidingWindowManager inst(node_in);
  return inst;
}

SlidingWindowManager& SlidingWindowManager::instance() {
  return instance(rclcpp::Node::SharedPtr());
}

void SlidingWindowManager::initialize(int window_size, int num_threads) {
  win_size = window_size;
  thread_num = num_threads;

  // Reserve space for buffers
  x_buf.reserve(win_size);
  pvec_buf.reserve(win_size);

  // Initialize slide window pools for each thread
  sws.resize(thread_num);
}

bool SlidingWindowManager::addFrame(const core::IMUST& x_curr, core::PVecPtr pptr,
                                    std::deque<std::shared_ptr<sensor_msgs::msg::Imu>>& imus) {
  // Add current state and point cloud to buffers
  win_count++;
  x_buf.push_back(x_curr);
  pvec_buf.push_back(pptr);

  // Create IMU preintegration factor if we have more than one frame
  if (win_count > 1) {
    imu_pre_buf.push_back(new ImuPreintegration(x_buf[win_count - 2].bg, x_buf[win_count - 2].ba));
    imu_pre_buf[win_count - 2]->pushImu(imus);
  }

  return isWindowFull();
}

void SlidingWindowManager::shiftWindow(int mgsize) {
  // Shift state and point cloud buffers
  for (int i = mgsize; i < win_count; i++) {
    x_buf[i - mgsize] = x_buf[i];
    std::swap(pvec_buf[i - mgsize], pvec_buf[i]);
  }

  // Remove old entries from the end
  for (int i = 0; i < mgsize; ++i) {
    x_buf.pop_back();
    pvec_buf.pop_back();

    delete imu_pre_buf.front();
    imu_pre_buf.pop_front();
  }

  win_base += mgsize;
  win_count -= mgsize;
}

void SlidingWindowManager::cleanupSlideWindows() {
  if (sws[0].size() <= kSlideWindowMaxSize)
    return;

  for (int i = 0; i < kSlideWindowBatchDelete; i++) {
    delete sws[0].back();
    sws[0].pop_back();
  }
  malloc_trim(0);
}

void SlidingWindowManager::reset() {
  // Clean up IMU preintegration factors
  for (size_t i = 0; i < imu_pre_buf.size(); i++) {
    delete imu_pre_buf[i];
  }

  // Clear all buffers
  x_buf.clear();
  pvec_buf.clear();
  imu_pre_buf.clear();

  // Re-initialize buffers with proper size
  x_buf.resize(win_size);
  pvec_buf.resize(win_size);
  for (int i = 0; i < win_size; i++) {
    pvec_buf[i].reset(new core::PVec());
  }

  // Reset counters
  win_base = 0;
  win_count = 0;
}

core::IMUST SlidingWindowManager::getCurrentState() const {
  if (win_count > 0 && !x_buf.empty()) {
    return x_buf[win_count - 1];
  }
  return core::IMUST();
}

core::IMUST SlidingWindowManager::getLastState() const {
  if (win_count > 1 && x_buf.size() >= 2) {
    return x_buf[win_count - 2];
  }
  return core::IMUST();
}

} // namespace estimation
} // namespace vina_slam
