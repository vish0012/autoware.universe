// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define EIGEN_MPL2_ONLY

#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>

#include <boost/optional.hpp>

#include <cmath>
#include <list>
#include <memory>
#include <string>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

// static int publish_counter = 0;
static double processing_time_ms = 0;

namespace autoware::image_projection_based_fusion
{
using autoware::universe_utils::ScopedTimeTrack;

template <class TargetMsg3D, class ObjType, class Msg2D>
FusionNode<TargetMsg3D, ObjType, Msg2D>::FusionNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // set rois_number
  rois_number_ = static_cast<std::size_t>(declare_parameter<int32_t>("rois_number"));
  if (rois_number_ < 1) {
    RCLCPP_WARN(
      this->get_logger(), "minimum rois_number is 1. current rois_number is %zu", rois_number_);
    rois_number_ = 1;
  }
  if (rois_number_ > 8) {
    RCLCPP_WARN(
      this->get_logger(),
      "Current rois_number is %zu. Large rois number may cause performance issue.", rois_number_);
  }

  // Set parameters
  match_threshold_ms_ = declare_parameter<double>("match_threshold_ms");
  timeout_ms_ = declare_parameter<double>("timeout_ms");

  input_rois_topics_.resize(rois_number_);
  input_camera_topics_.resize(rois_number_);
  input_camera_info_topics_.resize(rois_number_);

  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    input_rois_topics_.at(roi_i) = declare_parameter<std::string>(
      "input/rois" + std::to_string(roi_i),
      "/perception/object_recognition/detection/rois" + std::to_string(roi_i));

    input_camera_info_topics_.at(roi_i) = declare_parameter<std::string>(
      "input/camera_info" + std::to_string(roi_i),
      "/sensing/camera/camera" + std::to_string(roi_i) + "/camera_info");

    input_camera_topics_.at(roi_i) = declare_parameter<std::string>(
      "input/image" + std::to_string(roi_i),
      "/sensing/camera/camera" + std::to_string(roi_i) + "/image_rect_color");
  }

  input_offset_ms_ = declare_parameter<std::vector<double>>("input_offset_ms");
  if (!input_offset_ms_.empty() && rois_number_ > input_offset_ms_.size()) {
    throw std::runtime_error("The number of offsets does not match the number of topics.");
  }

  // sub camera info
  camera_info_subs_.resize(rois_number_);
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    std::function<void(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)> fnc =
      std::bind(&FusionNode::cameraInfoCallback, this, std::placeholders::_1, roi_i);
    camera_info_subs_.at(roi_i) = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      input_camera_info_topics_.at(roi_i), rclcpp::QoS{1}.best_effort(), fnc);
  }

  // sub rois
  rois_subs_.resize(rois_number_);
  cached_roi_msgs_.resize(rois_number_);
  is_fused_.resize(rois_number_, false);
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    std::function<void(const typename Msg2D::ConstSharedPtr msg)> roi_callback =
      std::bind(&FusionNode::roiCallback, this, std::placeholders::_1, roi_i);
    rois_subs_.at(roi_i) = this->create_subscription<Msg2D>(
      input_rois_topics_.at(roi_i), rclcpp::QoS{1}.best_effort(), roi_callback);
  }

  // subscribers
  std::function<void(const typename TargetMsg3D::ConstSharedPtr msg)> sub_callback =
    std::bind(&FusionNode::subCallback, this, std::placeholders::_1);
  sub_ =
    this->create_subscription<TargetMsg3D>("input", rclcpp::QoS(1).best_effort(), sub_callback);

  // publisher
  pub_ptr_ = this->create_publisher<TargetMsg3D>("output", rclcpp::QoS{1});

  // Set timer
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(timeout_ms_));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&FusionNode::timer_callback, this));

  // debugger
  if (declare_parameter("debug_mode", false)) {
    std::size_t image_buffer_size =
      static_cast<std::size_t>(declare_parameter<int32_t>("image_buffer_size"));
    debugger_ =
      std::make_shared<Debugger>(this, rois_number_, image_buffer_size, input_camera_topics_);
  }

  // time keeper
  bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware::universe_utils::TimeKeeper(detailed_processing_time_publisher_);
    time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(time_keeper);
  }

  point_project_to_unrectified_image_ =
    declare_parameter<bool>("point_project_to_unrectified_image");

  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  filter_scope_min_x_ = declare_parameter<double>("filter_scope_min_x");
  filter_scope_max_x_ = declare_parameter<double>("filter_scope_max_x");
  filter_scope_min_y_ = declare_parameter<double>("filter_scope_min_y");
  filter_scope_max_y_ = declare_parameter<double>("filter_scope_max_y");
  filter_scope_min_z_ = declare_parameter<double>("filter_scope_min_z");
  filter_scope_max_z_ = declare_parameter<double>("filter_scope_max_z");
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg,
  const std::size_t camera_id)
{
  camera_info_map_[camera_id] = *input_camera_info_msg;
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::preprocess(TargetMsg3D & ouput_msg
                                                     __attribute__((unused)))
{
  // do nothing by default
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::subCallback(
  const typename TargetMsg3D::ConstSharedPtr input_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (cached_msg_.second != nullptr) {
    stop_watch_ptr_->toc("processing_time", true);
    timer_->cancel();
    postprocess(*(cached_msg_.second));
    publish(*(cached_msg_.second));
    std::fill(is_fused_.begin(), is_fused_.end(), false);

    // add processing time for debug
    if (debug_publisher_) {
      const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
      const double pipeline_latency_ms =
        std::chrono::duration<double, std::milli>(
          std::chrono::nanoseconds(
            (this->get_clock()->now() - cached_msg_.second->header.stamp).nanoseconds()))
          .count();
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/cyclic_time_ms", cyclic_time_ms);
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/processing_time_ms",
        processing_time_ms + stop_watch_ptr_->toc("processing_time", true));
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/pipeline_latency_ms", pipeline_latency_ms);
      processing_time_ms = 0;
    }

    cached_msg_.second = nullptr;
  }

  std::lock_guard<std::mutex> lock(mutex_cached_msgs_);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(timeout_ms_));
  try {
    setPeriod(period.count());
  } catch (rclcpp::exceptions::RCLError & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
  }
  timer_->reset();

  stop_watch_ptr_->toc("processing_time", true);

  typename TargetMsg3D::SharedPtr output_msg = std::make_shared<TargetMsg3D>(*input_msg);

  preprocess(*output_msg);

  int64_t timestamp_nsec =
    (*output_msg).header.stamp.sec * static_cast<int64_t>(1e9) + (*output_msg).header.stamp.nanosec;

  // if matching rois exist, fuseOnSingle
  // please ask maintainers before parallelize this loop because debugger is not thread safe
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    if (camera_info_map_.find(roi_i) == camera_info_map_.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "no camera info. id is %zu", roi_i);
      continue;
    }

    if ((cached_roi_msgs_.at(roi_i)).size() > 0) {
      int64_t min_interval = 1e9;
      int64_t matched_stamp = -1;
      std::list<int64_t> outdate_stamps;

      for (const auto & [k, v] : cached_roi_msgs_.at(roi_i)) {
        int64_t new_stamp = timestamp_nsec + input_offset_ms_.at(roi_i) * static_cast<int64_t>(1e6);
        int64_t interval = abs(static_cast<int64_t>(k) - new_stamp);

        if (
          interval <= min_interval && interval <= match_threshold_ms_ * static_cast<int64_t>(1e6)) {
          min_interval = interval;
          matched_stamp = k;
        } else if (
          static_cast<int64_t>(k) < new_stamp &&
          interval > match_threshold_ms_ * static_cast<int64_t>(1e6)) {
          outdate_stamps.push_back(static_cast<int64_t>(k));
        }
      }

      // remove outdated stamps
      for (auto stamp : outdate_stamps) {
        (cached_roi_msgs_.at(roi_i)).erase(stamp);
      }

      // fuseOnSingle
      if (matched_stamp != -1) {
        if (debugger_) {
          debugger_->clear();
        }

        fuseOnSingleImage(
          *input_msg, roi_i, *((cached_roi_msgs_.at(roi_i))[matched_stamp]),
          camera_info_map_.at(roi_i), *output_msg);
        (cached_roi_msgs_.at(roi_i)).erase(matched_stamp);
        is_fused_.at(roi_i) = true;

        // add timestamp interval for debug
        if (debug_publisher_) {
          double timestamp_interval_ms = (matched_stamp - timestamp_nsec) / 1e6;
          debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
            "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_ms", timestamp_interval_ms);
          debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
            "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_offset_ms",
            timestamp_interval_ms - input_offset_ms_.at(roi_i));
        }
      }
    }
  }

  // if all camera fused, postprocess; else, publish the old Msg(if exists) and cache the current
  // Msg
  if (std::count(is_fused_.begin(), is_fused_.end(), true) == static_cast<int>(rois_number_)) {
    timer_->cancel();
    postprocess(*output_msg);
    publish(*output_msg);
    std::fill(is_fused_.begin(), is_fused_.end(), false);
    cached_msg_.second = nullptr;

    // add processing time for debug
    if (debug_publisher_) {
      const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
      processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/cyclic_time_ms", cyclic_time_ms);
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/processing_time_ms", processing_time_ms);
      processing_time_ms = 0;
    }
  } else {
    cached_msg_.first = timestamp_nsec;
    cached_msg_.second = output_msg;
    processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  }
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::roiCallback(
  const typename Msg2D::ConstSharedPtr input_roi_msg, const std::size_t roi_i)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  stop_watch_ptr_->toc("processing_time", true);

  int64_t timestamp_nsec = (*input_roi_msg).header.stamp.sec * static_cast<int64_t>(1e9) +
                           (*input_roi_msg).header.stamp.nanosec;

  // if cached Msg exist, try to match
  if (cached_msg_.second != nullptr) {
    int64_t new_stamp = cached_msg_.first + input_offset_ms_.at(roi_i) * static_cast<int64_t>(1e6);
    int64_t interval = abs(timestamp_nsec - new_stamp);

    if (
      interval < match_threshold_ms_ * static_cast<int64_t>(1e6) && is_fused_.at(roi_i) == false) {
      if (camera_info_map_.find(roi_i) == camera_info_map_.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000, "no camera info. id is %zu", roi_i);
        (cached_roi_msgs_.at(roi_i))[timestamp_nsec] = input_roi_msg;
        return;
      }
      if (debugger_) {
        debugger_->clear();
      }

      fuseOnSingleImage(
        *(cached_msg_.second), roi_i, *input_roi_msg, camera_info_map_.at(roi_i),
        *(cached_msg_.second));
      is_fused_.at(roi_i) = true;

      if (debug_publisher_) {
        double timestamp_interval_ms = (timestamp_nsec - cached_msg_.first) / 1e6;
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_ms", timestamp_interval_ms);
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_offset_ms",
          timestamp_interval_ms - input_offset_ms_.at(roi_i));
      }

      if (std::count(is_fused_.begin(), is_fused_.end(), true) == static_cast<int>(rois_number_)) {
        timer_->cancel();
        postprocess(*(cached_msg_.second));
        publish(*(cached_msg_.second));
        std::fill(is_fused_.begin(), is_fused_.end(), false);
        cached_msg_.second = nullptr;

        // add processing time for debug
        if (debug_publisher_) {
          const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
          debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
            "debug/cyclic_time_ms", cyclic_time_ms);
          debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
            "debug/processing_time_ms",
            processing_time_ms + stop_watch_ptr_->toc("processing_time", true));
          processing_time_ms = 0;
        }
      }
      processing_time_ms = processing_time_ms + stop_watch_ptr_->toc("processing_time", true);
      return;
    }
  }
  // store roi msg if not matched
  (cached_roi_msgs_.at(roi_i))[timestamp_nsec] = input_roi_msg;
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::postprocess(TargetMsg3D & output_msg
                                                      __attribute__((unused)))
{
  // do nothing by default
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::timer_callback()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  using std::chrono_literals::operator""ms;
  timer_->cancel();
  if (mutex_cached_msgs_.try_lock()) {
    // timeout, postprocess cached msg
    if (cached_msg_.second != nullptr) {
      stop_watch_ptr_->toc("processing_time", true);

      postprocess(*(cached_msg_.second));
      publish(*(cached_msg_.second));

      // add processing time for debug
      if (debug_publisher_) {
        const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug/cyclic_time_ms", cyclic_time_ms);
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug/processing_time_ms",
          processing_time_ms + stop_watch_ptr_->toc("processing_time", true));
        processing_time_ms = 0;
      }
    }
    std::fill(is_fused_.begin(), is_fused_.end(), false);
    cached_msg_.second = nullptr;

    mutex_cached_msgs_.unlock();
  } else {
    try {
      std::chrono::nanoseconds period = 10ms;
      setPeriod(period.count());
    } catch (rclcpp::exceptions::RCLError & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    }
    timer_->reset();
  }
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::setPeriod(const int64_t new_period)
{
  if (!timer_) {
    return;
  }
  int64_t old_period = 0;
  rcl_ret_t ret = rcl_timer_get_period(timer_->get_timer_handle().get(), &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get old period");
  }
  ret = rcl_timer_exchange_period(timer_->get_timer_handle().get(), new_period, &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
  }
}

template <class TargetMsg3D, class Obj, class Msg2D>
void FusionNode<TargetMsg3D, Obj, Msg2D>::publish(const TargetMsg3D & output_msg)
{
  if (pub_ptr_->get_subscription_count() < 1) {
    return;
  }
  pub_ptr_->publish(output_msg);
}

template class FusionNode<DetectedObjects, DetectedObject, DetectedObjectsWithFeature>;
template class FusionNode<
  DetectedObjectsWithFeature, DetectedObjectWithFeature, DetectedObjectsWithFeature>;
template class FusionNode<PointCloud2, DetectedObjects, DetectedObjectsWithFeature>;
template class FusionNode<PointCloud2, DetectedObjectWithFeature, DetectedObjectsWithFeature>;
template class FusionNode<PointCloud2, PointCloud2, Image>;
}  // namespace autoware::image_projection_based_fusion
