/** ============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "etsi_its_cam_ts_msgs/msg/cam.hpp"

#include "displays/CAM/cam_render_object.hpp"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include <rclcpp/rclcpp.hpp>

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{
  class ColorProperty;
  class FloatProperty;
  class RosTopicProperty;
  class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace etsi_its_msgs
{
namespace displays
{

/**
 * @class CAMDisplay
 * @brief Displays an etsi_its_cam_msgs::CAM
 */
class CAMDisplay : public
  rviz_common::RosTopicDisplay<etsi_its_cam_msgs::msg::CAM>
{
  Q_OBJECT

public:
  CAMDisplay();
  ~CAMDisplay() override;

  void onInitialize() override;

  void reset() override;

protected Q_SLOTS:
  void changedCAMViz();
  void changedCAMTSViz();
  void changedCAMTSTopic();

protected:
  // Base-class override so the class is not abstract
  void processMessage(etsi_its_cam_msgs::msg::CAM::ConstSharedPtr msg) override;

  // Unified handler that accepts either CAM (release 1) or CAM TS (release 2)
  void processMessage(const std::variant<
    etsi_its_cam_msgs::msg::CAM,
    etsi_its_cam_ts_msgs::msg::CAM
  > & msg_variant);

  void update(float wall_dt, float ros_dt) override;

  Ogre::ManualObject * manual_object_;

  rclcpp::Node::SharedPtr rviz_node_;

  // CAM (release 1) properties
  rviz_common::properties::BoolProperty *viz_cam_, *show_meta_, *show_station_id_, *show_speed_;
  rviz_common::properties::FloatProperty *buffer_timeout_, *bb_scale_, *char_height_;
  rviz_common::properties::ColorProperty *color_property_, *text_color_property_;

  // CAM TS (release 2) properties
  rviz_common::properties::BoolProperty *viz_cam_ts_, *show_meta_ts_, *show_station_id_ts_, *show_speed_ts_;
  rviz_common::properties::FloatProperty *buffer_timeout_ts_, *bb_scale_ts_, *char_height_ts_;
  rviz_common::properties::ColorProperty *color_property_ts_, *text_color_property_ts_;
  rviz_common::properties::RosTopicProperty *cam_ts_topic_property_;
  rviz_common::properties::QosProfileProperty *cam_ts_qos_property_;
  rclcpp::QoS cam_ts_qos_profile_ = rclcpp::QoS(1);
  rclcpp::Subscription<etsi_its_cam_ts_msgs::msg::CAM>::SharedPtr cam_ts_sub_;

  std::unordered_map<int, CAMRenderObject> cams_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> bboxs_;
  std::vector<std::shared_ptr<rviz_rendering::MovableText>> texts_;

  uint64_t received_ts_ = 0;
};

}  // namespace displays
}  // namespace etsi_its_msgs