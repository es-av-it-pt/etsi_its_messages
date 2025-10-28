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

#include "displays/CAM/cam_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

#include "rviz_common/properties/parse_color.hpp"

namespace etsi_its_msgs
{
namespace displays
{

CAMDisplay::CAMDisplay()
{
  // CAM (release 1) properties
  viz_cam_ = new rviz_common::properties::BoolProperty("Visualize CAMs", true,
    "Show CAM (release 1) messages", this, SLOT(changedCAMViz()));
  buffer_timeout_ = new rviz_common::properties::FloatProperty(
    "Timeout", 0.1f,
    "Time (in s) until objects disappear", viz_cam_);
  buffer_timeout_->setMin(0);
  bb_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 1.0f,
    "Scale of objects", viz_cam_);
  bb_scale_->setMin(0.01);
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 0, 255),
    "Object color", viz_cam_);
  show_meta_ = new rviz_common::properties::BoolProperty("Metadata", true,
    "Show metadata as text next to objects", viz_cam_);
  text_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 0, 255),
    "Text color", show_meta_);
  char_height_ = new rviz_common::properties::FloatProperty("Scale", 4.0, "Scale of text", show_meta_);
  show_station_id_ = new rviz_common::properties::BoolProperty("StationID", true,
    "Show StationID", show_meta_);
  show_speed_ = new rviz_common::properties::BoolProperty("Speed", true,
    "Show speed", show_meta_);

  // CAM TS (release 2) properties
  cam_ts_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "CAM TS Topic", "/etsi_its_conversion/cam_ts/out",
    rosidl_generator_traits::data_type<etsi_its_cam_ts_msgs::msg::CAM>(),
    "Topic of CAM TS (release 2) messages", this, SLOT(changedCAMTSTopic()));
  cam_ts_qos_property_ = new rviz_common::properties::QosProfileProperty(cam_ts_topic_property_, cam_ts_qos_profile_);
  viz_cam_ts_ = new rviz_common::properties::BoolProperty("Visualize CAMs TS", false,
    "Show CAM TS (release 2) messages", this, SLOT(changedCAMTSViz()));
  buffer_timeout_ts_ = new rviz_common::properties::FloatProperty(
    "Timeout", 0.1f,
    "Time (in s) until objects disappear", viz_cam_ts_);
  buffer_timeout_ts_->setMin(0);
  bb_scale_ts_ = new rviz_common::properties::FloatProperty(
    "Scale", 1.0f,
    "Scale of objects", viz_cam_ts_);
  bb_scale_ts_->setMin(0.01);
  color_property_ts_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 0, 255),
    "Object color", viz_cam_ts_);
  show_meta_ts_ = new rviz_common::properties::BoolProperty("Metadata", true,
    "Show metadata as text next to objects", viz_cam_ts_);
  text_color_property_ts_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 0, 255),
    "Text color", show_meta_ts_);
  char_height_ts_ = new rviz_common::properties::FloatProperty("Scale", 4.0, "Scale of text", show_meta_ts_);
  show_station_id_ts_ = new rviz_common::properties::BoolProperty("StationID", true,
    "Show StationID", show_meta_ts_);
  show_speed_ts_ = new rviz_common::properties::BoolProperty("Speed", true,
    "Show speed", show_meta_ts_);
}

CAMDisplay::~CAMDisplay()
{
  if (initialized() ) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void CAMDisplay::onInitialize()
{
  RTDClass::onInitialize();

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();

  // Initialize topic and QoS properties for CAM TS
  cam_ts_topic_property_->initialize(nodeAbstraction);
  cam_ts_qos_property_->initialize(
      [this](rclcpp::QoS profile) {
        cam_ts_qos_profile_ = profile;
        changedCAMTSTopic();
      });

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void CAMDisplay::reset()
{
  RTDClass::reset();
  manual_object_->clear();
  cam_ts_sub_.reset();
}

void CAMDisplay::changedCAMViz() {
  if(!viz_cam_->getBool()) {
    deleteStatus("Topic");
  }
}

void CAMDisplay::changedCAMTSViz() {
  if(!viz_cam_ts_->getBool()) {
    deleteStatus("CAM TS Topic");
    cam_ts_sub_.reset();
  } 
  else changedCAMTSTopic();
}

void CAMDisplay::changedCAMTSTopic() {
  cam_ts_sub_.reset();
  received_ts_ = 0;
  if(cam_ts_topic_property_->isEmpty()) {
    setStatus(
        rviz_common::properties::StatusProperty::Warn,
        "CAM TS Topic",
        QString("Error subscribing: Empty topic name"));
      return;
  }

  std::map<std::string, std::vector<std::string>> published_topics = rviz_node_->get_topic_names_and_types();
  bool topic_available = false;
  std::string topic_message_type;
  for (const auto & topic : published_topics) {
    // Only add topics whose type matches.
    if(topic.first == cam_ts_topic_property_->getTopicStd()) {
      topic_available = true;
      for (const auto & type : topic.second) {
        topic_message_type = type;
        if (type == "etsi_its_cam_ts_msgs/msg/CAM") {
          cam_ts_sub_ = rviz_node_->create_subscription<etsi_its_cam_ts_msgs::msg::CAM>(
            cam_ts_topic_property_->getTopicStd(),
            cam_ts_qos_profile_,
            [this](const etsi_its_cam_ts_msgs::msg::CAM::SharedPtr msg) {
              // Forward release 2 message to unified handler
              this->processMessage(std::variant<etsi_its_cam_msgs::msg::CAM, etsi_its_cam_ts_msgs::msg::CAM>(*msg));
            });
          return;
        }
      }
    }
  }
  if(!topic_available) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "CAM TS Topic",
      QString("Error subscribing to ") + QString::fromStdString(cam_ts_topic_property_->getTopicStd())
      + QString(": Topic is not available!"));
  }
  else {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "CAM TS Topic",
      QString("Error subscribing to ") + QString::fromStdString(cam_ts_topic_property_->getTopicStd())
      + QString(": Message type ") + QString::fromStdString(topic_message_type) + QString::fromStdString(" does not equal etsi_its_cam_ts_msgs::msg::CAM!"));
  }

}

// Base-class override but delegate to the unified handler
void CAMDisplay::processMessage(etsi_its_cam_msgs::msg::CAM::ConstSharedPtr msg)
{
  processMessage(std::variant<etsi_its_cam_msgs::msg::CAM, etsi_its_cam_ts_msgs::msg::CAM>(*msg));
}

// Unified handler used by both release 1 (base-class override delegates to this) and release 2 subscription lambda
void CAMDisplay::processMessage(const std::variant<
    etsi_its_cam_msgs::msg::CAM,
    etsi_its_cam_ts_msgs::msg::CAM
  > & msg_variant)
{
  // Generate CAM render object from message
  rclcpp::Time now = rviz_node_->now();
  uint64_t nanosecs = now.nanoseconds();
  if (nanosecs == 0) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "Topic",
      "Message received before clock got a valid time");
    return;
  }

  CAMRenderObject cam(msg_variant, now, getLeapSecondInsertionsSince2004(static_cast<uint64_t>(now.seconds())));
  if (!cam.validateFloats()) {
    // report error on the proper status entry depending on origin
    if (cam.isTS()) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "CAM TS Topic",
        "Message contained invalid floating point values (nans or infs)");
    } else {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs)");
    }
    return;
  }

  // Check if Station ID is already present in list
  auto it = cams_.find(cam.getStationID());
  if (it != cams_.end()) it->second = cam; // Key exists, update the value
  else cams_.insert(std::make_pair(cam.getStationID(), cam));

  // Update CAM TS message counter and status shown in RViz
  if (cam.isTS()) {
    ++received_ts_;
    setStatus(
      rviz_common::properties::StatusProperty::Ok, "CAM TS Topic", QString::number(received_ts_) + " messages received");
  }

  return;
}

void CAMDisplay::update(float, float)
{
  // Check for outdated CAMs
  for (auto it = cams_.begin(); it != cams_.end(); ) {
    CAMRenderObject & obj = it->second;
    double timeout = obj.isTS() ? buffer_timeout_ts_->getFloat() : buffer_timeout_->getFloat();
    if (obj.getAge(rviz_node_->now()) > timeout) {
      it = cams_.erase(it);
    }
    else {
      ++it;
    }
  }

  // Render all valid cams
  bboxs_.clear();
  texts_.clear();
  for(auto it = cams_.begin(); it != cams_.end(); ++it) {

    CAMRenderObject cam = it->second;

    // Skip rendering depending on the visualize checkboxes
    if (cam.isTS() && !viz_cam_ts_->getBool()) continue;
    if (!cam.isTS() && !viz_cam_->getBool()) continue;

    Ogre::Vector3 sn_position;
    Ogre::Quaternion sn_orientation;
    if (!context_->getFrameManager()->getTransform(cam.getHeader(), sn_position, sn_orientation)) {
      // Check if transform exists
      setMissingTransformToFixedFrame(cam.getHeader().frame_id);
      return;
    }
    // We don't want to use the transform in sn_position and sn_orientation though, because they are only in float precision.
    // So we get the transfrom manually from tf2:
    std::string fixed_frame = fixed_frame_.toStdString();
    geometry_msgs::msg::PoseStamped pose_origin;
    pose_origin.header = cam.getHeader();
    pose_origin.pose.position.x = 0;
    pose_origin.pose.position.y = 0;
    pose_origin.pose.position.z = 0;
    pose_origin.pose.orientation.w = 1;
    pose_origin.pose.orientation.x = 0;
    pose_origin.pose.orientation.y = 0;
    pose_origin.pose.orientation.z = 0;
    geometry_msgs::msg::PoseStamped pose_fixed_frame = context_->getTransformationManager()->getCurrentTransformer()->transform(pose_origin, fixed_frame);
    geometry_msgs::msg::TransformStamped transform_to_fixed_frame;
    transform_to_fixed_frame.header = pose_fixed_frame.header;
    transform_to_fixed_frame.transform.translation.x = pose_fixed_frame.pose.position.x;
    transform_to_fixed_frame.transform.translation.y = pose_fixed_frame.pose.position.y;
    transform_to_fixed_frame.transform.translation.z = pose_fixed_frame.pose.position.z;
    transform_to_fixed_frame.transform.rotation = pose_fixed_frame.pose.orientation;

    setTransformOk();

    // set pose of scene node to origin (=fixed frame)!
    scene_node_->setPosition(Ogre::Vector3{0.0f, 0.0f, 0.0f});
    scene_node_->setOrientation(Ogre::Quaternion{1.0f, 0.0f, 0.0f, 0.0f});

    auto child_scene_node = scene_node_->createChildSceneNode();
    // Set position of scene node to the position relative to the fixed frame
    geometry_msgs::msg::Pose pose = cam.getPose();
    geometry_msgs::msg::Vector3 dimensions = cam.getDimensions();
    tf2::doTransform(pose, pose, transform_to_fixed_frame);
    Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    Ogre::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    if(3 <= cam.getStationType() && cam.getStationType() <= 11)
    {
      // If the station type of the originating ITS-S is set to one out of the values 3 to 11
      // the reference point shall be the ground position of the centre of the front side of
      // the bounding box of the vehicle.
      // https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.03.01_30/en_30263702v010301v.pdf
      tf2::Quaternion q;
      tf2::fromMsg(pose.orientation, q);
      tf2::Matrix3x3 m(q);
      tf2::Vector3 v(-dimensions.x/2.0, 0.0, dimensions.z/2.0);
      v = m*v;
      position.x += v.x();
      position.y += v.y();
      position.z += v.z();
    }

    // set pose of child scene node of bounding-box
    child_scene_node->setPosition(position);
    child_scene_node->setOrientation(orientation);

    // create bounding-box object
    std::shared_ptr<rviz_rendering::Shape> bbox = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, child_scene_node);

    // set the dimensions of bounding box
    Ogre::Vector3 dims;
    double scale = cam.isTS() ? bb_scale_ts_->getFloat() : bb_scale_->getFloat();
    dims.x = dimensions.x*scale;
    dims.y = dimensions.y*scale;
    dims.z = dimensions.z*scale;
    bbox->setScale(dims);
    // set the color of bounding box
    Ogre::ColourValue bb_color = rviz_common::properties::qtToOgre((cam.isTS() ? color_property_ts_->getColor() : color_property_->getColor()));
    bbox->setColor(bb_color);
    bboxs_.push_back(bbox);

    // Visualize meta-information as text
    if((cam.isTS() ? show_meta_ts_->getBool() : show_meta_->getBool())) {
      std::string text;
      if((cam.isTS() ? show_station_id_ts_->getBool() : show_station_id_->getBool())) {
        text+="StationID: " + std::to_string(cam.getStationID());
        text+="\n";
      }
      if((cam.isTS() ? show_speed_ts_->getBool() : show_speed_->getBool())) {
        text+="Speed: " + std::to_string((int)(cam.getSpeed()*3.6)) + " km/h";
      }
      if(!text.size()) continue; // skip adding empty text
      std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", (cam.isTS() ? char_height_ts_->getFloat() : char_height_->getFloat()));
      double height = dims.z;
      height+=text_render->getBoundingRadius();
      Ogre::Vector3 offs(0.0, 0.0, height);
      // There is a bug in rviz_rendering::MovableText::setGlobalTranslation https://github.com/ros2/rviz/issues/974
      text_render->setGlobalTranslation(offs);
      Ogre::ColourValue text_color = rviz_common::properties::qtToOgre((cam.isTS() ? text_color_property_ts_->getColor() : text_color_property_->getColor()));
      text_render->setColor(text_color);
      child_scene_node->attachObject(text_render.get());
      texts_.push_back(text_render);
    }
  }
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::CAMDisplay, rviz_common::Display)