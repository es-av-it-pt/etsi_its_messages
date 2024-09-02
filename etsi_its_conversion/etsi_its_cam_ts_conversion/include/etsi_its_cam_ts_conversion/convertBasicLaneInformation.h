/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro

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

// --- Auto-generated by asn1ToConversionHeader.py -----------------------------

#pragma once

#include <etsi_its_cam_ts_coding/cam_ts_BasicLaneInformation.h>
#include <etsi_its_cam_ts_conversion/convertDirection.h>
#include <etsi_its_cam_ts_conversion/convertLanePosition.h>
#include <etsi_its_cam_ts_conversion/convertLaneWidth.h>
#include <etsi_its_cam_ts_conversion/convertRoadSectionId.h>
#ifdef ROS1
#include <etsi_its_cam_ts_msgs/BasicLaneInformation.h>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs;
#else
#include <etsi_its_cam_ts_msgs/msg/basic_lane_information.hpp>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs::msg;
#endif


namespace etsi_its_cam_ts_conversion {

void toRos_BasicLaneInformation(const cam_ts_BasicLaneInformation_t& in, cam_ts_msgs::BasicLaneInformation& out) {
  toRos_LanePosition(in.laneNumber, out.lane_number);
  toRos_Direction(in.direction, out.direction);
  if (in.laneWidth) {
    toRos_LaneWidth(*in.laneWidth, out.lane_width);
    out.lane_width_is_present = true;
  }
  if (in.connectingLane) {
    toRos_LanePosition(*in.connectingLane, out.connecting_lane);
    out.connecting_lane_is_present = true;
  }
  if (in.connectingRoadSection) {
    toRos_RoadSectionId(*in.connectingRoadSection, out.connecting_road_section);
    out.connecting_road_section_is_present = true;
  }
}

void toStruct_BasicLaneInformation(const cam_ts_msgs::BasicLaneInformation& in, cam_ts_BasicLaneInformation_t& out) {
  memset(&out, 0, sizeof(cam_ts_BasicLaneInformation_t));

  toStruct_LanePosition(in.lane_number, out.laneNumber);
  toStruct_Direction(in.direction, out.direction);
  if (in.lane_width_is_present) {
    out.laneWidth = (cam_ts_LaneWidth_t*) calloc(1, sizeof(cam_ts_LaneWidth_t));
    toStruct_LaneWidth(in.lane_width, *out.laneWidth);
  }
  if (in.connecting_lane_is_present) {
    out.connectingLane = (cam_ts_LanePosition_t*) calloc(1, sizeof(cam_ts_LanePosition_t));
    toStruct_LanePosition(in.connecting_lane, *out.connectingLane);
  }
  if (in.connecting_road_section_is_present) {
    out.connectingRoadSection = (cam_ts_RoadSectionId_t*) calloc(1, sizeof(cam_ts_RoadSectionId_t));
    toStruct_RoadSectionId(in.connecting_road_section, *out.connectingRoadSection);
  }
}

}