/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include <etsi_its_spatem_coding/Connection.h>
#include <etsi_its_spatem_conversion/convertConnectingLane.h>
#include <etsi_its_spatem_conversion/convertIntersectionReferenceID.h>
#include <etsi_its_spatem_conversion/convertSignalGroupID.h>
#include <etsi_its_spatem_conversion/convertRestrictionClassID.h>
#include <etsi_its_spatem_conversion/convertLaneConnectionID.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/Connection.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/connection.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_Connection(const Connection_t& in, spatem_msgs::Connection& out) {

  toRos_ConnectingLane(in.connectingLane, out.connecting_lane);
  if (in.remoteIntersection) {
    toRos_IntersectionReferenceID(*in.remoteIntersection, out.remote_intersection);
    out.remote_intersection_is_present = true;
  }

  if (in.signalGroup) {
    toRos_SignalGroupID(*in.signalGroup, out.signal_group);
    out.signal_group_is_present = true;
  }

  if (in.userClass) {
    toRos_RestrictionClassID(*in.userClass, out.user_class);
    out.user_class_is_present = true;
  }

  if (in.connectionID) {
    toRos_LaneConnectionID(*in.connectionID, out.connection_id);
    out.connection_id_is_present = true;
  }

}

void toStruct_Connection(const spatem_msgs::Connection& in, Connection_t& out) {

  memset(&out, 0, sizeof(Connection_t));

  toStruct_ConnectingLane(in.connecting_lane, out.connectingLane);
  if (in.remote_intersection_is_present) {
    IntersectionReferenceID_t remote_intersection;
    toStruct_IntersectionReferenceID(in.remote_intersection, remote_intersection);
    out.remoteIntersection = new IntersectionReferenceID_t(remote_intersection);
  }

  if (in.signal_group_is_present) {
    SignalGroupID_t signal_group;
    toStruct_SignalGroupID(in.signal_group, signal_group);
    out.signalGroup = new SignalGroupID_t(signal_group);
  }

  if (in.user_class_is_present) {
    RestrictionClassID_t user_class;
    toStruct_RestrictionClassID(in.user_class, user_class);
    out.userClass = new RestrictionClassID_t(user_class);
  }

  if (in.connection_id_is_present) {
    LaneConnectionID_t connection_id;
    toStruct_LaneConnectionID(in.connection_id, connection_id);
    out.connectionID = new LaneConnectionID_t(connection_id);
  }

}

}