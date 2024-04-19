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

#include <etsi_its_mapem_coding/SignalRequestMessage.h>
#include <etsi_its_mapem_conversion/convertMinuteOfTheYear.h>
#include <etsi_its_mapem_conversion/convertDSecond.h>
#include <etsi_its_mapem_conversion/convertMsgCount.h>
#include <etsi_its_mapem_conversion/convertSignalRequestList.h>
#include <etsi_its_mapem_conversion/convertRequestorDescription.h>
#include <etsi_its_mapem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/SignalRequestMessage.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/signal_request_message.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_SignalRequestMessage(const SignalRequestMessage_t& in, mapem_msgs::SignalRequestMessage& out) {

  if (in.timeStamp) {
    toRos_MinuteOfTheYear(*in.timeStamp, out.time_stamp);
    out.time_stamp_is_present = true;
  }

  toRos_DSecond(in.second, out.second);
  if (in.sequenceNumber) {
    toRos_MsgCount(*in.sequenceNumber, out.sequence_number);
    out.sequence_number_is_present = true;
  }

  if (in.requests) {
    toRos_SignalRequestList(*in.requests, out.requests);
    out.requests_is_present = true;
  }

  toRos_RequestorDescription(in.requestor, out.requestor);
  if (in.regional) {
    // TODO: toRos_RegionalExtension(*in.regional, out.regional);
    out.regional_is_present = true;
  }

}

void toStruct_SignalRequestMessage(const mapem_msgs::SignalRequestMessage& in, SignalRequestMessage_t& out) {

  memset(&out, 0, sizeof(SignalRequestMessage_t));

  if (in.time_stamp_is_present) {
    MinuteOfTheYear_t time_stamp;
    toStruct_MinuteOfTheYear(in.time_stamp, time_stamp);
    out.timeStamp = new MinuteOfTheYear_t(time_stamp);
  }

  toStruct_DSecond(in.second, out.second);
  if (in.sequence_number_is_present) {
    MsgCount_t sequence_number;
    toStruct_MsgCount(in.sequence_number, sequence_number);
    out.sequenceNumber = new MsgCount_t(sequence_number);
  }

  if (in.requests_is_present) {
    SignalRequestList_t requests;
    toStruct_SignalRequestList(in.requests, requests);
    out.requests = new SignalRequestList_t(requests);
  }

  toStruct_RequestorDescription(in.requestor, out.requestor);
  if (in.regional_is_present) {
    RegionalExtension_364P0_t regional;
    // TODO: toStruct_RegionalExtension(in.regional, regional);
    // TODO: out.regional = new RegionalExtension_364P0_t(regional);
  }

}

}