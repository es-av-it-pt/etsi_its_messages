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

#include <etsi_its_mapem_coding/TimeChangeDetails.h>
#include <etsi_its_mapem_conversion/convertTimeMark.h>
#include <etsi_its_mapem_conversion/convertTimeMark.h>
#include <etsi_its_mapem_conversion/convertTimeMark.h>
#include <etsi_its_mapem_conversion/convertTimeMark.h>
#include <etsi_its_mapem_conversion/convertTimeIntervalConfidence.h>
#include <etsi_its_mapem_conversion/convertTimeMark.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/TimeChangeDetails.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/time_change_details.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_TimeChangeDetails(const TimeChangeDetails_t& in, mapem_msgs::TimeChangeDetails& out) {

  if (in.startTime) {
    toRos_TimeMark(*in.startTime, out.start_time);
    out.start_time_is_present = true;
  }

  toRos_TimeMark(in.minEndTime, out.min_end_time);
  if (in.maxEndTime) {
    toRos_TimeMark(*in.maxEndTime, out.max_end_time);
    out.max_end_time_is_present = true;
  }

  if (in.likelyTime) {
    toRos_TimeMark(*in.likelyTime, out.likely_time);
    out.likely_time_is_present = true;
  }

  if (in.confidence) {
    toRos_TimeIntervalConfidence(*in.confidence, out.confidence);
    out.confidence_is_present = true;
  }

  if (in.nextTime) {
    toRos_TimeMark(*in.nextTime, out.next_time);
    out.next_time_is_present = true;
  }

}

void toStruct_TimeChangeDetails(const mapem_msgs::TimeChangeDetails& in, TimeChangeDetails_t& out) {

  memset(&out, 0, sizeof(TimeChangeDetails_t));

  if (in.start_time_is_present) {
    TimeMark_t start_time;
    toStruct_TimeMark(in.start_time, start_time);
    out.startTime = new TimeMark_t(start_time);
  }

  toStruct_TimeMark(in.min_end_time, out.minEndTime);
  if (in.max_end_time_is_present) {
    TimeMark_t max_end_time;
    toStruct_TimeMark(in.max_end_time, max_end_time);
    out.maxEndTime = new TimeMark_t(max_end_time);
  }

  if (in.likely_time_is_present) {
    TimeMark_t likely_time;
    toStruct_TimeMark(in.likely_time, likely_time);
    out.likelyTime = new TimeMark_t(likely_time);
  }

  if (in.confidence_is_present) {
    TimeIntervalConfidence_t confidence;
    toStruct_TimeIntervalConfidence(in.confidence, confidence);
    out.confidence = new TimeIntervalConfidence_t(confidence);
  }

  if (in.next_time_is_present) {
    TimeMark_t next_time;
    toStruct_TimeMark(in.next_time, next_time);
    out.nextTime = new TimeMark_t(next_time);
  }

}

}