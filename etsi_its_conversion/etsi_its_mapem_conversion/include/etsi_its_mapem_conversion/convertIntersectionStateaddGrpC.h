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

#include <etsi_its_mapem_coding/IntersectionState-addGrpC.h>
#include <etsi_its_mapem_conversion/convertPrioritizationResponseList.h>
#ifdef ROS1
#include <etsi_its_mapem_msgs/IntersectionStateaddGrpC.h>
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
#include <etsi_its_mapem_msgs/msg/intersection_state_add_grp_c.hpp>
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


namespace etsi_its_mapem_conversion {

void toRos_IntersectionStateaddGrpC(const IntersectionState_addGrpC_t& in, mapem_msgs::IntersectionStateaddGrpC& out) {

  if (in.activePrioritizations) {
    toRos_PrioritizationResponseList(*in.activePrioritizations, out.active_prioritizations);
    out.active_prioritizations_is_present = true;
  }

}

void toStruct_IntersectionStateaddGrpC(const mapem_msgs::IntersectionStateaddGrpC& in, IntersectionState_addGrpC_t& out) {

  memset(&out, 0, sizeof(IntersectionState_addGrpC_t));

  if (in.active_prioritizations_is_present) {
    PrioritizationResponseList_t active_prioritizations;
    toStruct_PrioritizationResponseList(in.active_prioritizations, active_prioritizations);
    out.activePrioritizations = new PrioritizationResponseList_t(active_prioritizations);
  }

}

}