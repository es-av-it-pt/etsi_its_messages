/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University

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

/** Auto-generated by https://github.com/ika-rwth-aachen/etsi_its_messages -----
python3 \
  utils/codegen/codegen-py/asn1ToConversionHeader.py \
  asn1/raw/denm_ts103831/DENM-PDU-Descriptions.asn \
  asn1/patched/denm_ts103831/cdd/ETSI-ITS-CDD.asn \
  -t \
  denm_ts \
  -o \
  etsi_its_conversion/etsi_its_denm_ts_conversion/include/etsi_its_denm_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
*
 * This DF indicates the opening/closure status of the lanes of a carriageway.
 *
 * It shall include the following components: 
 * 
 * @field innerhardShoulderStatus: this information is optional and shall be included if an inner hard shoulder is present and the information is known.
 * It indicates the open/closing status of inner hard shoulder lanes. 
 * 
 * @field outerhardShoulderStatus: this information is optional and shall be included if an outer hard shoulder is present and the information is known.
 * It indicates the open/closing status of outer hard shoulder lanes. 
 * 
 * @field drivingLaneStatus: this information is optional and shall be included if the information is known.
 * It indicates the open/closing status of driving lanes. 
 * For carriageways with more than 13 driving lanes, the drivingLaneStatus component shall not be present.
 * 
 * @category: Infrastructure information, Road topology information
 * @revision: Description revised in V2.1.1
 *
ClosedLanes ::= SEQUENCE {
    innerhardShoulderStatus    HardShoulderStatus OPTIONAL,
    outerhardShoulderStatus    HardShoulderStatus OPTIONAL,
    drivingLaneStatus          DrivingLaneStatus OPTIONAL,
    ...
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_ClosedLanes.h>
#include <etsi_its_denm_ts_conversion/convertDrivingLaneStatus.h>
#include <etsi_its_denm_ts_conversion/convertHardShoulderStatus.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/ClosedLanes.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/closed_lanes.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_ClosedLanes(const denm_ts_ClosedLanes_t& in, denm_ts_msgs::ClosedLanes& out) {
  if (in.innerhardShoulderStatus) {
    toRos_HardShoulderStatus(*in.innerhardShoulderStatus, out.innerhard_shoulder_status);
    out.innerhard_shoulder_status_is_present = true;
  }
  if (in.outerhardShoulderStatus) {
    toRos_HardShoulderStatus(*in.outerhardShoulderStatus, out.outerhard_shoulder_status);
    out.outerhard_shoulder_status_is_present = true;
  }
  if (in.drivingLaneStatus) {
    toRos_DrivingLaneStatus(*in.drivingLaneStatus, out.driving_lane_status);
    out.driving_lane_status_is_present = true;
  }
}

void toStruct_ClosedLanes(const denm_ts_msgs::ClosedLanes& in, denm_ts_ClosedLanes_t& out) {
  memset(&out, 0, sizeof(denm_ts_ClosedLanes_t));
  if (in.innerhard_shoulder_status_is_present) {
    out.innerhardShoulderStatus = (denm_ts_HardShoulderStatus_t*) calloc(1, sizeof(denm_ts_HardShoulderStatus_t));
    toStruct_HardShoulderStatus(in.innerhard_shoulder_status, *out.innerhardShoulderStatus);
  }
  if (in.outerhard_shoulder_status_is_present) {
    out.outerhardShoulderStatus = (denm_ts_HardShoulderStatus_t*) calloc(1, sizeof(denm_ts_HardShoulderStatus_t));
    toStruct_HardShoulderStatus(in.outerhard_shoulder_status, *out.outerhardShoulderStatus);
  }
  if (in.driving_lane_status_is_present) {
    out.drivingLaneStatus = (denm_ts_DrivingLaneStatus_t*) calloc(1, sizeof(denm_ts_DrivingLaneStatus_t));
    toStruct_DrivingLaneStatus(in.driving_lane_status, *out.drivingLaneStatus);
  }
}

}
