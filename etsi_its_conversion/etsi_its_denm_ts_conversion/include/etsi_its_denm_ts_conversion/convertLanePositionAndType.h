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
 * This DF indicates a transversal position in resolution of lanes and other associated details.
 *
 * It shall include the following components: 
 * 
 * @field transversalPosition: the transversal position.
 * 
 * @field laneType: the type of the lane identified in the component transversalPosition. By default set to `traffic`.
 *
 * @field direction: the traffic direction for the lane position relative to a defined reference direction. By default set to `sameDirection`, i.e. following the reference direction.
 *
 * @category Road topology information
 * @revision: direction added in V2.2.1
 *
LanePositionAndType::= SEQUENCE {
    transversalPosition    LanePosition,
    laneType               LaneType DEFAULT traffic,
    direction              Direction DEFAULT sameDirection,
    ...
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_LanePositionAndType.h>
#include <etsi_its_denm_ts_conversion/convertDirection.h>
#include <etsi_its_denm_ts_conversion/convertLanePosition.h>
#include <etsi_its_denm_ts_conversion/convertLaneType.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/LanePositionAndType.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/lane_position_and_type.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_LanePositionAndType(const denm_ts_LanePositionAndType_t& in, denm_ts_msgs::LanePositionAndType& out) {
  toRos_LanePosition(in.transversalPosition, out.transversal_position);
  toRos_LaneType(in.laneType, out.lane_type);
  toRos_Direction(in.direction, out.direction);
}

void toStruct_LanePositionAndType(const denm_ts_msgs::LanePositionAndType& in, denm_ts_LanePositionAndType_t& out) {
  memset(&out, 0, sizeof(denm_ts_LanePositionAndType_t));
  toStruct_LanePosition(in.transversal_position, out.transversalPosition);
  toStruct_LaneType(in.lane_type, out.laneType);
  toStruct_Direction(in.direction, out.direction);
}

}
