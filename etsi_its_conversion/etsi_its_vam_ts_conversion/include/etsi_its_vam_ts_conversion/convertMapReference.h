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

/** Auto-generated by https://github.com/ika-rwth-aachen/etsi_its_messages -----
python3 \
  utils/codegen/codegen-py/asn1ToConversionHeader.py \
  asn1/raw/vam-ts103300_3/VAM-PDU-Descriptions.asn \
  asn1/patched/vam-ts103300_3/cdd/ETSI-ITS-CDD.asn \
  -t \
  vam_ts \
  -o \
  etsi_its_conversion/etsi_its_vam_ts_conversion/include/etsi_its_vam_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
*
 * This DF provides the reference to the information contained in a MAPEM according to ETSI TS 103 301 [15]. 
 *
 * The following options are provided:
 * 
 * @field roadsegment: option that identifies the description of a road segment contained in a MAPEM.
 * 
 * @field intersection: option that identifies the description of an intersection contained in a MAPEM.
 *
 * @category: Road topology information
 * @revision: Created in V2.1.1
 *
MapReference::= CHOICE {
	roadsegment     RoadSegmentReferenceId,
	intersection    IntersectionReferenceId
	}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_vam_ts_coding/vam_ts_MapReference.h>
#include <etsi_its_vam_ts_conversion/convertIntersectionReferenceId.h>
#include <etsi_its_vam_ts_conversion/convertRoadSegmentReferenceId.h>
#ifdef ROS1
#include <etsi_its_vam_ts_msgs/MapReference.h>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs;
#else
#include <etsi_its_vam_ts_msgs/msg/map_reference.hpp>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs::msg;
#endif


namespace etsi_its_vam_ts_conversion {

void toRos_MapReference(const vam_ts_MapReference_t& in, vam_ts_msgs::MapReference& out) {
  switch (in.present) {
  case vam_ts_MapReference_PR_roadsegment:
    toRos_RoadSegmentReferenceId(in.choice.roadsegment, out.roadsegment);
    out.choice = vam_ts_msgs::MapReference::CHOICE_ROADSEGMENT;
    break;
  case vam_ts_MapReference_PR_intersection:
    toRos_IntersectionReferenceId(in.choice.intersection, out.intersection);
    out.choice = vam_ts_msgs::MapReference::CHOICE_INTERSECTION;
    break;
  default: break;
  }
}

void toStruct_MapReference(const vam_ts_msgs::MapReference& in, vam_ts_MapReference_t& out) {
  memset(&out, 0, sizeof(vam_ts_MapReference_t));
  switch (in.choice) {
  case vam_ts_msgs::MapReference::CHOICE_ROADSEGMENT:
    toStruct_RoadSegmentReferenceId(in.roadsegment, out.choice.roadsegment);
    out.present = vam_ts_MapReference_PR::vam_ts_MapReference_PR_roadsegment;
    break;
  case vam_ts_msgs::MapReference::CHOICE_INTERSECTION:
    toStruct_IntersectionReferenceId(in.intersection, out.choice.intersection);
    out.present = vam_ts_MapReference_PR::vam_ts_MapReference_PR_intersection;
    break;
  default: break;
  }
}

}
