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
 * This type represents the Location Container.
 *
 * It shall include the following components: 
 *
 * @field eventSpeed: optional speed of a detected dynamic event and the confidence of the speed information. 
 *
 * @field eventPositionHeading: the optional heading of a dynamic event and the confidence of the heading information.
 *
 * @field detectionZonesToEventPosition: the detection zone information approaching the event position, see clause 6.1.3.3.
 *
 * @field roadType: the optional road type information at the event position. 
 *
 * @field lanePositions: the optional lane(s) where the event is located, at the position indicated by the component eventPosition 
 * of the Management container and for a given reference direction.
 *
 * @field occupiedLanes: the optional lane(s) that are fully or partially occupied by the event, at the position indicated by the 
 * component eventPosition of the Management container and for a given reference direction.
 *
 * @field linkedIvims: the optional list of DF IvimReference, pointing to IVIMs that are semantically connected because providing information 
 * applying to the road segment(s) covered by the components detectionZonesToEventPosition, detectionZonesToSpecifiedEventPoint and 
 * the SituationContainer component eventZone.
 *
 * @field linkedMapem: the optional list of DF Mapreference, pointing to MAPEMs that are semantically connected because providing information 
 * applying to the road segment(s) covered by the component detectionZonesToEventPosition, detectionZonesToSpecifiedEventPoint and 
 * the SituationContainer component eventZone.
 *
 * @field detectionZonesToSpecifiedEventPoint: the optional detection zone information approaching towards a 
 * specified event point, see clause 6.1.3.3. 
 *
 * @field predictedPaths: the optional list of future paths or trajectories that the event may move along or zones that the event may occupy. 
 *
*
LocationContainer ::= SEQUENCE {
    eventSpeed                            Speed OPTIONAL,
    eventPositionHeading                  Wgs84Angle OPTIONAL,
    detectionZonesToEventPosition         Traces,
    roadType                              RoadType OPTIONAL,
    ...,
[[	lanePositions                         GeneralizedLanePositions OPTIONAL,
    occupiedLanes                         OccupiedLanesWithConfidence OPTIONAL,
    linkedIvims                           IvimReferences OPTIONAL, 
    linkedMapems                          MapReferences OPTIONAL, 
    detectionZonesToSpecifiedEventPoint   TracesExtended OPTIONAL,
    predictedPaths	          	          PathPredictedList OPTIONAL ]]
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_LocationContainer.h>
#include <etsi_its_denm_ts_conversion/convertGeneralizedLanePositions.h>
#include <etsi_its_denm_ts_conversion/convertIvimReferences.h>
#include <etsi_its_denm_ts_conversion/convertMapReferences.h>
#include <etsi_its_denm_ts_conversion/convertOccupiedLanesWithConfidence.h>
#include <etsi_its_denm_ts_conversion/convertPathPredictedList.h>
#include <etsi_its_denm_ts_conversion/convertRoadType.h>
#include <etsi_its_denm_ts_conversion/convertSpeed.h>
#include <etsi_its_denm_ts_conversion/convertTraces.h>
#include <etsi_its_denm_ts_conversion/convertTracesExtended.h>
#include <etsi_its_denm_ts_conversion/convertWgs84Angle.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/LocationContainer.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/location_container.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_LocationContainer(const denm_ts_LocationContainer_t& in, denm_ts_msgs::LocationContainer& out) {
  if (in.eventSpeed) {
    toRos_Speed(*in.eventSpeed, out.event_speed);
    out.event_speed_is_present = true;
  }
  if (in.eventPositionHeading) {
    toRos_Wgs84Angle(*in.eventPositionHeading, out.event_position_heading);
    out.event_position_heading_is_present = true;
  }
  toRos_Traces(in.detectionZonesToEventPosition, out.detection_zones_to_event_position);
  if (in.roadType) {
    toRos_RoadType(*in.roadType, out.road_type);
    out.road_type_is_present = true;
  }
  toRos_GeneralizedLanePositions(in.lanePositions, out.lane_positions);
  toRos_OccupiedLanesWithConfidence(in.occupiedLanes, out.occupied_lanes);
  toRos_IvimReferences(in.linkedIvims, out.linked_ivims);
  toRos_MapReferences(in.linkedMapems, out.linked_mapems);
  toRos_TracesExtended(in.detectionZonesToSpecifiedEventPoint, out.detection_zones_to_specified_event_point);
  toRos_PathPredictedList(in.predictedPaths, out.predicted_paths);
}

void toStruct_LocationContainer(const denm_ts_msgs::LocationContainer& in, denm_ts_LocationContainer_t& out) {
  memset(&out, 0, sizeof(denm_ts_LocationContainer_t));
  if (in.event_speed_is_present) {
    out.eventSpeed = (denm_ts_Speed_t*) calloc(1, sizeof(denm_ts_Speed_t));
    toStruct_Speed(in.event_speed, *out.eventSpeed);
  }
  if (in.event_position_heading_is_present) {
    out.eventPositionHeading = (denm_ts_Wgs84Angle_t*) calloc(1, sizeof(denm_ts_Wgs84Angle_t));
    toStruct_Wgs84Angle(in.event_position_heading, *out.eventPositionHeading);
  }
  toStruct_Traces(in.detection_zones_to_event_position, out.detectionZonesToEventPosition);
  if (in.road_type_is_present) {
    out.roadType = (denm_ts_RoadType_t*) calloc(1, sizeof(denm_ts_RoadType_t));
    toStruct_RoadType(in.road_type, *out.roadType);
  }
  toStruct_GeneralizedLanePositions(in.lane_positions, out.lanePositions);
  toStruct_OccupiedLanesWithConfidence(in.occupied_lanes, out.occupiedLanes);
  toStruct_IvimReferences(in.linked_ivims, out.linkedIvims);
  toStruct_MapReferences(in.linked_mapems, out.linkedMapems);
  toStruct_TracesExtended(in.detection_zones_to_specified_event_point, out.detectionZonesToSpecifiedEventPoint);
  toStruct_PathPredictedList(in.predicted_paths, out.predictedPaths);
}

}
