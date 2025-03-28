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
  asn1/raw/cpm_ts103324/asn/CPM-PDU-Descriptions.asn \
  asn1/raw/cpm_ts103324/asn/CPM-OriginatingStationContainers.asn \
  asn1/raw/cpm_ts103324/asn/CPM-PerceivedObjectContainer.asn \
  asn1/raw/cpm_ts103324/asn/CPM-PerceptionRegionContainer.asn \
  asn1/raw/cpm_ts103324/asn/CPM-SensorInformationContainer.asn \
  asn1/patched/cpm_ts103324/asn/cdd/ETSI-ITS-CDD.asn \
  -t \
  cpm_ts \
  -o \
  etsi_its_conversion/etsi_its_cpm_ts_conversion/include/etsi_its_cpm_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
* 
 * This DF represents the Euler angles which describe the orientation of an object bounding box in a Cartesian coordinate system with an associated confidence level for each angle.
 *
 * It shall include the following components: 
 *
 * @field zAngle: z-angle of object bounding box at the time of measurement, with the associated confidence.
 * The angle is measured with positive values considering the object orientation turning around the z-axis using the right-hand rule, starting from the x-axis. 
 * This extrinsic rotation shall be applied around the centre point of the object bounding box before all other rotations.
 *
 * @field yAngle: optional y-angle of object bounding box at the time of measurement, with the associated confidence.
 * The angle is measured with positive values considering the object orientation turning around the y-axis using the right-hand rule, starting from the z-axis. 
 * This extrinsic rotation shall be applied around the centre point of the object bounding box after the rotation by zAngle and before the rotation by xAngle.
 *
 * @field xAngle: optional x-angle of object bounding box at the time of measurement, with the associated confidence.
 * The angle is measured with positive values considering the object orientation turning around the x-axis using the right-hand rule, starting from the z-axis. 
 * This extrinsic rotation shall be applied around the centre point of the object bounding box after all other rotations.
 * 
 * @category: Basic information
 * @revision: Created in V2.1.1
*
EulerAnglesWithConfidence ::= SEQUENCE {
    zAngle CartesianAngle,
    yAngle CartesianAngle OPTIONAL,
    xAngle CartesianAngle OPTIONAL
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_cpm_ts_coding/cpm_ts_EulerAnglesWithConfidence.h>
#include <etsi_its_cpm_ts_conversion/convertCartesianAngle.h>
#ifdef ROS1
#include <etsi_its_cpm_ts_msgs/EulerAnglesWithConfidence.h>
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs;
#else
#include <etsi_its_cpm_ts_msgs/msg/euler_angles_with_confidence.hpp>
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs::msg;
#endif


namespace etsi_its_cpm_ts_conversion {

void toRos_EulerAnglesWithConfidence(const cpm_ts_EulerAnglesWithConfidence_t& in, cpm_ts_msgs::EulerAnglesWithConfidence& out) {
  toRos_CartesianAngle(in.zAngle, out.z_angle);
  if (in.yAngle) {
    toRos_CartesianAngle(*in.yAngle, out.y_angle);
    out.y_angle_is_present = true;
  }
  if (in.xAngle) {
    toRos_CartesianAngle(*in.xAngle, out.x_angle);
    out.x_angle_is_present = true;
  }
}

void toStruct_EulerAnglesWithConfidence(const cpm_ts_msgs::EulerAnglesWithConfidence& in, cpm_ts_EulerAnglesWithConfidence_t& out) {
  memset(&out, 0, sizeof(cpm_ts_EulerAnglesWithConfidence_t));
  toStruct_CartesianAngle(in.z_angle, out.zAngle);
  if (in.y_angle_is_present) {
    out.yAngle = (cpm_ts_CartesianAngle_t*) calloc(1, sizeof(cpm_ts_CartesianAngle_t));
    toStruct_CartesianAngle(in.y_angle, *out.yAngle);
  }
  if (in.x_angle_is_present) {
    out.xAngle = (cpm_ts_CartesianAngle_t*) calloc(1, sizeof(cpm_ts_CartesianAngle_t));
    toStruct_CartesianAngle(in.x_angle, *out.xAngle);
  }
}

}
