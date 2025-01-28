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
 * This DE represents an angular velocity component described in a local Cartesian coordinate system, per default counted positive in
 * a right-hand local coordinate system from the abscissa.
 *
 * The value shall be set to: 
 * - `-255` if the velocity is equal to or less than -255 degrees/s,
 * - `n` (`n > -255` and `n < 255`) if the velocity is equal to or less than n x 1 degree/s, and greater than (n-1) x 1 degree/s,
 * - `255` if the velocity is greater than 254 degrees/s,
 * - `256` if the information is unavailable.
 *
 * @unit: degree/s
 * @category: Kinematic information
 * @revision: Created in V2.1.1
*
CartesianAngularVelocityComponentValue ::= INTEGER {
    negativeOutofRange (-255),
    positiveOutOfRange (255),
    unavailable	       (256)
} (-255..256)
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_CartesianAngularVelocityComponentValue.h>
#include <etsi_its_denm_ts_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/CartesianAngularVelocityComponentValue.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/cartesian_angular_velocity_component_value.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_CartesianAngularVelocityComponentValue(const denm_ts_CartesianAngularVelocityComponentValue_t& in, denm_ts_msgs::CartesianAngularVelocityComponentValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_CartesianAngularVelocityComponentValue(const denm_ts_msgs::CartesianAngularVelocityComponentValue& in, denm_ts_CartesianAngularVelocityComponentValue_t& out) {
  memset(&out, 0, sizeof(denm_ts_CartesianAngularVelocityComponentValue_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
