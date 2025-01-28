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
 * This DE indicates the positioning technology being used to estimate a geographical position.
 *
 * The value shall be set to:
 * - 0 `noPositioningSolution`  - no positioning solution used,
 * - 1 `sGNSS`                  - Global Navigation Satellite System used,
 * - 2 `dGNSS`                  - Differential GNSS used,
 * - 3 `sGNSSplusDR`            - GNSS and dead reckoning used,
 * - 4 `dGNSSplusDR`            - Differential GNSS and dead reckoning used,
 * - 5 `dR`                     - dead reckoning used,
 * - 6 `manuallyByOperator`     - position set manually by a human operator.
 *
 * @category: GeoReference information
 * @revision: V1.3.1, extension with value 6 added in V2.2.1
 *
PositioningSolutionType ::= ENUMERATED {
    noPositioningSolution (0), 
    sGNSS                 (1), 
    dGNSS                 (2), 
    sGNSSplusDR           (3), 
    dGNSSplusDR           (4), 
    dR                    (5), 
    ...,
    manuallyByOperator    (6)
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_PositioningSolutionType.h>

#ifdef ROS1
#include <etsi_its_denm_ts_msgs/PositioningSolutionType.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/positioning_solution_type.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_PositioningSolutionType(const denm_ts_PositioningSolutionType_t& in, denm_ts_msgs::PositioningSolutionType& out) {
  out.value = in;
}

void toStruct_PositioningSolutionType(const denm_ts_msgs::PositioningSolutionType& in, denm_ts_PositioningSolutionType_t& out) {
  memset(&out, 0, sizeof(denm_ts_PositioningSolutionType_t));
  out = in.value;
}

}
