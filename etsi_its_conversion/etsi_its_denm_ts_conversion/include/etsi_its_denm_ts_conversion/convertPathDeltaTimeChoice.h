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
 * This DF represents estimated/predicted travel time between a position and a predefined reference position. 
 *
 * the following options are available:
 * 
 * @field deltaTimeHighPrecision: delta time with precision of 0,1 s.
 *
 * @field deltaTimeBigRange: delta time with precision of 10 s.
 *
 * @category: Basic information
 * @revision: Created in V2.2.1
 *
PathDeltaTimeChoice::= CHOICE {
    deltaTimeHighPrecision	DeltaTimeTenthOfSecond,
    deltaTimeBigRange	    DeltaTimeTenSeconds,
    ...
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_PathDeltaTimeChoice.h>
#include <etsi_its_denm_ts_conversion/convertDeltaTimeTenSeconds.h>
#include <etsi_its_denm_ts_conversion/convertDeltaTimeTenthOfSecond.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/PathDeltaTimeChoice.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/path_delta_time_choice.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_PathDeltaTimeChoice(const denm_ts_PathDeltaTimeChoice_t& in, denm_ts_msgs::PathDeltaTimeChoice& out) {
  switch (in.present) {
  case denm_ts_PathDeltaTimeChoice_PR_deltaTimeHighPrecision:
    toRos_DeltaTimeTenthOfSecond(in.choice.deltaTimeHighPrecision, out.delta_time_high_precision);
    out.choice = denm_ts_msgs::PathDeltaTimeChoice::CHOICE_DELTA_TIME_HIGH_PRECISION;
    break;
  case denm_ts_PathDeltaTimeChoice_PR_deltaTimeBigRange:
    toRos_DeltaTimeTenSeconds(in.choice.deltaTimeBigRange, out.delta_time_big_range);
    out.choice = denm_ts_msgs::PathDeltaTimeChoice::CHOICE_DELTA_TIME_BIG_RANGE;
    break;
  default: break;
  }
}

void toStruct_PathDeltaTimeChoice(const denm_ts_msgs::PathDeltaTimeChoice& in, denm_ts_PathDeltaTimeChoice_t& out) {
  memset(&out, 0, sizeof(denm_ts_PathDeltaTimeChoice_t));
  switch (in.choice) {
  case denm_ts_msgs::PathDeltaTimeChoice::CHOICE_DELTA_TIME_HIGH_PRECISION:
    toStruct_DeltaTimeTenthOfSecond(in.delta_time_high_precision, out.choice.deltaTimeHighPrecision);
    out.present = denm_ts_PathDeltaTimeChoice_PR::denm_ts_PathDeltaTimeChoice_PR_deltaTimeHighPrecision;
    break;
  case denm_ts_msgs::PathDeltaTimeChoice::CHOICE_DELTA_TIME_BIG_RANGE:
    toStruct_DeltaTimeTenSeconds(in.delta_time_big_range, out.choice.deltaTimeBigRange);
    out.present = denm_ts_PathDeltaTimeChoice_PR::denm_ts_PathDeltaTimeChoice_PR_deltaTimeBigRange;
    break;
  default: break;
  }
}

}
