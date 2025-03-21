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
  asn1/raw/denm_ts103831/DENM-PDU-Descriptions.asn \
  asn1/patched/denm_ts103831/cdd/ETSI-ITS-CDD.asn \
  -t \
  denm_ts \
  -o \
  etsi_its_conversion/etsi_its_denm_ts_conversion/include/etsi_its_denm_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
*
 * This DE represents the value of the sub cause codes of the @ref CauseCode `dangerousSituation` 
 * 
 * The value shall be set to:
 * - 0 - `unavailable`                      - in case information on the type of dangerous situation is unavailable,
 * - 1 - `emergencyElectronicBrakeEngaged`  - in case emergency electronic brake is engaged,
 * - 2 - `preCrashSystemEngaged`            - in case pre-crash system is engaged,
 * - 3 - `espEngaged`                       - in case Electronic Stability Program (ESP) system is engaged,
 * - 4 - `absEngaged`                       - in case Anti-lock Braking System (ABS) is engaged,
 * - 5 - `aebEngaged`                       - in case Autonomous Emergency Braking (AEB) system is engaged,
 * - 6 - `brakeWarningEngaged`              - in case brake warning is engaged,
 * - 7 - `collisionRiskWarningEngaged`      - in case collision risk warning is engaged,
 * - 8-255                                  - reserved for future usage.
 *
 * @category: Traffic information
 * @revision: V1.3.1
 *
DangerousSituationSubCauseCode ::= INTEGER {
    unavailable                     (0), 
    emergencyElectronicBrakeEngaged (1), 
    preCrashSystemEngaged           (2), 
    espEngaged                      (3), 
    absEngaged                      (4), 
    ebEngaged                       (5), 
    brakeWarningEngaged             (6), 
    collisionRiskWarningEngaged     (7)
} (0..255)
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_DangerousSituationSubCauseCode.h>
#include <etsi_its_denm_ts_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/DangerousSituationSubCauseCode.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/dangerous_situation_sub_cause_code.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_DangerousSituationSubCauseCode(const denm_ts_DangerousSituationSubCauseCode_t& in, denm_ts_msgs::DangerousSituationSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_DangerousSituationSubCauseCode(const denm_ts_msgs::DangerousSituationSubCauseCode& in, denm_ts_DangerousSituationSubCauseCode_t& out) {
  memset(&out, 0, sizeof(denm_ts_DangerousSituationSubCauseCode_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
