/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro

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

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/denm_Traces.h>
#include <etsi_its_denm_coding/denm_PathHistory.h>
#include <etsi_its_denm_conversion/convertPathHistory.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PathHistory.h>
#include <etsi_its_denm_msgs/Traces.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/path_history.hpp>
#include <etsi_its_denm_msgs/msg/traces.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_Traces(const denm_Traces_t& in, denm_msgs::Traces& out) {
  for (int i = 0; i < in.list.count; ++i) {
    denm_msgs::PathHistory el;
    toRos_PathHistory(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_Traces(const denm_msgs::Traces& in, denm_Traces_t& out) {
  memset(&out, 0, sizeof(denm_Traces_t));

  for (int i = 0; i < in.array.size(); ++i) {
    denm_PathHistory_t* el = (denm_PathHistory_t*) calloc(1, sizeof(denm_PathHistory_t));
    toStruct_PathHistory(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
