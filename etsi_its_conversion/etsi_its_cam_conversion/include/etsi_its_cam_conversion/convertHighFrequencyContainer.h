/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University
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

#include <etsi_its_cam_coding/HighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertRSUContainerHighFrequency.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/HighFrequencyContainer.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/high_frequency_container.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_HighFrequencyContainer(const etsi_its_cam_coding::HighFrequencyContainer_t& in, cam_msgs::HighFrequencyContainer& out) {
  switch (in.present) {
  case etsi_its_cam_coding::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency:
    toRos_BasicVehicleContainerHighFrequency(in.choice.basicVehicleContainerHighFrequency, out.basic_vehicle_container_high_frequency);
    out.choice = cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
    break;
  case etsi_its_cam_coding::HighFrequencyContainer_PR_rsuContainerHighFrequency:
    toRos_RSUContainerHighFrequency(in.choice.rsuContainerHighFrequency, out.rsu_container_high_frequency);
    out.choice = cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
    break;
  default: break;
  }
}

void toStruct_HighFrequencyContainer(const cam_msgs::HighFrequencyContainer& in, etsi_its_cam_coding::HighFrequencyContainer_t& out) {
  memset(&out, 0, sizeof(etsi_its_cam_coding::HighFrequencyContainer_t));

  switch (in.choice) {
  case cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY:
    toStruct_BasicVehicleContainerHighFrequency(in.basic_vehicle_container_high_frequency, out.choice.basicVehicleContainerHighFrequency);
    out.present = etsi_its_cam_coding::HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    break;
  case cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY:
    toStruct_RSUContainerHighFrequency(in.rsu_container_high_frequency, out.choice.rsuContainerHighFrequency);
    out.present = etsi_its_cam_coding::HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency;
    break;
  default: break;
  }
}

}
