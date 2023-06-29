#pragma once

#include <etsi_its_cam_coding/CAM.h>
#include <etsi_its_cam_conversion/convertItsPduHeader.h>
#include <etsi_its_cam_conversion/convertCoopAwareness.h>
#include <etsi_its_cam_msgs/CAM.h>


namespace etsi_its_cam_conversion {

void toRos_CAM(const CAM_t& in, etsi_its_cam_msgs::CAM& out) {

  toRos_ItsPduHeader(in.header, out.header);
  toRos_CoopAwareness(in.cam, out.cam);
}

void toStruct_CAM(const etsi_its_cam_msgs::CAM& in, CAM_t& out) {
    
  memset(&out, 0, sizeof(CAM_t));

  toStruct_ItsPduHeader(in.header, out.header);
  toStruct_CoopAwareness(in.cam, out.cam);
}

}