#pragma once

#include <etsi_its_cam_coding/YawRate.h>
#include <etsi_its_cam_msgs/YawRate.h>
#include <etsi_its_cam_conversion/convertYawRateValue.h>
#include <etsi_its_cam_conversion/convertYawRateConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::YawRate convert_YawRatetoRos(const YawRate_t& _YawRate_in)
	{
		etsi_its_cam_msgs::YawRate YawRate_out;
		YawRate_out.yawRateValue = convert_YawRateValuetoRos(_YawRate_in.yawRateValue);
		YawRate_out.yawRateConfidence = convert_YawRateConfidencetoRos(_YawRate_in.yawRateConfidence);
		return YawRate_out;
	}
}