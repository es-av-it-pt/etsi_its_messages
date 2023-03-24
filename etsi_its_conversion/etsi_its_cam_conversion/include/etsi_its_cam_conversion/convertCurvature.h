#pragma once

#include <etsi_its_cam_coding/Curvature.h>
#include <etsi_its_cam_msgs/Curvature.h>
#include <etsi_its_cam_conversion/convertCurvatureValue.h>
#include <etsi_its_cam_conversion/convertCurvatureConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Curvature convert_CurvaturetoRos(const Curvature_t& _Curvature_in)
	{
		etsi_its_cam_msgs::Curvature Curvature_out;
		Curvature_out.curvatureValue = convert_CurvatureValuetoRos(_Curvature_in.curvatureValue);
		Curvature_out.curvatureConfidence = convert_CurvatureConfidencetoRos(_Curvature_in.curvatureConfidence);
		return Curvature_out;
	}
}