/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_SteeringWheelAngle_H_
#define	_ivim_ts_SteeringWheelAngle_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_SteeringWheelAngleValue.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_SteeringWheelAngleConfidence.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_SteeringWheelAngle */
typedef struct ivim_ts_SteeringWheelAngle {
	ivim_ts_SteeringWheelAngleValue_t	 steeringWheelAngleValue;
	ivim_ts_SteeringWheelAngleConfidence_t	 steeringWheelAngleConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_SteeringWheelAngle_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_SteeringWheelAngle;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_SteeringWheelAngle_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
