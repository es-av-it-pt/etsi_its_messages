/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_cpm_coding/LongitudinalAccelerationValue.h"
#include "etsi_its_cpm_coding/AccelerationConfidence.h"
#include <etsi_its_cpm_coding/constr_SEQUENCE.h>
#ifndef	_LongitudinalAcceleration_H_
#define	_LongitudinalAcceleration_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LongitudinalAcceleration */
typedef struct LongitudinalAcceleration {
	LongitudinalAccelerationValue_t	 longitudinalAccelerationValue;
	AccelerationConfidence_t	 longitudinalAccelerationConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LongitudinalAcceleration_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LongitudinalAcceleration;

#ifdef __cplusplus
}
#endif

#endif	/* _LongitudinalAcceleration_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
