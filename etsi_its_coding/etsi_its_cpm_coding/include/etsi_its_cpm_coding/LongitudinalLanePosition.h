/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_cpm_coding/LongitudinalLanePositionValue.h"
#include "etsi_its_cpm_coding/LongitudinalLanePositionConfidence.h"
#include <etsi_its_cpm_coding/constr_SEQUENCE.h>
#ifndef	_LongitudinalLanePosition_H_
#define	_LongitudinalLanePosition_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LongitudinalLanePosition */
typedef struct LongitudinalLanePosition {
	LongitudinalLanePositionValue_t	 longitudinalLanePositionValue;
	LongitudinalLanePositionConfidence_t	 longitudinalLanePositionConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LongitudinalLanePosition_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LongitudinalLanePosition;
extern asn_SEQUENCE_specifics_t asn_SPC_LongitudinalLanePosition_specs_1;
extern asn_TYPE_member_t asn_MBR_LongitudinalLanePosition_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _LongitudinalLanePosition_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
