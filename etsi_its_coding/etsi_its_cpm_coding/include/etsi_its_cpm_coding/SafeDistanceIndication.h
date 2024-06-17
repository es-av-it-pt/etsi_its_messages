/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_cpm_coding/StationId.h"
#include "etsi_its_cpm_coding/SafeDistanceIndicator.h"
#include "etsi_its_cpm_coding/DeltaTimeTenthOfSecond.h"
#include <etsi_its_cpm_coding/constr_SEQUENCE.h>
#ifndef	_SafeDistanceIndication_H_
#define	_SafeDistanceIndication_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SafeDistanceIndication */
typedef struct SafeDistanceIndication {
	StationId_t	*subjectStation;	/* OPTIONAL */
	SafeDistanceIndicator_t	 safeDistanceIndicator;
	DeltaTimeTenthOfSecond_t	*timeToCollision;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SafeDistanceIndication_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SafeDistanceIndication;
extern asn_SEQUENCE_specifics_t asn_SPC_SafeDistanceIndication_specs_1;
extern asn_TYPE_member_t asn_MBR_SafeDistanceIndication_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _SafeDistanceIndication_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
