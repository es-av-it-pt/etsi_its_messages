/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=denm_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_denm_DigitalMap_H_
#define	_denm_DigitalMap_H_


#include <etsi_its_denm_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct denm_ReferencePosition;

/* denm_DigitalMap */
typedef struct denm_DigitalMap {
	A_SEQUENCE_OF(struct denm_ReferencePosition) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} denm_DigitalMap_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_denm_DigitalMap;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_denm_coding/denm_ReferencePosition.h"

#endif	/* _denm_DigitalMap_H_ */
#include <etsi_its_denm_coding/asn_internal.h>