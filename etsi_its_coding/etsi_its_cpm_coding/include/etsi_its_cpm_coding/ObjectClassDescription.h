/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_cpm_coding/constr_SEQUENCE_OF.h>
#ifndef	_ObjectClassDescription_H_
#define	_ObjectClassDescription_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ObjectClassWithConfidence;

/* ObjectClassDescription */
typedef struct ObjectClassDescription {
	A_SEQUENCE_OF(struct ObjectClassWithConfidence) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ObjectClassDescription_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ObjectClassDescription;
extern asn_SET_OF_specifics_t asn_SPC_ObjectClassDescription_specs_1;
extern asn_TYPE_member_t asn_MBR_ObjectClassDescription_1[1];
extern asn_per_constraints_t asn_PER_type_ObjectClassDescription_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_cpm_coding/ObjectClassWithConfidence.h"

#endif	/* _ObjectClassDescription_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
