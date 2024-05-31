/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_denm_coding/PosPillar.h"
#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/constr_SEQUENCE_OF.h>
#ifndef	_PositionOfPillars_H_
#define	_PositionOfPillars_H_


#include <etsi_its_denm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PositionOfPillars */
typedef struct PositionOfPillars {
	A_SEQUENCE_OF(PosPillar_t) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PositionOfPillars_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PositionOfPillars;
extern asn_SET_OF_specifics_t asn_SPC_PositionOfPillars_specs_1;
extern asn_TYPE_member_t asn_MBR_PositionOfPillars_1[1];
extern asn_per_constraints_t asn_PER_type_PositionOfPillars_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _PositionOfPillars_H_ */
#include <etsi_its_denm_coding/asn_internal.h>