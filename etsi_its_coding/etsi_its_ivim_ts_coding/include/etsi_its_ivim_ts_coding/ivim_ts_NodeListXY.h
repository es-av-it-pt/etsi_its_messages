/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_NodeListXY_H_
#define	_ivim_ts_NodeListXY_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_NodeSetXY.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_ComputedLane.h"
#include <etsi_its_ivim_ts_coding/constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_NodeListXY_PR {
	ivim_ts_NodeListXY_PR_NOTHING,	/* No components present */
	ivim_ts_NodeListXY_PR_nodes,
	ivim_ts_NodeListXY_PR_computed
	/* Extensions may appear below */
	
} ivim_ts_NodeListXY_PR;

/* ivim_ts_NodeListXY */
typedef struct ivim_ts_NodeListXY {
	ivim_ts_NodeListXY_PR present;
	union ivim_ts_NodeListXY_u {
		ivim_ts_NodeSetXY_t	 nodes;
		ivim_ts_ComputedLane_t	 computed;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_NodeListXY_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_NodeListXY;
extern asn_CHOICE_specifics_t asn_SPC_ivim_ts_NodeListXY_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_NodeListXY_1[2];
extern asn_per_constraints_t asn_PER_type_ivim_ts_NodeListXY_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_NodeListXY_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
