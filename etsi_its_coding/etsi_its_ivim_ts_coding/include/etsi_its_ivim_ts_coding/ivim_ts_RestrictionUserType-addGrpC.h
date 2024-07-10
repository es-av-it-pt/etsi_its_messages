/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_RestrictionUserType_addGrpC_H_
#define	_ivim_ts_RestrictionUserType_addGrpC_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_EmissionType.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_FuelType.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_RestrictionUserType-addGrpC */
typedef struct ivim_ts_RestrictionUserType_addGrpC {
	ivim_ts_EmissionType_t	*emission;	/* OPTIONAL */
	ivim_ts_FuelType_t	*fuel;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_RestrictionUserType_addGrpC_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_RestrictionUserType_addGrpC;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_RestrictionUserType_addGrpC_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_RestrictionUserType_addGrpC_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_RestrictionUserType_addGrpC_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
