/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/input/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_AxleWeightLimits_H_
#define	_ivim_ts_AxleWeightLimits_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_Int2.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_AxleWeightLimits */
typedef struct ivim_ts_AxleWeightLimits {
	ivim_ts_Int2_t	 maxLadenweightOnAxle1;
	ivim_ts_Int2_t	 maxLadenweightOnAxle2;
	ivim_ts_Int2_t	 maxLadenweightOnAxle3;
	ivim_ts_Int2_t	 maxLadenweightOnAxle4;
	ivim_ts_Int2_t	 maxLadenweightOnAxle5;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_AxleWeightLimits_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_AxleWeightLimits;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_AxleWeightLimits_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_AxleWeightLimits_1[5];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_AxleWeightLimits_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
