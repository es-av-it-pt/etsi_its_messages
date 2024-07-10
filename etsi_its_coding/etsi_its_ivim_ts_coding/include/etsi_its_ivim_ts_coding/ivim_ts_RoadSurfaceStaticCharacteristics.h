/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_RoadSurfaceStaticCharacteristics_H_
#define	_ivim_ts_RoadSurfaceStaticCharacteristics_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_FrictionCoefficient.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_MaterialType.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_WearLevel.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_BankingAngle.h"
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_RoadSurfaceStaticCharacteristics */
typedef struct ivim_ts_RoadSurfaceStaticCharacteristics {
	ivim_ts_FrictionCoefficient_t	 frictionCoefficient;
	ivim_ts_MaterialType_t	 material;
	ivim_ts_WearLevel_t	 wear;
	ivim_ts_BankingAngle_t	 avBankingAngle;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_RoadSurfaceStaticCharacteristics_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_RoadSurfaceStaticCharacteristics;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_RoadSurfaceStaticCharacteristics_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_RoadSurfaceStaticCharacteristics_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_RoadSurfaceStaticCharacteristics_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
