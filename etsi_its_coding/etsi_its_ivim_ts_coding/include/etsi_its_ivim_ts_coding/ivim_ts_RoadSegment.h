/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_RoadSegment_H_
#define	_ivim_ts_RoadSegment_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_DescriptiveName.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RoadSegmentReferenceID.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_MsgCount.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_Position3D.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_LaneWidth.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RoadLaneSetList.h"
#include <etsi_its_ivim_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ivim_ts_SpeedLimitList;
struct ivim_ts_Reg_RoadSegment;

/* ivim_ts_RoadSegment */
typedef struct ivim_ts_RoadSegment {
	ivim_ts_DescriptiveName_t	*name;	/* OPTIONAL */
	ivim_ts_RoadSegmentReferenceID_t	 id;
	ivim_ts_MsgCount_t	 revision;
	ivim_ts_Position3D_t	 refPoint;
	ivim_ts_LaneWidth_t	*laneWidth;	/* OPTIONAL */
	struct ivim_ts_SpeedLimitList	*speedLimits;	/* OPTIONAL */
	ivim_ts_RoadLaneSetList_t	 roadLaneSet;
	struct ivim_ts_RoadSegment__regional {
		A_SEQUENCE_OF(struct ivim_ts_Reg_RoadSegment) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_RoadSegment_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_RoadSegment;
extern asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_RoadSegment_specs_1;
extern asn_TYPE_member_t asn_MBR_ivim_ts_RoadSegment_1[8];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_ivim_ts_coding/ivim_ts_SpeedLimitList.h"
#include "etsi_its_ivim_ts_coding/ivim_ts_RegionalExtension.h"

#endif	/* _ivim_ts_RoadSegment_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
