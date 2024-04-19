/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_spatem_coding/VehicleID.h"
#include "etsi_its_spatem_coding/DescriptiveName.h"
#include "etsi_its_spatem_coding/TransitVehicleStatus.h"
#include "etsi_its_spatem_coding/TransitVehicleOccupancy.h"
#include "etsi_its_spatem_coding/DeltaTime.h"
#include <etsi_its_spatem_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_spatem_coding/constr_SEQUENCE_OF.h>
#include <etsi_its_spatem_coding/constr_SEQUENCE.h>
#ifndef	_RequestorDescription_H_
#define	_RequestorDescription_H_


#include <etsi_its_spatem_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RequestorType;
struct RequestorPositionVector;
struct Reg_RequestorDescription;

/* RequestorDescription */
typedef struct RequestorDescription {
	VehicleID_t	 id;
	struct RequestorType	*type;	/* OPTIONAL */
	struct RequestorPositionVector	*position;	/* OPTIONAL */
	DescriptiveName_t	*name;	/* OPTIONAL */
	DescriptiveName_t	*routeName;	/* OPTIONAL */
	TransitVehicleStatus_t	*transitStatus;	/* OPTIONAL */
	TransitVehicleOccupancy_t	*transitOccupancy;	/* OPTIONAL */
	DeltaTime_t	*transitSchedule;	/* OPTIONAL */
	struct RequestorDescription__regional {
		A_SEQUENCE_OF(struct Reg_RequestorDescription) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RequestorDescription_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RequestorDescription;
extern asn_SEQUENCE_specifics_t asn_SPC_RequestorDescription_specs_1;
extern asn_TYPE_member_t asn_MBR_RequestorDescription_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "etsi_its_spatem_coding/RequestorType.h"
#include "etsi_its_spatem_coding/RequestorPositionVector.h"
#include "etsi_its_spatem_coding/RegionalExtension.h"

#endif	/* _RequestorDescription_H_ */
#include <etsi_its_spatem_coding/asn_internal.h>