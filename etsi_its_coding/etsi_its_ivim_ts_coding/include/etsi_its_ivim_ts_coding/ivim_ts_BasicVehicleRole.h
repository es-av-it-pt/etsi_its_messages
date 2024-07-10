/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_BasicVehicleRole_H_
#define	_ivim_ts_BasicVehicleRole_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_BasicVehicleRole {
	ivim_ts_BasicVehicleRole_basicVehicle	= 0,
	ivim_ts_BasicVehicleRole_publicTransport	= 1,
	ivim_ts_BasicVehicleRole_specialTransport	= 2,
	ivim_ts_BasicVehicleRole_dangerousGoods	= 3,
	ivim_ts_BasicVehicleRole_roadWork	= 4,
	ivim_ts_BasicVehicleRole_roadRescue	= 5,
	ivim_ts_BasicVehicleRole_emergency	= 6,
	ivim_ts_BasicVehicleRole_safetyCar	= 7,
	ivim_ts_BasicVehicleRole_none_unknown	= 8,
	ivim_ts_BasicVehicleRole_truck	= 9,
	ivim_ts_BasicVehicleRole_motorcycle	= 10,
	ivim_ts_BasicVehicleRole_roadSideSource	= 11,
	ivim_ts_BasicVehicleRole_police	= 12,
	ivim_ts_BasicVehicleRole_fire	= 13,
	ivim_ts_BasicVehicleRole_ambulance	= 14,
	ivim_ts_BasicVehicleRole_dot	= 15,
	ivim_ts_BasicVehicleRole_transit	= 16,
	ivim_ts_BasicVehicleRole_slowMoving	= 17,
	ivim_ts_BasicVehicleRole_stopNgo	= 18,
	ivim_ts_BasicVehicleRole_cyclist	= 19,
	ivim_ts_BasicVehicleRole_pedestrian	= 20,
	ivim_ts_BasicVehicleRole_nonMotorized	= 21,
	ivim_ts_BasicVehicleRole_military	= 22
	/*
	 * Enumeration is extensible
	 */
} e_ivim_ts_BasicVehicleRole;

/* ivim_ts_BasicVehicleRole */
typedef long	 ivim_ts_BasicVehicleRole_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ivim_ts_BasicVehicleRole_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_BasicVehicleRole;
extern const asn_INTEGER_specifics_t asn_SPC_ivim_ts_BasicVehicleRole_specs_1;
asn_struct_free_f ivim_ts_BasicVehicleRole_free;
asn_struct_print_f ivim_ts_BasicVehicleRole_print;
asn_constr_check_f ivim_ts_BasicVehicleRole_constraint;
per_type_decoder_f ivim_ts_BasicVehicleRole_decode_uper;
per_type_encoder_f ivim_ts_BasicVehicleRole_encode_uper;
per_type_decoder_f ivim_ts_BasicVehicleRole_decode_aper;
per_type_encoder_f ivim_ts_BasicVehicleRole_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_BasicVehicleRole_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
