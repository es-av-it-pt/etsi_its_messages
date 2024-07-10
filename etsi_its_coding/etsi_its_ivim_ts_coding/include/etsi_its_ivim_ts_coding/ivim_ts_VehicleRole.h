/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_VehicleRole_H_
#define	_ivim_ts_VehicleRole_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_VehicleRole {
	ivim_ts_VehicleRole_default	= 0,
	ivim_ts_VehicleRole_publicTransport	= 1,
	ivim_ts_VehicleRole_specialTransport	= 2,
	ivim_ts_VehicleRole_dangerousGoods	= 3,
	ivim_ts_VehicleRole_roadWork	= 4,
	ivim_ts_VehicleRole_rescue	= 5,
	ivim_ts_VehicleRole_emergency	= 6,
	ivim_ts_VehicleRole_safetyCar	= 7,
	ivim_ts_VehicleRole_agriculture	= 8,
	ivim_ts_VehicleRole_commercial	= 9,
	ivim_ts_VehicleRole_military	= 10,
	ivim_ts_VehicleRole_roadOperator	= 11,
	ivim_ts_VehicleRole_taxi	= 12,
	ivim_ts_VehicleRole_reserved1	= 13,
	ivim_ts_VehicleRole_reserved2	= 14,
	ivim_ts_VehicleRole_reserved3	= 15
} e_ivim_ts_VehicleRole;

/* ivim_ts_VehicleRole */
typedef long	 ivim_ts_VehicleRole_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ivim_ts_VehicleRole_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_VehicleRole;
extern const asn_INTEGER_specifics_t asn_SPC_ivim_ts_VehicleRole_specs_1;
asn_struct_free_f ivim_ts_VehicleRole_free;
asn_struct_print_f ivim_ts_VehicleRole_print;
asn_constr_check_f ivim_ts_VehicleRole_constraint;
per_type_decoder_f ivim_ts_VehicleRole_decode_uper;
per_type_encoder_f ivim_ts_VehicleRole_encode_uper;
per_type_decoder_f ivim_ts_VehicleRole_decode_aper;
per_type_encoder_f ivim_ts_VehicleRole_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_VehicleRole_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
