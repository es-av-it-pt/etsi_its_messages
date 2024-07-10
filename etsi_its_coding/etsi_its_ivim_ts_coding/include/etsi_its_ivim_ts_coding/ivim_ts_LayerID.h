/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_LayerID_H_
#define	_ivim_ts_LayerID_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_LayerID */
typedef long	 ivim_ts_LayerID_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ivim_ts_LayerID_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_LayerID;
asn_struct_free_f ivim_ts_LayerID_free;
asn_struct_print_f ivim_ts_LayerID_print;
asn_constr_check_f ivim_ts_LayerID_constraint;
per_type_decoder_f ivim_ts_LayerID_decode_uper;
per_type_encoder_f ivim_ts_LayerID_encode_uper;
per_type_decoder_f ivim_ts_LayerID_decode_aper;
per_type_encoder_f ivim_ts_LayerID_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_LayerID_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
