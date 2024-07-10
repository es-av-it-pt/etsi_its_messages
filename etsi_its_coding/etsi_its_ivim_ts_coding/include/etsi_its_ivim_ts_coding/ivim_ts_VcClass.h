/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_VcClass_H_
#define	_ivim_ts_VcClass_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_VcClass {
	ivim_ts_VcClass_classA	= 0,
	ivim_ts_VcClass_classB	= 1,
	ivim_ts_VcClass_classC	= 2,
	ivim_ts_VcClass_classD	= 3,
	ivim_ts_VcClass_classE	= 4,
	ivim_ts_VcClass_classF	= 5,
	ivim_ts_VcClass_classG	= 6,
	ivim_ts_VcClass_classH	= 7
} e_ivim_ts_VcClass;

/* ivim_ts_VcClass */
typedef long	 ivim_ts_VcClass_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ivim_ts_VcClass_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_VcClass;
asn_struct_free_f ivim_ts_VcClass_free;
asn_struct_print_f ivim_ts_VcClass_print;
asn_constr_check_f ivim_ts_VcClass_constraint;
per_type_decoder_f ivim_ts_VcClass_decode_uper;
per_type_encoder_f ivim_ts_VcClass_encode_uper;
per_type_decoder_f ivim_ts_VcClass_decode_aper;
per_type_encoder_f ivim_ts_VcClass_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_VcClass_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
