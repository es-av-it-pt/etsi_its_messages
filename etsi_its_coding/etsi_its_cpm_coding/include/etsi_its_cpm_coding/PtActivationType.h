/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeInteger.h>
#ifndef	_PtActivationType_H_
#define	_PtActivationType_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PtActivationType {
	PtActivationType_undefinedCodingType	= 0,
	PtActivationType_r09_16CodingType	= 1,
	PtActivationType_vdv_50149CodingType	= 2
} e_PtActivationType;

/* PtActivationType */
typedef long	 PtActivationType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_PtActivationType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_PtActivationType;
asn_struct_free_f PtActivationType_free;
asn_struct_print_f PtActivationType_print;
asn_constr_check_f PtActivationType_constraint;
ber_type_decoder_f PtActivationType_decode_ber;
der_type_encoder_f PtActivationType_encode_der;
xer_type_decoder_f PtActivationType_decode_xer;
xer_type_encoder_f PtActivationType_encode_xer;
jer_type_encoder_f PtActivationType_encode_jer;
oer_type_decoder_f PtActivationType_decode_oer;
oer_type_encoder_f PtActivationType_encode_oer;
per_type_decoder_f PtActivationType_decode_uper;
per_type_encoder_f PtActivationType_encode_uper;
per_type_decoder_f PtActivationType_decode_aper;
per_type_encoder_f PtActivationType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _PtActivationType_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
