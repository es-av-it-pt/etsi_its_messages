/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeInteger.h>
#ifndef	_OtherSubClass_H_
#define	_OtherSubClass_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum OtherSubClass {
	OtherSubClass_unknown	= 0,
	OtherSubClass_singleObject	= 1,
	OtherSubClass_multipleObjects	= 2,
	OtherSubClass_bulkMaterial	= 3
} e_OtherSubClass;

/* OtherSubClass */
typedef long	 OtherSubClass_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_OtherSubClass_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_OtherSubClass;
asn_struct_free_f OtherSubClass_free;
asn_struct_print_f OtherSubClass_print;
asn_constr_check_f OtherSubClass_constraint;
ber_type_decoder_f OtherSubClass_decode_ber;
der_type_encoder_f OtherSubClass_encode_der;
xer_type_decoder_f OtherSubClass_decode_xer;
xer_type_encoder_f OtherSubClass_encode_xer;
jer_type_encoder_f OtherSubClass_encode_jer;
oer_type_decoder_f OtherSubClass_decode_oer;
oer_type_encoder_f OtherSubClass_encode_oer;
per_type_decoder_f OtherSubClass_decode_uper;
per_type_encoder_f OtherSubClass_encode_uper;
per_type_decoder_f OtherSubClass_decode_aper;
per_type_encoder_f OtherSubClass_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _OtherSubClass_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
