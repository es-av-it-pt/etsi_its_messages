/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeInteger.h>
#ifndef	_PrecipitationIntensity_H_
#define	_PrecipitationIntensity_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PrecipitationIntensity {
	PrecipitationIntensity_outOfRange	= 2000,
	PrecipitationIntensity_unavailable	= 2001
} e_PrecipitationIntensity;

/* PrecipitationIntensity */
typedef long	 PrecipitationIntensity_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PrecipitationIntensity;
asn_struct_free_f PrecipitationIntensity_free;
asn_struct_print_f PrecipitationIntensity_print;
asn_constr_check_f PrecipitationIntensity_constraint;
ber_type_decoder_f PrecipitationIntensity_decode_ber;
der_type_encoder_f PrecipitationIntensity_encode_der;
xer_type_decoder_f PrecipitationIntensity_decode_xer;
xer_type_encoder_f PrecipitationIntensity_encode_xer;
jer_type_encoder_f PrecipitationIntensity_encode_jer;
oer_type_decoder_f PrecipitationIntensity_decode_oer;
oer_type_encoder_f PrecipitationIntensity_encode_oer;
per_type_decoder_f PrecipitationIntensity_decode_uper;
per_type_encoder_f PrecipitationIntensity_encode_uper;
per_type_decoder_f PrecipitationIntensity_decode_aper;
per_type_encoder_f PrecipitationIntensity_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _PrecipitationIntensity_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
