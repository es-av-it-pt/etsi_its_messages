/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeInteger.h>
#ifndef	_Direction_H_
#define	_Direction_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum Direction {
	Direction_sameDirection	= 0,
	Direction_oppositeDirection	= 1,
	Direction_bothDirections	= 2,
	Direction_unavailable	= 3
} e_Direction;

/* Direction */
typedef long	 Direction_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Direction;
asn_struct_free_f Direction_free;
asn_struct_print_f Direction_print;
asn_constr_check_f Direction_constraint;
ber_type_decoder_f Direction_decode_ber;
der_type_encoder_f Direction_encode_der;
xer_type_decoder_f Direction_decode_xer;
xer_type_encoder_f Direction_encode_xer;
jer_type_encoder_f Direction_encode_jer;
oer_type_decoder_f Direction_decode_oer;
oer_type_encoder_f Direction_encode_oer;
per_type_decoder_f Direction_decode_uper;
per_type_encoder_f Direction_encode_uper;
per_type_decoder_f Direction_decode_aper;
per_type_encoder_f Direction_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _Direction_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
