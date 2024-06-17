/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/BIT_STRING.h>
#ifndef	_MatrixIncludedComponents_H_
#define	_MatrixIncludedComponents_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MatrixIncludedComponents {
	MatrixIncludedComponents_xPosition	= 0,
	MatrixIncludedComponents_yPosition	= 1,
	MatrixIncludedComponents_zPosition	= 2,
	MatrixIncludedComponents_xVelocityOrVelocityMagnitude	= 3,
	MatrixIncludedComponents_yVelocityOrVelocityDirection	= 4,
	MatrixIncludedComponents_zSpeed	= 5,
	MatrixIncludedComponents_xAccelOrAccelMagnitude	= 6,
	MatrixIncludedComponents_yAccelOrAccelDirection	= 7,
	MatrixIncludedComponents_zAcceleration	= 8,
	MatrixIncludedComponents_zAngle	= 9,
	MatrixIncludedComponents_yAngle	= 10,
	MatrixIncludedComponents_xAngle	= 11,
	MatrixIncludedComponents_zAngularVelocity	= 12
} e_MatrixIncludedComponents;

/* MatrixIncludedComponents */
typedef BIT_STRING_t	 MatrixIncludedComponents_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_MatrixIncludedComponents_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_MatrixIncludedComponents;
asn_struct_free_f MatrixIncludedComponents_free;
asn_struct_print_f MatrixIncludedComponents_print;
asn_constr_check_f MatrixIncludedComponents_constraint;
ber_type_decoder_f MatrixIncludedComponents_decode_ber;
der_type_encoder_f MatrixIncludedComponents_encode_der;
xer_type_decoder_f MatrixIncludedComponents_decode_xer;
xer_type_encoder_f MatrixIncludedComponents_encode_xer;
jer_type_encoder_f MatrixIncludedComponents_encode_jer;
oer_type_decoder_f MatrixIncludedComponents_decode_oer;
oer_type_encoder_f MatrixIncludedComponents_encode_oer;
per_type_decoder_f MatrixIncludedComponents_decode_uper;
per_type_encoder_f MatrixIncludedComponents_encode_uper;
per_type_decoder_f MatrixIncludedComponents_decode_aper;
per_type_encoder_f MatrixIncludedComponents_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _MatrixIncludedComponents_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
