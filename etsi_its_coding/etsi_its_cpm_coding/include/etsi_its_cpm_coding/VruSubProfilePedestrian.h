/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeEnumerated.h>
#ifndef	_VruSubProfilePedestrian_H_
#define	_VruSubProfilePedestrian_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VruSubProfilePedestrian {
	VruSubProfilePedestrian_unavailable	= 0,
	VruSubProfilePedestrian_ordinary_pedestrian	= 1,
	VruSubProfilePedestrian_road_worker	= 2,
	VruSubProfilePedestrian_first_responder	= 3,
	VruSubProfilePedestrian_max	= 15
} e_VruSubProfilePedestrian;

/* VruSubProfilePedestrian */
typedef long	 VruSubProfilePedestrian_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_VruSubProfilePedestrian_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_VruSubProfilePedestrian;
extern const asn_INTEGER_specifics_t asn_SPC_VruSubProfilePedestrian_specs_1;
asn_struct_free_f VruSubProfilePedestrian_free;
asn_struct_print_f VruSubProfilePedestrian_print;
asn_constr_check_f VruSubProfilePedestrian_constraint;
ber_type_decoder_f VruSubProfilePedestrian_decode_ber;
der_type_encoder_f VruSubProfilePedestrian_encode_der;
xer_type_decoder_f VruSubProfilePedestrian_decode_xer;
xer_type_encoder_f VruSubProfilePedestrian_encode_xer;
jer_type_encoder_f VruSubProfilePedestrian_encode_jer;
oer_type_decoder_f VruSubProfilePedestrian_decode_oer;
oer_type_encoder_f VruSubProfilePedestrian_encode_oer;
per_type_decoder_f VruSubProfilePedestrian_decode_uper;
per_type_encoder_f VruSubProfilePedestrian_encode_uper;
per_type_decoder_f VruSubProfilePedestrian_decode_aper;
per_type_encoder_f VruSubProfilePedestrian_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _VruSubProfilePedestrian_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
