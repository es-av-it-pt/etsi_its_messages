/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include <etsi_its_cpm_coding/NativeInteger.h>
#ifndef	_SignalViolationSubCauseCode_H_
#define	_SignalViolationSubCauseCode_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SignalViolationSubCauseCode {
	SignalViolationSubCauseCode_unavailable	= 0,
	SignalViolationSubCauseCode_stopSignViolation	= 1,
	SignalViolationSubCauseCode_trafficLightViolation	= 2,
	SignalViolationSubCauseCode_turningRegulationViolation	= 3
} e_SignalViolationSubCauseCode;

/* SignalViolationSubCauseCode */
typedef long	 SignalViolationSubCauseCode_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_SignalViolationSubCauseCode_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_SignalViolationSubCauseCode;
asn_struct_free_f SignalViolationSubCauseCode_free;
asn_struct_print_f SignalViolationSubCauseCode_print;
asn_constr_check_f SignalViolationSubCauseCode_constraint;
ber_type_decoder_f SignalViolationSubCauseCode_decode_ber;
der_type_encoder_f SignalViolationSubCauseCode_encode_der;
xer_type_decoder_f SignalViolationSubCauseCode_decode_xer;
xer_type_encoder_f SignalViolationSubCauseCode_encode_xer;
jer_type_encoder_f SignalViolationSubCauseCode_encode_jer;
oer_type_decoder_f SignalViolationSubCauseCode_decode_oer;
oer_type_encoder_f SignalViolationSubCauseCode_encode_oer;
per_type_decoder_f SignalViolationSubCauseCode_decode_uper;
per_type_encoder_f SignalViolationSubCauseCode_encode_uper;
per_type_decoder_f SignalViolationSubCauseCode_decode_aper;
per_type_encoder_f SignalViolationSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SignalViolationSubCauseCode_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
