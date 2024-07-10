/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/input/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_ReceiptOBUId_H_
#define	_ivim_ts_ReceiptOBUId_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_ReceiptOBUId */
typedef OCTET_STRING_t	 ivim_ts_ReceiptOBUId_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_ReceiptOBUId;
asn_struct_free_f ivim_ts_ReceiptOBUId_free;
asn_struct_print_f ivim_ts_ReceiptOBUId_print;
asn_constr_check_f ivim_ts_ReceiptOBUId_constraint;
per_type_decoder_f ivim_ts_ReceiptOBUId_decode_uper;
per_type_encoder_f ivim_ts_ReceiptOBUId_encode_uper;
per_type_decoder_f ivim_ts_ReceiptOBUId_decode_aper;
per_type_encoder_f ivim_ts_ReceiptOBUId_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_ReceiptOBUId_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
