/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/input/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_CreditRq_H_
#define	_ivim_ts_CreditRq_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_ivim_ts_coding/ivim_ts_PaymentFee.h"
#include <etsi_its_ivim_ts_coding/OCTET_STRING.h>
#include <etsi_its_ivim_ts_coding/NativeInteger.h>
#include <etsi_its_ivim_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivim_ts_CreditRq */
typedef struct ivim_ts_CreditRq {
	ivim_ts_PaymentFee_t	 refund;
	OCTET_STRING_t	 nonce;
	long	 key;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ivim_ts_CreditRq_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_CreditRq;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_CreditRq_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
