/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -fprefix=vam_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_vam_ts_MessageSegmentationInfo_H_
#define	_vam_ts_MessageSegmentationInfo_H_


#include <etsi_its_vam_ts_coding/asn_application.h>

/* Including external dependencies */
#include "etsi_its_vam_ts_coding/vam_ts_CardinalNumber3b.h"
#include "etsi_its_vam_ts_coding/vam_ts_OrdinalNumber3b.h"
#include <etsi_its_vam_ts_coding/constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* vam_ts_MessageSegmentationInfo */
typedef struct vam_ts_MessageSegmentationInfo {
	vam_ts_CardinalNumber3b_t	 totalMsgNo;
	vam_ts_OrdinalNumber3b_t	 thisMsgNo;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} vam_ts_MessageSegmentationInfo_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_vam_ts_MessageSegmentationInfo;

#ifdef __cplusplus
}
#endif

#endif	/* _vam_ts_MessageSegmentationInfo_H_ */
#include <etsi_its_vam_ts_coding/asn_internal.h>