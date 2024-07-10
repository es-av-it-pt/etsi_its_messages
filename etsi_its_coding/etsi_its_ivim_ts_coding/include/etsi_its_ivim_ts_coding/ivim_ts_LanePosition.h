/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/input/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#ifndef	_ivim_ts_LanePosition_H_
#define	_ivim_ts_LanePosition_H_


#include <etsi_its_ivim_ts_coding/asn_application.h>

/* Including external dependencies */
#include <etsi_its_ivim_ts_coding/NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ivim_ts_LanePosition {
	ivim_ts_LanePosition_offTheRoad	= -1,
	ivim_ts_LanePosition_hardShoulder	= 0,
	ivim_ts_LanePosition_outermostDrivingLane	= 1,
	ivim_ts_LanePosition_secondLaneFromOutside	= 2
} e_ivim_ts_LanePosition;

/* ivim_ts_LanePosition */
typedef long	 ivim_ts_LanePosition_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ivim_ts_LanePosition_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ivim_ts_LanePosition;
asn_struct_free_f ivim_ts_LanePosition_free;
asn_struct_print_f ivim_ts_LanePosition_print;
asn_constr_check_f ivim_ts_LanePosition_constraint;
per_type_decoder_f ivim_ts_LanePosition_decode_uper;
per_type_encoder_f ivim_ts_LanePosition_encode_uper;
per_type_decoder_f ivim_ts_LanePosition_decode_aper;
per_type_encoder_f ivim_ts_LanePosition_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ivim_ts_LanePosition_H_ */
#include <etsi_its_ivim_ts_coding/asn_internal.h>
