/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/input/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#include "etsi_its_ivim_ts_coding/ivim_ts_VehicleLicencePlateNumber.h"

/*
 * This type is implemented using ivim_ts_LPN,
 * so here we adjust the DEF accordingly.
 */
static const ber_tlv_tag_t asn_DEF_ivim_ts_VehicleLicencePlateNumber_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_TYPE_descriptor_t asn_DEF_ivim_ts_VehicleLicencePlateNumber = {
	"VehicleLicencePlateNumber",
	"VehicleLicencePlateNumber",
	&asn_OP_SEQUENCE,
	asn_DEF_ivim_ts_VehicleLicencePlateNumber_tags_1,
	sizeof(asn_DEF_ivim_ts_VehicleLicencePlateNumber_tags_1)
		/sizeof(asn_DEF_ivim_ts_VehicleLicencePlateNumber_tags_1[0]), /* 1 */
	asn_DEF_ivim_ts_VehicleLicencePlateNumber_tags_1,	/* Same as above */
	sizeof(asn_DEF_ivim_ts_VehicleLicencePlateNumber_tags_1)
		/sizeof(asn_DEF_ivim_ts_VehicleLicencePlateNumber_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_ivim_ts_LPN_1,
	3,	/* Elements count */
	&asn_SPC_ivim_ts_LPN_specs_1	/* Additional specs */
};

