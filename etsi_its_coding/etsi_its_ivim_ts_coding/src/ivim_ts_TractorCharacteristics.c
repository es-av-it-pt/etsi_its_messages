/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/input/ISO19321IVIv2.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#include "etsi_its_ivim_ts_coding/ivim_ts_TractorCharacteristics.h"

asn_TYPE_member_t asn_MBR_ivim_ts_TractorCharacteristics_1[] = {
	{ ATF_POINTER, 3, offsetof(struct ivim_ts_TractorCharacteristics, equalTo),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ivim_ts_VehicleCharacteristicsFixValuesList,
		0,
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
			0
		},
		0, 0, /* No default value */
		"equalTo"
		},
	{ ATF_POINTER, 2, offsetof(struct ivim_ts_TractorCharacteristics, notEqualTo),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ivim_ts_VehicleCharacteristicsFixValuesList,
		0,
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
			0
		},
		0, 0, /* No default value */
		"notEqualTo"
		},
	{ ATF_POINTER, 1, offsetof(struct ivim_ts_TractorCharacteristics, ranges),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ivim_ts_VehicleCharacteristicsRangesList,
		0,
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
			0
		},
		0, 0, /* No default value */
		"ranges"
		},
};
static const int asn_MAP_ivim_ts_TractorCharacteristics_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_ivim_ts_TractorCharacteristics_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ivim_ts_TractorCharacteristics_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* equalTo */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* notEqualTo */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* ranges */
};
asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_TractorCharacteristics_specs_1 = {
	sizeof(struct ivim_ts_TractorCharacteristics),
	offsetof(struct ivim_ts_TractorCharacteristics, _asn_ctx),
	asn_MAP_ivim_ts_TractorCharacteristics_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_ivim_ts_TractorCharacteristics_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ivim_ts_TractorCharacteristics = {
	"TractorCharacteristics",
	"TractorCharacteristics",
	&asn_OP_SEQUENCE,
	asn_DEF_ivim_ts_TractorCharacteristics_tags_1,
	sizeof(asn_DEF_ivim_ts_TractorCharacteristics_tags_1)
		/sizeof(asn_DEF_ivim_ts_TractorCharacteristics_tags_1[0]), /* 1 */
	asn_DEF_ivim_ts_TractorCharacteristics_tags_1,	/* Same as above */
	sizeof(asn_DEF_ivim_ts_TractorCharacteristics_tags_1)
		/sizeof(asn_DEF_ivim_ts_TractorCharacteristics_tags_1[0]), /* 1 */
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
	asn_MBR_ivim_ts_TractorCharacteristics_1,
	3,	/* Elements count */
	&asn_SPC_ivim_ts_TractorCharacteristics_specs_1	/* Additional specs */
};

