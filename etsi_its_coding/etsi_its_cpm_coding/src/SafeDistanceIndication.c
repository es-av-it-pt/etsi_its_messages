/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_cpm_coding/SafeDistanceIndication.h"

asn_TYPE_member_t asn_MBR_SafeDistanceIndication_1[] = {
	{ ATF_POINTER, 1, offsetof(struct SafeDistanceIndication, subjectStation),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_StationId,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"subjectStation"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SafeDistanceIndication, safeDistanceIndicator),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SafeDistanceIndicator,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"safeDistanceIndicator"
		},
	{ ATF_POINTER, 1, offsetof(struct SafeDistanceIndication, timeToCollision),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DeltaTimeTenthOfSecond,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"timeToCollision"
		},
};
static const int asn_MAP_SafeDistanceIndication_oms_1[] = { 0, 2 };
static const ber_tlv_tag_t asn_DEF_SafeDistanceIndication_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SafeDistanceIndication_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* subjectStation */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* safeDistanceIndicator */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* timeToCollision */
};
asn_SEQUENCE_specifics_t asn_SPC_SafeDistanceIndication_specs_1 = {
	sizeof(struct SafeDistanceIndication),
	offsetof(struct SafeDistanceIndication, _asn_ctx),
	asn_MAP_SafeDistanceIndication_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_SafeDistanceIndication_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SafeDistanceIndication = {
	"SafeDistanceIndication",
	"SafeDistanceIndication",
	&asn_OP_SEQUENCE,
	asn_DEF_SafeDistanceIndication_tags_1,
	sizeof(asn_DEF_SafeDistanceIndication_tags_1)
		/sizeof(asn_DEF_SafeDistanceIndication_tags_1[0]), /* 1 */
	asn_DEF_SafeDistanceIndication_tags_1,	/* Same as above */
	sizeof(asn_DEF_SafeDistanceIndication_tags_1)
		/sizeof(asn_DEF_SafeDistanceIndication_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_SafeDistanceIndication_1,
	3,	/* Elements count */
	&asn_SPC_SafeDistanceIndication_specs_1	/* Additional specs */
};

