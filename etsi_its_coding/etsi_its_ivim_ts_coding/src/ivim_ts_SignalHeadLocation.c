/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/input/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -fcompound-names -fprefix=ivim_ts_ -no-gen-BER -no-gen-XER -no-gen-JER -no-gen-OER -no-gen-example -gen-UPER`
 */

#include "etsi_its_ivim_ts_coding/ivim_ts_SignalHeadLocation.h"

asn_TYPE_member_t asn_MBR_ivim_ts_SignalHeadLocation_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ivim_ts_SignalHeadLocation, nodeXY),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_ivim_ts_NodeOffsetPointXY,
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
		"nodeXY"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ivim_ts_SignalHeadLocation, nodeZ),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ivim_ts_DeltaAltitude,
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
		"nodeZ"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ivim_ts_SignalHeadLocation, signalGroupID),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ivim_ts_SignalGroupID,
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
		"signalGroupID"
		},
};
static const ber_tlv_tag_t asn_DEF_ivim_ts_SignalHeadLocation_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ivim_ts_SignalHeadLocation_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* nodeXY */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* nodeZ */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* signalGroupID */
};
asn_SEQUENCE_specifics_t asn_SPC_ivim_ts_SignalHeadLocation_specs_1 = {
	sizeof(struct ivim_ts_SignalHeadLocation),
	offsetof(struct ivim_ts_SignalHeadLocation, _asn_ctx),
	asn_MAP_ivim_ts_SignalHeadLocation_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ivim_ts_SignalHeadLocation = {
	"SignalHeadLocation",
	"SignalHeadLocation",
	&asn_OP_SEQUENCE,
	asn_DEF_ivim_ts_SignalHeadLocation_tags_1,
	sizeof(asn_DEF_ivim_ts_SignalHeadLocation_tags_1)
		/sizeof(asn_DEF_ivim_ts_SignalHeadLocation_tags_1[0]), /* 1 */
	asn_DEF_ivim_ts_SignalHeadLocation_tags_1,	/* Same as above */
	sizeof(asn_DEF_ivim_ts_SignalHeadLocation_tags_1)
		/sizeof(asn_DEF_ivim_ts_SignalHeadLocation_tags_1[0]), /* 1 */
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
	asn_MBR_ivim_ts_SignalHeadLocation_1,
	3,	/* Elements count */
	&asn_SPC_ivim_ts_SignalHeadLocation_specs_1	/* Additional specs */
};

