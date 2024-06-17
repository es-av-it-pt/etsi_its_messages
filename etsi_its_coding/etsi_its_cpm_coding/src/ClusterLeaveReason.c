/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */

#include "etsi_its_cpm_coding/ClusterLeaveReason.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_type_ClusterLeaveReason_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_ClusterLeaveReason_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  9 }	/* (0..9) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_ClusterLeaveReason_value2enum_1[] = {
	{ 0,	11,	"notProvided" },
	{ 1,	17,	"clusterLeaderLost" },
	{ 2,	24,	"clusterDisbandedByLeader" },
	{ 3,	23,	"outOfClusterBoundingBox" },
	{ 4,	22,	"outOfClusterSpeedRange" },
	{ 5,	21,	"joiningAnotherCluster" },
	{ 6,	13,	"cancelledJoin" },
	{ 7,	10,	"failedJoin" },
	{ 8,	15,	"safetyCondition" },
	{ 15,	3,	"max" }
};
static const unsigned int asn_MAP_ClusterLeaveReason_enum2value_1[] = {
	6,	/* cancelledJoin(6) */
	2,	/* clusterDisbandedByLeader(2) */
	1,	/* clusterLeaderLost(1) */
	7,	/* failedJoin(7) */
	5,	/* joiningAnotherCluster(5) */
	9,	/* max(15) */
	0,	/* notProvided(0) */
	3,	/* outOfClusterBoundingBox(3) */
	4,	/* outOfClusterSpeedRange(4) */
	8	/* safetyCondition(8) */
};
const asn_INTEGER_specifics_t asn_SPC_ClusterLeaveReason_specs_1 = {
	asn_MAP_ClusterLeaveReason_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_ClusterLeaveReason_enum2value_1,	/* N => "tag"; sorted by N */
	10,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_ClusterLeaveReason_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_ClusterLeaveReason = {
	"ClusterLeaveReason",
	"ClusterLeaveReason",
	&asn_OP_NativeEnumerated,
	asn_DEF_ClusterLeaveReason_tags_1,
	sizeof(asn_DEF_ClusterLeaveReason_tags_1)
		/sizeof(asn_DEF_ClusterLeaveReason_tags_1[0]), /* 1 */
	asn_DEF_ClusterLeaveReason_tags_1,	/* Same as above */
	sizeof(asn_DEF_ClusterLeaveReason_tags_1)
		/sizeof(asn_DEF_ClusterLeaveReason_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		&asn_OER_type_ClusterLeaveReason_constr_1,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_ClusterLeaveReason_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_ClusterLeaveReason_specs_1	/* Additional specs */
};

