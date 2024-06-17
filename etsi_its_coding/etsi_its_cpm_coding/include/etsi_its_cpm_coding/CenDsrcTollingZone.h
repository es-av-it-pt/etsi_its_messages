/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_cpm_coding/Latitude.h"
#include "etsi_its_cpm_coding/Longitude.h"
#include "etsi_its_cpm_coding/ProtectedZoneId.h"
#include <etsi_its_cpm_coding/constr_SEQUENCE.h>
#ifndef	_CenDsrcTollingZone_H_
#define	_CenDsrcTollingZone_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CenDsrcTollingZone */
typedef struct CenDsrcTollingZone {
	Latitude_t	 protectedZoneLatitude;
	Longitude_t	 protectedZoneLongitude;
	ProtectedZoneId_t	*cenDsrcTollingZoneId;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CenDsrcTollingZone_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CenDsrcTollingZone;

#ifdef __cplusplus
}
#endif

#endif	/* _CenDsrcTollingZone_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
