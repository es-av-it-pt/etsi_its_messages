/*
 * Generated by asn1c-0.9.29-DF (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "/input/ETSI-ITS-CDD.asn"
 * 	`asn1c -fcompound-names -no-gen-example -gen-UPER`
 */


/* Including external dependencies */
#include "etsi_its_cpm_coding/SubCauseCodeType.h"
#include "etsi_its_cpm_coding/TrafficConditionSubCauseCode.h"
#include "etsi_its_cpm_coding/AccidentSubCauseCode.h"
#include "etsi_its_cpm_coding/RoadworksSubCauseCode.h"
#include "etsi_its_cpm_coding/AdverseWeatherCondition-AdhesionSubCauseCode.h"
#include "etsi_its_cpm_coding/HazardousLocation-SurfaceConditionSubCauseCode.h"
#include "etsi_its_cpm_coding/HazardousLocation-ObstacleOnTheRoadSubCauseCode.h"
#include "etsi_its_cpm_coding/HazardousLocation-AnimalOnTheRoadSubCauseCode.h"
#include "etsi_its_cpm_coding/HumanPresenceOnTheRoadSubCauseCode.h"
#include "etsi_its_cpm_coding/WrongWayDrivingSubCauseCode.h"
#include "etsi_its_cpm_coding/RescueAndRecoveryWorkInProgressSubCauseCode.h"
#include "etsi_its_cpm_coding/AdverseWeatherCondition-ExtremeWeatherConditionSubCauseCode.h"
#include "etsi_its_cpm_coding/AdverseWeatherCondition-VisibilitySubCauseCode.h"
#include "etsi_its_cpm_coding/AdverseWeatherCondition-PrecipitationSubCauseCode.h"
#include "etsi_its_cpm_coding/SlowVehicleSubCauseCode.h"
#include "etsi_its_cpm_coding/DangerousEndOfQueueSubCauseCode.h"
#include "etsi_its_cpm_coding/VehicleBreakdownSubCauseCode.h"
#include "etsi_its_cpm_coding/PostCrashSubCauseCode.h"
#include "etsi_its_cpm_coding/HumanProblemSubCauseCode.h"
#include "etsi_its_cpm_coding/StationaryVehicleSubCauseCode.h"
#include "etsi_its_cpm_coding/EmergencyVehicleApproachingSubCauseCode.h"
#include "etsi_its_cpm_coding/HazardousLocation-DangerousCurveSubCauseCode.h"
#include "etsi_its_cpm_coding/CollisionRiskSubCauseCode.h"
#include "etsi_its_cpm_coding/SignalViolationSubCauseCode.h"
#include "etsi_its_cpm_coding/DangerousSituationSubCauseCode.h"
#include "etsi_its_cpm_coding/RailwayLevelCrossingSubCauseCode.h"
#include <etsi_its_cpm_coding/constr_CHOICE.h>
#ifndef	_CauseCodeChoice_H_
#define	_CauseCodeChoice_H_


#include <etsi_its_cpm_coding/asn_application.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CauseCodeChoice_PR {
	CauseCodeChoice_PR_NOTHING,	/* No components present */
	CauseCodeChoice_PR_reserved0,
	CauseCodeChoice_PR_trafficCondition1,
	CauseCodeChoice_PR_accident2,
	CauseCodeChoice_PR_roadworks3,
	CauseCodeChoice_PR_reserved4,
	CauseCodeChoice_PR_impassability5,
	CauseCodeChoice_PR_adverseWeatherCondition_Adhesion6,
	CauseCodeChoice_PR_aquaplaning7,
	CauseCodeChoice_PR_reserved8,
	CauseCodeChoice_PR_hazardousLocation_SurfaceCondition9,
	CauseCodeChoice_PR_hazardousLocation_ObstacleOnTheRoad10,
	CauseCodeChoice_PR_hazardousLocation_AnimalOnTheRoad11,
	CauseCodeChoice_PR_humanPresenceOnTheRoad12,
	CauseCodeChoice_PR_reserved13,
	CauseCodeChoice_PR_wrongWayDriving14,
	CauseCodeChoice_PR_rescueAndRecoveryWorkInProgress15,
	CauseCodeChoice_PR_reserved16,
	CauseCodeChoice_PR_adverseWeatherCondition_ExtremeWeatherCondition17,
	CauseCodeChoice_PR_adverseWeatherCondition_Visibility18,
	CauseCodeChoice_PR_adverseWeatherCondition_Precipitation19,
	CauseCodeChoice_PR_violence20,
	CauseCodeChoice_PR_reserved21,
	CauseCodeChoice_PR_reserved22,
	CauseCodeChoice_PR_reserved23,
	CauseCodeChoice_PR_reserved24,
	CauseCodeChoice_PR_reserved25,
	CauseCodeChoice_PR_slowVehicle26,
	CauseCodeChoice_PR_dangerousEndOfQueue27,
	CauseCodeChoice_PR_reserved28,
	CauseCodeChoice_PR_reserved29,
	CauseCodeChoice_PR_reserved30,
	CauseCodeChoice_PR_reserved31,
	CauseCodeChoice_PR_reserved32,
	CauseCodeChoice_PR_reserved33,
	CauseCodeChoice_PR_reserved34,
	CauseCodeChoice_PR_reserved35,
	CauseCodeChoice_PR_reserved36,
	CauseCodeChoice_PR_reserved37,
	CauseCodeChoice_PR_reserved38,
	CauseCodeChoice_PR_reserved39,
	CauseCodeChoice_PR_reserved40,
	CauseCodeChoice_PR_reserved41,
	CauseCodeChoice_PR_reserved42,
	CauseCodeChoice_PR_reserved43,
	CauseCodeChoice_PR_reserved44,
	CauseCodeChoice_PR_reserved45,
	CauseCodeChoice_PR_reserved46,
	CauseCodeChoice_PR_reserved47,
	CauseCodeChoice_PR_reserved48,
	CauseCodeChoice_PR_reserved49,
	CauseCodeChoice_PR_reserved50,
	CauseCodeChoice_PR_reserved51,
	CauseCodeChoice_PR_reserved52,
	CauseCodeChoice_PR_reserved53,
	CauseCodeChoice_PR_reserved54,
	CauseCodeChoice_PR_reserved55,
	CauseCodeChoice_PR_reserved56,
	CauseCodeChoice_PR_reserved57,
	CauseCodeChoice_PR_reserved58,
	CauseCodeChoice_PR_reserved59,
	CauseCodeChoice_PR_reserved60,
	CauseCodeChoice_PR_reserved61,
	CauseCodeChoice_PR_reserved62,
	CauseCodeChoice_PR_reserved63,
	CauseCodeChoice_PR_reserved64,
	CauseCodeChoice_PR_reserved65,
	CauseCodeChoice_PR_reserved66,
	CauseCodeChoice_PR_reserved67,
	CauseCodeChoice_PR_reserved68,
	CauseCodeChoice_PR_reserved69,
	CauseCodeChoice_PR_reserved70,
	CauseCodeChoice_PR_reserved71,
	CauseCodeChoice_PR_reserved72,
	CauseCodeChoice_PR_reserved73,
	CauseCodeChoice_PR_reserved74,
	CauseCodeChoice_PR_reserved75,
	CauseCodeChoice_PR_reserved76,
	CauseCodeChoice_PR_reserved77,
	CauseCodeChoice_PR_reserved78,
	CauseCodeChoice_PR_reserved79,
	CauseCodeChoice_PR_reserved80,
	CauseCodeChoice_PR_reserved81,
	CauseCodeChoice_PR_reserved82,
	CauseCodeChoice_PR_reserved83,
	CauseCodeChoice_PR_reserved84,
	CauseCodeChoice_PR_reserved85,
	CauseCodeChoice_PR_reserved86,
	CauseCodeChoice_PR_reserved87,
	CauseCodeChoice_PR_reserved88,
	CauseCodeChoice_PR_reserved89,
	CauseCodeChoice_PR_reserved90,
	CauseCodeChoice_PR_vehicleBreakdown91,
	CauseCodeChoice_PR_postCrash92,
	CauseCodeChoice_PR_humanProblem93,
	CauseCodeChoice_PR_stationaryVehicle94,
	CauseCodeChoice_PR_emergencyVehicleApproaching95,
	CauseCodeChoice_PR_hazardousLocation_DangerousCurve96,
	CauseCodeChoice_PR_collisionRisk97,
	CauseCodeChoice_PR_signalViolation98,
	CauseCodeChoice_PR_dangerousSituation99,
	CauseCodeChoice_PR_railwayLevelCrossing100,
	CauseCodeChoice_PR_reserved101,
	CauseCodeChoice_PR_reserved102,
	CauseCodeChoice_PR_reserved103,
	CauseCodeChoice_PR_reserved104,
	CauseCodeChoice_PR_reserved105,
	CauseCodeChoice_PR_reserved106,
	CauseCodeChoice_PR_reserved107,
	CauseCodeChoice_PR_reserved108,
	CauseCodeChoice_PR_reserved109,
	CauseCodeChoice_PR_reserved110,
	CauseCodeChoice_PR_reserved111,
	CauseCodeChoice_PR_reserved112,
	CauseCodeChoice_PR_reserved113,
	CauseCodeChoice_PR_reserved114,
	CauseCodeChoice_PR_reserved115,
	CauseCodeChoice_PR_reserved116,
	CauseCodeChoice_PR_reserved117,
	CauseCodeChoice_PR_reserved118,
	CauseCodeChoice_PR_reserved119,
	CauseCodeChoice_PR_reserved120,
	CauseCodeChoice_PR_reserved121,
	CauseCodeChoice_PR_reserved122,
	CauseCodeChoice_PR_reserved123,
	CauseCodeChoice_PR_reserved124,
	CauseCodeChoice_PR_reserved125,
	CauseCodeChoice_PR_reserved126,
	CauseCodeChoice_PR_reserved127,
	CauseCodeChoice_PR_reserved128
} CauseCodeChoice_PR;

/* CauseCodeChoice */
typedef struct CauseCodeChoice {
	CauseCodeChoice_PR present;
	union CauseCodeChoice_u {
		SubCauseCodeType_t	 reserved0;
		TrafficConditionSubCauseCode_t	 trafficCondition1;
		AccidentSubCauseCode_t	 accident2;
		RoadworksSubCauseCode_t	 roadworks3;
		SubCauseCodeType_t	 reserved4;
		SubCauseCodeType_t	 impassability5;
		AdverseWeatherCondition_AdhesionSubCauseCode_t	 adverseWeatherCondition_Adhesion6;
		SubCauseCodeType_t	 aquaplaning7;
		SubCauseCodeType_t	 reserved8;
		HazardousLocation_SurfaceConditionSubCauseCode_t	 hazardousLocation_SurfaceCondition9;
		HazardousLocation_ObstacleOnTheRoadSubCauseCode_t	 hazardousLocation_ObstacleOnTheRoad10;
		HazardousLocation_AnimalOnTheRoadSubCauseCode_t	 hazardousLocation_AnimalOnTheRoad11;
		HumanPresenceOnTheRoadSubCauseCode_t	 humanPresenceOnTheRoad12;
		SubCauseCodeType_t	 reserved13;
		WrongWayDrivingSubCauseCode_t	 wrongWayDriving14;
		RescueAndRecoveryWorkInProgressSubCauseCode_t	 rescueAndRecoveryWorkInProgress15;
		SubCauseCodeType_t	 reserved16;
		AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_t	 adverseWeatherCondition_ExtremeWeatherCondition17;
		AdverseWeatherCondition_VisibilitySubCauseCode_t	 adverseWeatherCondition_Visibility18;
		AdverseWeatherCondition_PrecipitationSubCauseCode_t	 adverseWeatherCondition_Precipitation19;
		SubCauseCodeType_t	 violence20;
		SubCauseCodeType_t	 reserved21;
		SubCauseCodeType_t	 reserved22;
		SubCauseCodeType_t	 reserved23;
		SubCauseCodeType_t	 reserved24;
		SubCauseCodeType_t	 reserved25;
		SlowVehicleSubCauseCode_t	 slowVehicle26;
		DangerousEndOfQueueSubCauseCode_t	 dangerousEndOfQueue27;
		SubCauseCodeType_t	 reserved28;
		SubCauseCodeType_t	 reserved29;
		SubCauseCodeType_t	 reserved30;
		SubCauseCodeType_t	 reserved31;
		SubCauseCodeType_t	 reserved32;
		SubCauseCodeType_t	 reserved33;
		SubCauseCodeType_t	 reserved34;
		SubCauseCodeType_t	 reserved35;
		SubCauseCodeType_t	 reserved36;
		SubCauseCodeType_t	 reserved37;
		SubCauseCodeType_t	 reserved38;
		SubCauseCodeType_t	 reserved39;
		SubCauseCodeType_t	 reserved40;
		SubCauseCodeType_t	 reserved41;
		SubCauseCodeType_t	 reserved42;
		SubCauseCodeType_t	 reserved43;
		SubCauseCodeType_t	 reserved44;
		SubCauseCodeType_t	 reserved45;
		SubCauseCodeType_t	 reserved46;
		SubCauseCodeType_t	 reserved47;
		SubCauseCodeType_t	 reserved48;
		SubCauseCodeType_t	 reserved49;
		SubCauseCodeType_t	 reserved50;
		SubCauseCodeType_t	 reserved51;
		SubCauseCodeType_t	 reserved52;
		SubCauseCodeType_t	 reserved53;
		SubCauseCodeType_t	 reserved54;
		SubCauseCodeType_t	 reserved55;
		SubCauseCodeType_t	 reserved56;
		SubCauseCodeType_t	 reserved57;
		SubCauseCodeType_t	 reserved58;
		SubCauseCodeType_t	 reserved59;
		SubCauseCodeType_t	 reserved60;
		SubCauseCodeType_t	 reserved61;
		SubCauseCodeType_t	 reserved62;
		SubCauseCodeType_t	 reserved63;
		SubCauseCodeType_t	 reserved64;
		SubCauseCodeType_t	 reserved65;
		SubCauseCodeType_t	 reserved66;
		SubCauseCodeType_t	 reserved67;
		SubCauseCodeType_t	 reserved68;
		SubCauseCodeType_t	 reserved69;
		SubCauseCodeType_t	 reserved70;
		SubCauseCodeType_t	 reserved71;
		SubCauseCodeType_t	 reserved72;
		SubCauseCodeType_t	 reserved73;
		SubCauseCodeType_t	 reserved74;
		SubCauseCodeType_t	 reserved75;
		SubCauseCodeType_t	 reserved76;
		SubCauseCodeType_t	 reserved77;
		SubCauseCodeType_t	 reserved78;
		SubCauseCodeType_t	 reserved79;
		SubCauseCodeType_t	 reserved80;
		SubCauseCodeType_t	 reserved81;
		SubCauseCodeType_t	 reserved82;
		SubCauseCodeType_t	 reserved83;
		SubCauseCodeType_t	 reserved84;
		SubCauseCodeType_t	 reserved85;
		SubCauseCodeType_t	 reserved86;
		SubCauseCodeType_t	 reserved87;
		SubCauseCodeType_t	 reserved88;
		SubCauseCodeType_t	 reserved89;
		SubCauseCodeType_t	 reserved90;
		VehicleBreakdownSubCauseCode_t	 vehicleBreakdown91;
		PostCrashSubCauseCode_t	 postCrash92;
		HumanProblemSubCauseCode_t	 humanProblem93;
		StationaryVehicleSubCauseCode_t	 stationaryVehicle94;
		EmergencyVehicleApproachingSubCauseCode_t	 emergencyVehicleApproaching95;
		HazardousLocation_DangerousCurveSubCauseCode_t	 hazardousLocation_DangerousCurve96;
		CollisionRiskSubCauseCode_t	 collisionRisk97;
		SignalViolationSubCauseCode_t	 signalViolation98;
		DangerousSituationSubCauseCode_t	 dangerousSituation99;
		RailwayLevelCrossingSubCauseCode_t	 railwayLevelCrossing100;
		SubCauseCodeType_t	 reserved101;
		SubCauseCodeType_t	 reserved102;
		SubCauseCodeType_t	 reserved103;
		SubCauseCodeType_t	 reserved104;
		SubCauseCodeType_t	 reserved105;
		SubCauseCodeType_t	 reserved106;
		SubCauseCodeType_t	 reserved107;
		SubCauseCodeType_t	 reserved108;
		SubCauseCodeType_t	 reserved109;
		SubCauseCodeType_t	 reserved110;
		SubCauseCodeType_t	 reserved111;
		SubCauseCodeType_t	 reserved112;
		SubCauseCodeType_t	 reserved113;
		SubCauseCodeType_t	 reserved114;
		SubCauseCodeType_t	 reserved115;
		SubCauseCodeType_t	 reserved116;
		SubCauseCodeType_t	 reserved117;
		SubCauseCodeType_t	 reserved118;
		SubCauseCodeType_t	 reserved119;
		SubCauseCodeType_t	 reserved120;
		SubCauseCodeType_t	 reserved121;
		SubCauseCodeType_t	 reserved122;
		SubCauseCodeType_t	 reserved123;
		SubCauseCodeType_t	 reserved124;
		SubCauseCodeType_t	 reserved125;
		SubCauseCodeType_t	 reserved126;
		SubCauseCodeType_t	 reserved127;
		SubCauseCodeType_t	 reserved128;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CauseCodeChoice_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CauseCodeChoice;
extern asn_CHOICE_specifics_t asn_SPC_CauseCodeChoice_specs_1;
extern asn_TYPE_member_t asn_MBR_CauseCodeChoice_1[129];
extern asn_per_constraints_t asn_PER_type_CauseCodeChoice_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _CauseCodeChoice_H_ */
#include <etsi_its_cpm_coding/asn_internal.h>
