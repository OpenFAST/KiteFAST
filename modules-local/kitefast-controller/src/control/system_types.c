#include "system_types.h"

#include <assert.h>
#include <stdbool.h>

const char *FlightPlanToString(FlightPlan flight_plan) {
  switch (flight_plan) {
    case kFlightPlanDisengageEngage:
      return "DisengageEngage";
    case kFlightPlanHoverAndAccel:
      return "HoverAndAccel";
    case kFlightPlanHoverInPlace:
      return "HoverInPlace";
    case kFlightPlanLaunchPerch:
      return "LaunchPerch";
    case kFlightPlanManual:
      return "Manual";
    case kFlightPlanRemotePerchHover:
      return "RemotePerchHover";
    case kFlightPlanRemotePerchCrosswind:
      return "RemotePerchCrosswind";
    case kFlightPlanStartDownwind:
      return "StartDownwind";
    case kFlightPlanTurnKey:
      return "TurnKey";
    default:
    case kFlightPlanForceSigned:
    case kNumFlightPlans:
      assert(false);
      return "<Unknown>";
  }
}

const char *GroundStationModelToString(GroundStationModel gs) {
  switch (gs) {
    case kGroundStationModelGSv1:
      return "GSv1";
    case kGroundStationModelGSv2:
      return "GSv2";
    case kGroundStationModelTopHat:
      return "TopHat";
    default:
    case kGroundStationModelForceSigned:
    case kNumGroundStationModels:
      assert(false);
      return "<Unknown>";
  }
}

const char *TestSiteToString(TestSite test_site) {
  switch (test_site) {
    case kTestSiteAlameda:
      return "Alameda";
    case kTestSiteChinaLake:
      return "ChinaLake";
    case kTestSiteParkerRanch:
      return "ParkerRanch";
    case kTestSiteNorway:
      return "Norway";
    default:
    case kTestSiteForceSigned:
    case kNumTestSites:
      assert(false);
      return "<Unknown>";
  }
}

const char *WingSerialToString(WingSerial wing_serial) {
  switch (wing_serial) {
    case kWingSerial01:
      return "01";
    case kWingSerial02:
      return "02";
    case kWingSerial02Final:
      return "02Final";
    case kWingSerial03Hover:
      return "03Hover";
    case kWingSerial03Crosswind:
      return "03Crosswind";
    case kWingSerial04Hover:
      return "04Hover";
    case kWingSerial04Crosswind:
      return "04Crosswind";
    default:
    case kWingSerialForceSigned:
    case kNumWingSerials:
      assert(false);
      return "<Unknown>";
  }
}

LoadcellSensorLabel BridleAndChannelToLoadcellSensorLabel(BridleLabel bridle,
                                                          int32_t channel) {
  switch (bridle) {
    case kBridlePort:
      switch (channel) {
        case 0:
          return kLoadcellSensorPort0;
        case 1:
          return kLoadcellSensorPort1;
        default:
          assert(false);
          return kLoadcellSensorPort0;
      }
    case kBridleStar:
      switch (channel) {
        case 0:
          return kLoadcellSensorStarboard0;
        case 1:
          return kLoadcellSensorStarboard1;
        default:
          assert(false);
          return kLoadcellSensorPort0;
      }
    case kBridleLabelForceSigned:
    case kNumBridles:
    default:
      assert(false);
      return kLoadcellSensorPort0;
  }
}

bool IsLowAltitudeFlightPlan(FlightPlan flight_plan) {
  switch (flight_plan) {
    case kFlightPlanDisengageEngage:
    case kFlightPlanHoverInPlace:
    case kFlightPlanLaunchPerch:
    case kFlightPlanManual:
    case kFlightPlanRemotePerchHover:
      return true;
    case kFlightPlanHoverAndAccel:
    case kFlightPlanTurnKey:
    case kFlightPlanRemotePerchCrosswind:
    case kFlightPlanStartDownwind:
      return false;
    case kFlightPlanForceSigned:
    case kNumFlightPlans:
    default:
      assert(false);
      return true;
  }
}
