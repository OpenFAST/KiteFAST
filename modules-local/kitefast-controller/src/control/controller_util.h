#ifndef CONTORLLER_UTIL_H
#define CONTROLLER_UTIL_H

#include "control/crosswind/crosswind_types.h"
#include "control/control_types.h"
//#include "system_params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    double airspeed_tmp[CROSSWIND_SCHEDULE_TABLE_LENGTH];
	double alpha_tmp[CROSSWIND_SCHEDULE_TABLE_LENGTH];
    double beta_tmp[CROSSWIND_SCHEDULE_TABLE_LENGTH];
    double azi_offset_tmp;
    double elevation_tmp;
    double radius_tmp;
    double wind_speed_tmp;
 }Temp_playbook;

void min_max_lateral_integrators(CrosswindParams *crosswind); 
void loadPlaybook(CrosswindParams *crosswind);
void loadcontroller(ControlParams *control_params);
void loadcrosswind(CrosswindParams *crosswind);
void loadFallback(CrosswindParams *crosswind);

#ifdef __cplusplus
}  // extern "C"
#endif
#endif

