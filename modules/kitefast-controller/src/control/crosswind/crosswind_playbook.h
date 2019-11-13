#ifndef CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_H_
#define CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_H_

#include "control/crosswind/crosswind_playbook_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void GetPlaybookEntry(const Playbook *playbook, const PlaybookEntry *fallback,
                      double wind_speed, double fallback_crossfade,
                      PlaybookEntry *pb_entry_interp);

double GetPlaybookEntryAzimuthWithLimits(double wind_dir,
                                         const PlaybookEntry *playbook_entry);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_H_
