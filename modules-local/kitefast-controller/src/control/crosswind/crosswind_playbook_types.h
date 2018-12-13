#ifndef CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_TYPES_H_
#define CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_TYPES_H_

#define CROSSWIND_SCHEDULE_TABLE_LENGTH 50
typedef struct {
  double wind_speed;
  double alpha_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double beta_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double airspeed_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double path_radius_target;
  double elevation;
  double azi_offset;
  double lookup_loop_angle[CROSSWIND_SCHEDULE_TABLE_LENGTH];
} PlaybookEntry;

#define NUM_PLAYBOOK_ENTRIES 15
typedef struct {
  int num_entries;
  PlaybookEntry entries[NUM_PLAYBOOK_ENTRIES];
} Playbook;

typedef struct {
  PlaybookEntry entry;
  double crossfade_rate;
  double crossfade_throttle;
} PlaybookFallbackParams;

#endif  // CONTROL_CROSSWIND_CROSSWIND_PLAYBOOK_TYPES_H_
