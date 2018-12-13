#include "control/system_params.h" 
#include "control/system_types.h"

static SystemParams system_params = { .ts = 0.01 }; //ts added by JMiller - STI
const GlobalSystemParams g_sys = {
          .ts = &system_params.ts,
          .phys = &system_params.phys,
          .tether = &system_params.tether,
          .wing = &system_params.wing,
          .perch = &system_params.perch,
          .rotors = &system_params.rotors };

const SystemParams *GetSystemParams(void) { return &system_params; }
SystemParams *GetSystemParamsUnsafe(void) { return &system_params; }
