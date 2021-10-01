//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init(400);
}

void Copter::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;
#if false
    // use range finder altitude if it is valid, else try to get terrain alt
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.alt_cm;
    } else if (terrain_use()) {
        if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, height_above_ground_cm)) {
            height_above_ground_cm = current_loc.alt;
        }
    }
#else
    // use range finder altitude if it is valid, otherwise use home alt
    if (rangefinder_alt_ok()) {
        get_rangefinder_height_interpolated_cm(height_above_ground_cm, true);
    }
#endif    

    precland.update(height_above_ground_cm, rangefinder_alt_ok());

    g2.precland_sm->set_alt_above_ground_cm(height_above_ground_cm);
    if (battery.has_failsafed()) {
        g2.precland_sm->set_emergency_flag(true);
    }
    g2.precland_sm->run();
}
#endif
