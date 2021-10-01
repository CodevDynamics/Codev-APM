#pragma once

#include <AP_Logger/LogStructure.h>

// @LoggerMessage: PLSM
// @Description: Precision Landing messages
// @Field: TimeUS: Time since system startup
// @Field: State: State Machine State
// @Field: Alt: Altitude above ground
// @Field: Climb: Climb rate
// @Field: PosErr: Average position error
// @Field: Retry: Retry number
struct PACKED log_Precland_SM {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        uint8_t state;
        int32_t alt_above_ground;
        int32_t last_alt_above_ground;
        float climb_rate;
        float avg_pos_error;
        uint8_t retry;
};

#define LOG_STRUCTURE_FROM_PLSM \
    { LOG_PRECLAND_SM_MSG, sizeof(log_Precland_SM), \
      "PLSM",    "QBiiffB",    "TimeUS,State,Alt,LAlt,Climb,PosErr,Retry", "s-mmnm-","F-BBBB-" },
