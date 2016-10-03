#ifndef __OSDVAR_H
#define __OSDVAR_H

#include "board.h"

/////////////////////////////////////////////////////////////////////////
extern uint8_t mavbeat;
extern uint32_t lastMAVBeat;
extern uint32_t lastWritePanel;
extern uint8_t waitingMAVBeats;
extern uint8_t mav_type;
extern uint8_t mav_system;
extern uint8_t mav_component;
extern uint8_t enable_mav_request;
extern uint32_t sys_start_time;
extern uint32_t heatbeat_start_time;
extern uint32_t armed_start_time;
extern uint32_t total_armed_time;


/////////////////////////////////////////////////////////////////////////

void variable_mutexes_init(void);

/////////////////////////////////////////////////////////////////////////
// Accessor convenience functions for accessing protected variables 
// between tasks. Hides details of mutexes and helps prevent accidental 
// misuse.
/////////////////////////////////////////////////////////////////////////

// void get_osd_lat_long(float * p_osd_lat, float * p_osd_long);
// void set_osd_lat_long(float osd_lat, float osd_long);

// void get_osd_alt(float * p_osd_alt);
// void set_osd_alt(float osd_alt);


/////////////////////////////////////////////////////////////////////////
// Protected set of variables
/////////////////////////////////////////////////////////////////////////

typedef struct osd_state_struct osd_state;
struct osd_state_struct {
    float osd_alt;                           // Current altitude
    
    float osd_lat;                           // GPS Latitude
    float osd_lon;                           // GPS Longitude
        
    float osd_vbat_A;                        // Battery A voltage in milivolt
    int16_t osd_curr_A;                      // Battery A current
    int8_t osd_battery_remaining_A;          // 0 to 100 <=> 0 to 1000

    float osd_pitch;                         // pitch from DCM
    float osd_roll;                          // roll from DCM
    // No usage in OSDProc yet, but it starts in Mavlink so is safe to put here.
    float osd_yaw;                           // relative heading form DCM
    float osd_heading;                       // ground course heading from GPS

    uint8_t osd_satellites_visible;          // number of satelites
    uint8_t osd_fix_type;                    // GPS lock 0-1=no fix, 2=2D, 3=3D
    double osd_hdop;                         // GPS HDOP
    
    // BATCH 2
    // --------------------------------
        
    float osd_lat2;                              // latitude GPS #2
    float osd_lon2;                              // longitude GPS #2
    uint8_t osd_satellites_visible2;             // number of satelites GPS #2
    uint8_t osd_fix_type2;                       // GPS lock 0-1=no fix, 2=2D, 3=3D GPS #2
    double osd_hdop2;

    float osd_airspeed;                          // airspeed -- NOTE, set to -1.0f by default. Was this important?
    float osd_groundspeed;                       // ground speed

    uint16_t osd_throttle;                       // throttle

    float osd_rel_alt;                           // relative altitude -- jmmods
    float osd_climb;

    float nav_roll;                              // Current desired roll in degrees
    float nav_pitch;                             // Current desired pitch in degrees
    int16_t nav_bearing;                         // Current desired heading in degrees
    
    uint16_t wp_dist;                            // Distance to active MISSION in meters
    uint8_t wp_number;                           // Current waypoint number
    float alt_error;                             // Current altitude error in meters
    float aspd_error;                            // Current airspeed error in meters/second
    float xtrack_error;                          // Current crosstrack error on x-y plane in meters

    bool motor_armed;
    bool last_motor_armed;
    uint8_t autopilot;
    uint8_t base_mode;
    uint32_t custom_mode;
    
    int16_t wp_target_bearing;                  // Bearing to current MISSION/target in degrees

    bool osd_chan_cnt_above_eight;
    uint16_t osd_chan1_raw;
    uint16_t osd_chan2_raw;
    uint16_t osd_chan3_raw;
    uint16_t osd_chan4_raw;
    uint16_t osd_chan5_raw;
    uint16_t osd_chan6_raw;
    uint16_t osd_chan7_raw;
    uint16_t osd_chan8_raw;
    uint16_t osd_chan9_raw;
    uint16_t osd_chan10_raw;
    uint16_t osd_chan11_raw;
    uint16_t osd_chan12_raw;
    uint16_t osd_chan13_raw;
    uint16_t osd_chan14_raw;
    uint16_t osd_chan15_raw;
    uint16_t osd_chan16_raw;
    uint8_t osd_rssi;           //raw value from mavlink    
    
};




// TODO-- A Clear function to get 0's initialized as per the global initialziers
// accomplished. 

// Globals with no home yet 
// These don't have obvious mavlink/osd concurrency issues, but need
// evaluation just the same -- they might be shared across threads.
// -------------------------------------------------------------------




    //float osd_curr_consumed_mah = 0;

// 2nd batch
// -------------

/*
  float osd_downVelocity = 0.0f;
    float osd_climb_ma[10];
    int osd_climb_ma_index = 0;




    // Declared but apparently unused??? Omitting. -- SLG
    int8_t wp_target_bearing_rotate_int = 0;   

    // Unsure where this goes???
    float eff = 0.0f; //Efficiency    
    
    // Unused?
    uint8_t osd_linkquality = 0;
    
*/    

// And here's at least one place we'll put those other globals

/* 
These are global variables that do NOT flow from the serial
protocols (Mavlink/Uavtalk/etc) to the OSD, but have other
global-type lifetimes.

Not every one of these globals needs mutex protection, but
given the errors of the past it seems wise to err on the
conservative side, assuming performance problems can
be avoided. 

-- SLG
*/
typedef struct other_osd_state_struct other_osd_state;
struct other_osd_state_struct {  
    float osd_total_trip_dist; 
    float osd_climb_ma[10];
    int osd_climb_ma_index;    
};
    
    
    
// -------------------------------------------------------------------


// Airlock OSD state. Access controlled with mutex, take care!
// Values in the airlock start in Mavlink/Uavtalk/other serial protocol,
// then move to the OSD thread.
extern osd_state airlock_osd_state;

// Other OSD state. This is other globals that need mutex control,
// but may not be flowing directly out of mavlink, but rather are
// manipulated elswhere. This is more ad-hoc stuff, with less of 
// a pattern to the flow.
extern other_osd_state adhoc_osd_state;

// Copy an osd_state object in a thread-safe manner
void copy_osd_state_thread_safe(osd_state * p_osd_state_source, 
                                osd_state * p_osd_state_target,
                                TickType_t tick_delay);


/////////////////////////////////////////////////////////////////////////

/*
extern float osd_lat2;                      // latitude
extern float osd_lon2;                      // longitude
extern uint8_t osd_satellites_visible2;     // number of satelites
extern uint8_t osd_fix_type2;               // GPS lock 0-1=no fix, 2=2D, 3=3D
extern double osd_hdop2;

extern float osd_airspeed;               // airspeed
extern float osd_groundspeed;            // ground speed
extern float osd_downVelocity;           // ground speed
extern uint16_t osd_throttle;            // throttle

extern float osd_rel_alt;                // relative altitude   //  jmmods
extern float osd_climb;
extern float osd_climb_ma[10];
extern int osd_climb_ma_index;
extern float osd_total_trip_dist; //total trip distance since startup, calculated in meter

extern float nav_roll; // Current desired roll in degrees
extern float nav_pitch; // Current desired pitch in degrees
extern int16_t nav_bearing; // Current desired heading in degrees
extern int16_t wp_target_bearing; // Bearing to current MISSION/target in degrees
extern int8_t wp_target_bearing_rotate_int;
extern uint16_t wp_dist; // Distance to active MISSION in meters
extern uint8_t wp_number; // Current waypoint number
extern float alt_error; // Current altitude error in meters
extern float aspd_error; // Current airspeed error in meters/second
extern float xtrack_error; // Current crosstrack error on x-y plane in meters
extern float eff; //Efficiency

extern uint32_t custom_mode;
extern bool motor_armed;
extern bool last_motor_armed;
extern uint8_t base_mode;
extern uint8_t autopilot;

extern bool osd_chan_cnt_above_eight;
extern uint16_t osd_chan1_raw;
extern uint16_t osd_chan2_raw;
extern uint16_t osd_chan3_raw;
extern uint16_t osd_chan4_raw;
extern uint16_t osd_chan5_raw;
extern uint16_t osd_chan6_raw;
extern uint16_t osd_chan7_raw;
extern uint16_t osd_chan8_raw;
extern uint16_t osd_chan9_raw;
extern uint16_t osd_chan10_raw;
extern uint16_t osd_chan11_raw;
extern uint16_t osd_chan12_raw;
extern uint16_t osd_chan13_raw;
extern uint16_t osd_chan14_raw;
extern uint16_t osd_chan15_raw;
extern uint16_t osd_chan16_raw;
extern uint8_t osd_rssi; //raw value from mavlink
*/



extern uint8_t osd_got_home;               // tels if got home position or not
extern float osd_home_lat;               // home latidude
extern float osd_home_lon;               // home longitude
extern float osd_home_alt;
extern long osd_home_distance;          // distance from home
extern uint32_t osd_home_bearing;
extern uint8_t osd_alt_cnt;              // counter for stable osd_alt
extern float osd_alt_prev;             // previous altitude

extern float osd_windSpeed;
extern float osd_windDir;

extern volatile uint8_t current_panel;

extern float atti_mp_scale;
extern float atti_3d_scale;
extern uint32_t atti_3d_min_clipX;
extern uint32_t atti_3d_max_clipX;
extern uint32_t atti_3d_min_clipY;
extern uint32_t atti_3d_max_clipY;

#define MAX_WAYPOINTS   20

extern uint8_t got_mission_counts;
extern uint8_t enable_mission_count_request;
extern uint16_t mission_counts;
extern uint8_t enable_mission_item_request;
extern uint16_t current_mission_item_req_index;

extern uint16_t wp_counts;
extern uint8_t got_all_wps;


// Globals we are still in the process of finding homes for and migrating
// ----------------------------------------------------------------------

extern float osd_curr_consumed_mah; // total current drawn since startup in amp-hours


// ----------------------------------------------------------------------

// a self contained waypoint list
typedef struct WAYPOINT_TYP {
//	float para1;
//    float para2;
//    float para3;
//    float para4;

  float x;
  float y;
  float z;

  uint16_t seq;
  uint16_t cmd;

//    uint8_t frame;
  uint8_t current;
//    uint8_t autocontinue;
} WAYPOINT, *WAYPOINT_PTR;

extern WAYPOINT wp_list[MAX_WAYPOINTS];

extern int8_t osd_offset_Y;
extern int8_t osd_offset_X;
#endif
