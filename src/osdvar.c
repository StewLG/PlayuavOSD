/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
/*
 * With Grateful Acknowledgements to the projects:
 * MinimOSD - arducam-osd Controller(https://code.google.com/p/arducam-osd/)
 */
#include "osdvar.h"

/////////////////////////////////////////////////////////////////////////

// FINAL BATCH TO MOVE?
// ---------------------

uint8_t mavbeat = 0;
uint32_t lastMAVBeat = 0;
uint32_t lastWritePanel = 0;
uint8_t waitingMAVBeats = 1;
uint8_t mav_type;
uint8_t mav_system;
uint8_t mav_component;
uint8_t enable_mav_request = 0;
uint32_t sys_start_time = 0;
uint32_t heatbeat_start_time = 0;
uint32_t armed_start_time = 0;
uint32_t total_armed_time = 0;


// This is the OSD state that is unowned by any particular thread, and flows from
// Mavlink/UAVTalk to the OSD thread. This is called the "airlock".
osd_state airlock_osd_state;

// This is other global OSD state that doesn't flow from a serial protocol to OSD,
// but follows other patterns.
other_osd_state adhoc_osd_state;


// Globals with no home yet 
// These don't have obvious mavlink/osd concurrency issues, but need
// evaluation just the same -- they might be shared across threads.
// -------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////
// Mutexes to protect safe access to variables shared 
// between concurrent tasks.
/////////////////////////////////////////////////////////////////////////

// This mutex controls access to the airlock OSD State
xSemaphoreHandle osd_state_airlock_mutex;

// This mutex controls access to the ad-hoc OSD State
xSemaphoreHandle osd_state_adhoc_mutex;

// Initialize the various mutexes designed to protect variables shared between tasks.
void variable_mutexes_init() {    
    osd_state_airlock_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(osd_state_airlock_mutex);    
    
    osd_state_adhoc_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(osd_state_adhoc_mutex);        
}

/////////////////////////////////////////////////////////////
/// Convenience accessors [TODO: Move them all here??]
/////////////////////////////////////////////////////////////

float get_atti_3d_scale() {
  float atti_3d_scale = 0.0f;
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      atti_3d_scale = adhoc_osd_state.atti_3d_scale;
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
   }
   return atti_3d_scale;
}

float get_atti_mp_scale() {
  float atti_mp_scale = 0.0f;
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      atti_mp_scale = adhoc_osd_state.atti_mp_scale;
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
   }
   return atti_mp_scale;
}

float get_current_consumed_mah() {
  float current_consumed_mah = 0.0f;  
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      current_consumed_mah = adhoc_osd_state.osd_curr_consumed_mah;
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
   }
   return current_consumed_mah;
}

/////////////////////////////////////////////////////////////////////////
// Threadsafe Airlock Concept
/////////////////////////////////////////////////////////////////////////

// Copy an osd_state object in a thread-safe manner
// May not succeed if the lock is not gained in tick_delay ticks
void copy_osd_state_thread_safe(osd_state * p_osd_state_source, 
                                osd_state * p_osd_state_target,
                                TickType_t tick_delay) {    
    if (xSemaphoreTake(osd_state_airlock_mutex, tick_delay) == pdTRUE ) {
        // Copy current values
        *p_osd_state_target = *p_osd_state_source;
        // Release the airlock mutex
        xSemaphoreGive(osd_state_airlock_mutex);        
    }    
    else
    {
        // Did not succeed; values won't be copied.
    }
}
