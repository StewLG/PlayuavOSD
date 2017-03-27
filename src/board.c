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

#include "board.h"

#include "led.h"
#include "uavtalk.h"
#include "osdproc.h"
#include "osdcore.h"
#include "osdmavlink.h"
#include "max7456.h"
#include "usart2.h"
#include "osdconfig.h"
#include "math3d.h"
#include "osdvar.h"
#include <math.h>


void vTaskHeartBeat(void *pvParameters);
void vTask10HZ(void *pvParameters);
void triggerVideo(void);
void triggerPanel(void);
void checkDefaultParam(void);

// Intra-task communication semaphores
extern xSemaphoreHandle onScreenDisplaySemaphore;
extern xSemaphoreHandle onMavlinkSemaphore;
extern xSemaphoreHandle onUAVTalkSemaphore;

// This mutex controls access to the Mavlink OSD State
extern xSemaphoreHandle osd_state_mavlink_mutex;
// This mutex controls access to the airlock OSD State
extern xSemaphoreHandle osd_state_airlock_mutex;
// this mutex controls access to other OSD state
extern xSemaphoreHandle osd_state_adhoc_mutex;

// This is the actual Mavlink OSDState. 
// Only access via osd_state_mavlink_mutex!
extern osd_state mavlink_osd_state;

uint8_t video_switch = 0;

xTaskHandle xTaskVCPHandle;

int32_t pwmPanelNormal = 0;

void set_up_offsets_and_scale() {
  // Take the ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
    //fabs, make sure not broken the VBI
    adhoc_osd_state.osd_offset_Y = fabs(eeprom_buffer.params.osd_offsetY);

    adhoc_osd_state.osd_offset_X = eeprom_buffer.params.osd_offsetX;
    if (eeprom_buffer.params.osd_offsetX_sign == 0) {
        adhoc_osd_state.osd_offset_X = adhoc_osd_state.osd_offset_X * -1;
    }

    adhoc_osd_state.atti_mp_scale = (float)eeprom_buffer.params.Atti_mp_scale_real + (float)eeprom_buffer.params.Atti_mp_scale_frac * 0.01;
    adhoc_osd_state.atti_3d_scale = (float)eeprom_buffer.params.Atti_3D_scale_real + (float)eeprom_buffer.params.Atti_3D_scale_frac * 0.01;
    adhoc_osd_state.atti_3d_min_clipX = eeprom_buffer.params.Atti_mp_posX - (uint32_t)(22 * adhoc_osd_state.atti_mp_scale);
    adhoc_osd_state.atti_3d_max_clipX = eeprom_buffer.params.Atti_mp_posX + (uint32_t)(22 * adhoc_osd_state.atti_mp_scale);
    adhoc_osd_state.atti_3d_min_clipY = eeprom_buffer.params.Atti_mp_posY - (uint32_t)(30 * adhoc_osd_state.atti_mp_scale);
    adhoc_osd_state.atti_3d_max_clipY = eeprom_buffer.params.Atti_mp_posY + (uint32_t)(34 * adhoc_osd_state.atti_mp_scale);      
           
    // Release the ad-hoc mutex
    xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

void board_init(void) {
  GPIO_InitTypeDef gpio;
  SystemCoreClockUpdate();

  // turn on peripherals needed by all
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                         RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |
                         RCC_AHB1Periph_DMA1  | RCC_AHB1Periph_DMA2  |
                         RCC_AHB1Periph_BKPSRAM, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_SYSCFG, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_SPI3 | RCC_APB1Periph_TIM2 |
                         RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 |  RCC_APB1Periph_TIM12 |
                         RCC_APB1Periph_PWR, ENABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  RCC_LSEConfig(RCC_LSE_OFF);

  LEDInit(LED_BLUE);
  LEDInit(LED_GREEN);

  gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode = GPIO_Mode_OUT;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &gpio);
  GPIO_SetBits(GPIOC, GPIO_Pin_0);
  gpio.GPIO_Pin = GPIO_Pin_1;
  gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &gpio);
  GPIO_ResetBits(GPIOC, GPIO_Pin_1);

  //SPI1 output to electronic switch to control mask
  GPIO_StructInit(&gpio);
  gpio.GPIO_Pin = GPIO_Pin_6;       // SPI1 MISO
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);
  GPIO_ResetBits(GPIOA, GPIO_Pin_6);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

  // max7456 SPI MOIS MISO SLK pin config
  gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &gpio);

  // max7456 SPI CS Pin
  gpio.GPIO_Pin = GPIO_Pin_15;
  gpio.GPIO_Mode = GPIO_Mode_OUT;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);
  GPIO_SetBits(GPIOA, GPIO_Pin_15);

  Build_Sin_Cos_Tables();

  bool force_clear_params = false;
  force_clear_params = test_force_clear_all_params();
  if (force_clear_params)
  {
    clear_all_params();
  }

  LoadParams();
  checkDefaultParam();
  SPI_MAX7456_init();
  
  set_up_offsets_and_scale();
}

void module_init(void) {
  xTaskCreate(vTaskHeartBeat, (const char*)"Task Heartbeat",
              STACK_SIZE_MIN, NULL, THREAD_PRIO_LOW, NULL);

  xTaskCreate(vTask10HZ, (const char*)"Task 10HZ",
              STACK_SIZE_MIN, NULL, THREAD_PRIO_NORMAL, NULL);

  xTaskCreate(vTaskOSD, (const char*)"Task OSD",
              STACK_SIZE_MIN * 2, NULL, THREAD_PRIO_HIGHEST, NULL);

  xTaskCreate(vTaskVCP, (const char*)"Task VCP",
              STACK_SIZE_MIN * 2, NULL, THREAD_PRIO_NORMAL, &xTaskVCPHandle);

  switch (eeprom_buffer.params.FC_Protocol) {
  case PROTOCOL_MAVLINK:
    xTaskCreate(MavlinkTask, (const char*)"Task Mavlink",
                STACK_SIZE_MIN * 2, NULL, THREAD_PRIO_HIGH, NULL);
    break;
  case PROTOCOL_UAVTALK:
    xTaskCreate(UAVTalkTask, (const char*)"Task UAVTalk",
                STACK_SIZE_MIN * 2, NULL, THREAD_PRIO_HIGH, NULL);
    break;
  default:
    break;
  }

//	xTaskCreate( DJICanTask, (const char*)"DJI CAN",
//	STACK_SIZE_MIN, NULL, THREAD_PRIO_HIGH, NULL );
}

// Initialize the semaphores used for intra-task communication
void task_semaphores_init() {
  vSemaphoreCreateBinary(onScreenDisplaySemaphore);
  vSemaphoreCreateBinary(onMavlinkSemaphore);
  vSemaphoreCreateBinary(onUAVTalkSemaphore);
}

void vTaskHeartBeat(void *pvParameters) {
    
  for (;; )
  {
    LEDToggle(LED_GREEN);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void update_current_consumed_estimate() {    
    // Update the values directly (in/from) the airlock, which is presumed to be
    // reasonably current
    if (xSemaphoreTake(osd_state_airlock_mutex, portMAX_DELAY) == pdTRUE ) {
         float current_increment = (airlock_osd_state.osd_curr_A * 0.00027777778f);
        // Release the airlock mutex
        xSemaphoreGive(osd_state_airlock_mutex);
                
        // calculate osd_curr_consumed_mah(simulation) 
        if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
            adhoc_osd_state.osd_curr_consumed_mah += current_increment; 
            // Release the ad-hoc mutex
            xSemaphoreGive(osd_state_adhoc_mutex);
        }        
    }    
}     

void update_total_trip_distance() {
    // Update the values directly (in/from) the airlock, which is presumed to be
    // reasonably current
    if (xSemaphoreTake(osd_state_airlock_mutex, portMAX_DELAY) == pdTRUE ) {
        float additional_trip_distance = 0.0f;
        
        // calculate osd_total_trip_dist(simulation)
        if (airlock_osd_state.osd_groundspeed > 1.0f) {
            // jmmods > for calculation of trip , Groundspeed is better than airspeed        
            additional_trip_distance = (airlock_osd_state.osd_groundspeed * 0.1f); 
        }    
        // Release the airlock mutex
        xSemaphoreGive(osd_state_airlock_mutex);     

        // Update total trip distance using the ad-hoc mutex & global structure
        if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
            adhoc_osd_state.osd_total_trip_dist += additional_trip_distance; 
            // Release the ad-hoc mutex
            xSemaphoreGive(osd_state_adhoc_mutex);                          
        }
    }
}  

void update_mission_counts() {
    bool should_request_mission_count = false;
    bool should_request_mission_item = false;
    uint16_t TEMP_current_mission_item_request_index = -1;
        
    // These updates happen as early as Mavlink since the Mavlink task
    // looks at these values to determine when to query / re-query for Mission waypoints.

    // Lock access to Mavlink OSDState via mutex
    if (xSemaphoreTake(osd_state_mavlink_mutex, portMAX_DELAY) == pdTRUE ) {
        if (mavlink_osd_state.enable_mission_count_request == 1)
        {
          should_request_mission_count = true;
          mavlink_osd_state.enable_mission_count_request = 0;
        }

        if (mavlink_osd_state.enable_mission_item_request == 1)
        {
          should_request_mission_item = true;
          TEMP_current_mission_item_request_index = mavlink_osd_state.current_mission_item_req_index;
        }
        // Release the Mavlink mutex ASAP
        xSemaphoreGive(osd_state_mavlink_mutex);

        // These are Mavlink calls, and might be slow, so we take pains to do them
        // outside the possession of the mutex to prevent any potential pend.

        if (should_request_mission_count == true) {
          request_mission_count();
        }
        
        if (should_request_mission_item == true){
            request_mission_item(TEMP_current_mission_item_request_index);
        }
    }
}

void vTask10HZ(void *pvParameters) {
  for (;; )
  {
    vTaskDelay(100 / portTICK_RATE_MS);

    update_current_consumed_estimate();
    update_total_trip_distance();

    //trigger video switch
    if (eeprom_buffer.params.PWM_Video_en)
    {
      triggerVideo();
    }

    //trigger panel switch
    if (eeprom_buffer.params.PWM_Panel_en)
    {
      triggerPanel();
    }

    //if no mavlink update for 2 secs, show warning and request mavlink rate again
    if (GetSystimeMS() > (get_lastMAVBeat() + 2200))
    {
      set_heartbeat_start_time(0);
      set_waitingMAVBeats(1);
    }

    if (get_enable_mav_request() == 1)
    {
      for (int n = 0; n < 3; n++) {
        request_mavlink_rates();            //Three times to certify it will be readed
        vTaskDelay(50 / portTICK_RATE_MS);
      }
      set_enable_mav_request(0);
      set_waitingMAVBeats(0);
      set_lastMAVBeat(GetSystimeMS());
    }

    update_mission_counts();
  }
}

void triggerVideo(void) {
  static uint16_t video_ch_raw;
  static bool video_trigger = false;
  
  video_ch_raw = 0;

  // Take airlock mutex
  if (xSemaphoreTake(osd_state_airlock_mutex, portMAX_DELAY) == pdTRUE ) {
      if (eeprom_buffer.params.PWM_Video_ch == 5) video_ch_raw = airlock_osd_state.osd_chan5_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 6) video_ch_raw = airlock_osd_state.osd_chan6_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 7) video_ch_raw = airlock_osd_state.osd_chan7_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 8) video_ch_raw = airlock_osd_state.osd_chan8_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 9) video_ch_raw = airlock_osd_state.osd_chan9_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 10) video_ch_raw = airlock_osd_state.osd_chan10_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 11) video_ch_raw = airlock_osd_state.osd_chan11_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 12) video_ch_raw = airlock_osd_state.osd_chan12_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 13) video_ch_raw = airlock_osd_state.osd_chan13_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 14) video_ch_raw = airlock_osd_state.osd_chan14_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 15) video_ch_raw = airlock_osd_state.osd_chan15_raw;
      else if (eeprom_buffer.params.PWM_Video_ch == 16) video_ch_raw = airlock_osd_state.osd_chan16_raw;
      // Release the airlock mutex
      xSemaphoreGive(osd_state_airlock_mutex);
  }      
      
  if (eeprom_buffer.params.PWM_Video_mode == 0) {
    if (video_ch_raw > eeprom_buffer.params.PWM_Video_value) {
      if (!video_trigger) {
        video_trigger = true;
        if (video_switch == 0) {
          video_switch =  1;
          GPIO_ResetBits(GPIOC, GPIO_Pin_0);
          GPIO_SetBits(GPIOC, GPIO_Pin_1);
        } else {
          video_switch = 0;
          GPIO_SetBits(GPIOC, GPIO_Pin_0);
          GPIO_ResetBits(GPIOC, GPIO_Pin_1);
        }
      }
    } else {
      video_trigger = false;
    }
  } else {
    if (video_ch_raw < 1500 && video_switch == 1) {
      video_switch = 0;
      GPIO_SetBits(GPIOC, GPIO_Pin_0);
      GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    } else if (video_ch_raw >= 1500 && video_switch == 0) {
      video_switch = 1;
      GPIO_ResetBits(GPIOC, GPIO_Pin_0);
      GPIO_SetBits(GPIOC, GPIO_Pin_1);
    }
  }
}

// Request a reload of the Waypoints from outside the Mavlink thread.
// (Requests outside the Mavlink thread require mutex locking)
void request_reload_of_waypoints_outside_mavlink_thread() {
    // Lock access to Mavlink OSDState via mutex
    if (xSemaphoreTake(osd_state_mavlink_mutex, portMAX_DELAY) == pdTRUE ) {
        // Request a reload of waypoints
        mavlink_osd_state.enable_mission_count_request = 1;
        // Release the Mavlink OSDState mutex
        xSemaphoreGive(osd_state_mavlink_mutex);
    }
}

void triggerPanel(void) {
  static uint16_t panel_ch_raw;
  static bool panel_trigger = false;

  panel_ch_raw = 0;
  
  // Take airlock mutex
  if (xSemaphoreTake(osd_state_airlock_mutex, portMAX_DELAY) == pdTRUE ) {
      if (eeprom_buffer.params.PWM_Panel_ch == 5) panel_ch_raw = airlock_osd_state.osd_chan5_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 6) panel_ch_raw = airlock_osd_state.osd_chan6_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 7) panel_ch_raw = airlock_osd_state.osd_chan7_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 8) panel_ch_raw = airlock_osd_state.osd_chan8_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 9) panel_ch_raw = airlock_osd_state.osd_chan9_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 10) panel_ch_raw = airlock_osd_state.osd_chan10_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 11) panel_ch_raw = airlock_osd_state.osd_chan11_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 12) panel_ch_raw = airlock_osd_state.osd_chan12_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 13) panel_ch_raw = airlock_osd_state.osd_chan13_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 14) panel_ch_raw = airlock_osd_state.osd_chan14_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 15) panel_ch_raw = airlock_osd_state.osd_chan15_raw;
      else if (eeprom_buffer.params.PWM_Panel_ch == 16) panel_ch_raw = airlock_osd_state.osd_chan16_raw;
      // Release the airlock mutex
      xSemaphoreGive(osd_state_airlock_mutex);
  }    
  
    // Did the panel value get changed by this routine?
    bool panel_value_changed = false;

    // Panel info access is controlled by ad-hoc mutex
    if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      int original_panel_value = adhoc_osd_state.current_panel;
      if (eeprom_buffer.params.PWM_Panel_mode == 0) {
        if ((panel_ch_raw > eeprom_buffer.params.PWM_Panel_value)) {
          if (!panel_trigger) {
            panel_trigger = true;
            adhoc_osd_state.current_panel++;
          }
        } else {
          panel_trigger = false;
        }
      } else {
        if (panel_ch_raw > 950 && panel_ch_raw < 2050) {
          adhoc_osd_state.current_panel = ceil((panel_ch_raw - 950) / (1100 / (float) eeprom_buffer.params.Max_panels));
        } else {
          adhoc_osd_state.current_panel = 1;
        }
      }
      
      panel_value_changed = adhoc_osd_state.current_panel != original_panel_value;
      
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
    }
}

uint32_t GetSystimeMS(void) {
  return (uint32_t)TICKS2MS(xTaskGetTickCount());
}

float Rad2Deg(float x) {
  return x * (180.0F / M_PI);
}

float Deg2Rad(float x) {
  return x * (M_PI / 180.0F);
}

void Delay_us(u32 nus) {
  u16 i = 0;
  while (nus--)
  {
    i = 12;
    while (i--);
  }
}

void checkDefaultParam() {
  bool bNeedUpdateFlash = false;
  //if new version add parameters, we should set them to default
  u16 curVer = eeprom_buffer.params.firmware_ver;

  //v1.0.5              Released: 2015-6-15
  if (curVer < 5 || curVer == 0xFFFF)
  {
    bNeedUpdateFlash = true;

    eeprom_buffer.params.Atti_mp_posX = 180;
    eeprom_buffer.params.Atti_mp_posY = 133;
    eeprom_buffer.params.Atti_mp_scale_real = 1;
    eeprom_buffer.params.Atti_mp_scale_frac = 0;
    eeprom_buffer.params.Atti_3D_posX = 180;
    eeprom_buffer.params.Atti_3D_posY = 133;
    eeprom_buffer.params.Atti_3D_scale_real = 1;
    eeprom_buffer.params.Atti_3D_scale_frac = 0;
    eeprom_buffer.params.Atti_3D_map_radius = 40;
    eeprom_buffer.params.osd_offsetY = 0;
    eeprom_buffer.params.osd_offsetX = 0;
  }

  if (eeprom_buffer.params.osd_offsetX > 20) {
    eeprom_buffer.params.osd_offsetX = 20;
    bNeedUpdateFlash = true;
  }
  if (eeprom_buffer.params.osd_offsetX == 0xFFFF) {
    eeprom_buffer.params.osd_offsetX = 0;
    bNeedUpdateFlash = true;
  }

  if (eeprom_buffer.params.osd_offsetY > 20) {
    eeprom_buffer.params.osd_offsetY = 20;
    bNeedUpdateFlash = true;
  }
  if (eeprom_buffer.params.osd_offsetY == 0xFFFF) {
    eeprom_buffer.params.osd_offsetY = 0;
    bNeedUpdateFlash = true;
  }

  if (eeprom_buffer.params.firmware_ver < 6) {
    eeprom_buffer.params.firmware_ver = 6;
    bNeedUpdateFlash = true;
  }

  if (eeprom_buffer.params.firmware_ver < 7) {
    eeprom_buffer.params.firmware_ver = 7;
    eeprom_buffer.params.Speed_scale_posY = 133;
    eeprom_buffer.params.Alt_Scale_posY = 133;
    eeprom_buffer.params.BattConsumed_en = 1;
    eeprom_buffer.params.BattConsumed_panel = 1;
    eeprom_buffer.params.BattConsumed_posX = 350;
    eeprom_buffer.params.BattConsumed_posY = 34;
    eeprom_buffer.params.BattConsumed_fontsize = 0;
    eeprom_buffer.params.BattConsumed_align = 2;
    eeprom_buffer.params.TotalTripDist_en = 1;
    eeprom_buffer.params.TotalTripDist_panel = 1;
    eeprom_buffer.params.TotalTripDist_posX = 350;
    eeprom_buffer.params.TotalTripDist_posY = 210;
    eeprom_buffer.params.TotalTripDist_fontsize = 0;
    eeprom_buffer.params.TotalTripDist_align = 2;
    bNeedUpdateFlash = true;
  }

  if (eeprom_buffer.params.firmware_ver < 8) {
    eeprom_buffer.params.firmware_ver = 8;
    eeprom_buffer.params.Max_panels = 3;
    eeprom_buffer.params.RSSI_type = 0;
    eeprom_buffer.params.Map_en = 1;
    eeprom_buffer.params.Map_panel = 4;
    eeprom_buffer.params.Map_radius = 120;
    eeprom_buffer.params.Map_fontsize = 1;
    eeprom_buffer.params.Map_H_align = 0;
    eeprom_buffer.params.Map_V_align = 0;
    bNeedUpdateFlash = true;
  }

  if (eeprom_buffer.params.firmware_ver < 9) {
    eeprom_buffer.params.firmware_ver = 9;
    eeprom_buffer.params.Relative_ALT_en = 1;
    eeprom_buffer.params.Relative_ALT_panel = 2;
    eeprom_buffer.params.Relative_ALT_posX = 5;
    eeprom_buffer.params.Relative_ALT_posY = 25;
    eeprom_buffer.params.Relative_ALT_fontsize = 0;
    eeprom_buffer.params.Relative_ALT_align = 0;
    eeprom_buffer.params.Alt_Scale_type = 1;
    eeprom_buffer.params.Air_Speed_en = 1;
    eeprom_buffer.params.Air_Speed_panel = 2;
    eeprom_buffer.params.Air_Speed_posX = 5;
    eeprom_buffer.params.Air_Speed_posY = 55;
    eeprom_buffer.params.Air_Speed_fontsize = 0;
    eeprom_buffer.params.Air_Speed_align = 0;
    eeprom_buffer.params.Spd_Scale_type = 0;
    bNeedUpdateFlash = true;
  }

  if (eeprom_buffer.params.firmware_ver < 10) {
    eeprom_buffer.params.firmware_ver = 10;
    eeprom_buffer.params.osd_offsetX_sign = 1;
    eeprom_buffer.params.LinkQuality_chan = 0;
    bNeedUpdateFlash = true;
  }
  
  if (eeprom_buffer.params.firmware_ver < 11) {
    eeprom_buffer.params.firmware_ver = 11;
    eeprom_buffer.params.version_splash_milliseconds_to_show = 5000;
    bNeedUpdateFlash = true;
  }
  
  if (eeprom_buffer.params.firmware_ver < 12) {
    eeprom_buffer.params.firmware_ver = 12;
    eeprom_buffer.params.error_alert_milliseconds_to_show = 1000;
    bNeedUpdateFlash = true;
  }
    
  if (eeprom_buffer.params.firmware_ver < 13) {
    eeprom_buffer.params.firmware_ver = 13;
    eeprom_buffer.params.RC_Channels_en = 0;
    eeprom_buffer.params.RC_Channels_panel = 2;
    eeprom_buffer.params.RC_Channels_posx = 55;
    eeprom_buffer.params.RC_Channels_posy = 40;
    bNeedUpdateFlash = true;
  }
  
  if (eeprom_buffer.params.firmware_ver < 14) {
    eeprom_buffer.params.firmware_ver = 14;
    eeprom_buffer.params.HomeDirectionDebugInfo_enabled = 0;
    eeprom_buffer.params.HomeDirectionDebugInfo_panel = 2;
    eeprom_buffer.params.HomeDirectionDebugInfo_posX = 65;
    eeprom_buffer.params.HomeDirectionDebugInfo_posY = 70;
    bNeedUpdateFlash = true;
  }  
    
  if (eeprom_buffer.params.firmware_ver <= 15) {
    eeprom_buffer.params.firmware_ver = 15;
    eeprom_buffer.params.Summary_panel_enabled = 0;
    eeprom_buffer.params.Summary_panel = 2;
    // Position is unused at the moment
    eeprom_buffer.params.Summary_posX = 0;
    eeprom_buffer.params.Summary_posY = 0;

    eeprom_buffer.params.Summary_switch_channel_enabled_mode = 0;
    eeprom_buffer.params.Summary_switch_channel = 0;
    eeprom_buffer.params.Summary_switch_channel_min_value = 0;
    eeprom_buffer.params.Summary_switch_channel_max_value = 0;
    eeprom_buffer.params.Summary_disarm_enabled_mode = 1;
    bNeedUpdateFlash = true;
  }    
  
  
  bool ret = false;
  if (bNeedUpdateFlash)
  {
    ret = StoreParams();
    if (!ret)
    {
      //TODO - handle flash write error here
    }
  }
}



bool test_force_clear_all_params(void) {
  volatile unsigned samples = 0;
  volatile unsigned vote = 0;

  GPIO_InitTypeDef gpio;
  gpio.GPIO_Pin = GPIO_Pin_11;
  gpio.GPIO_Mode = GPIO_Mode_IN;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &gpio);

  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_OUT;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);

  for (volatile unsigned cycles = 0; cycles < 10; cycles++) {
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    for (unsigned count = 0; count < 20; count++) {
      if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) != 0)
        vote++;
      samples++;
    }
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    for (unsigned count = 0; count < 20; count++) {
      if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
        vote++;
      samples++;
    }
  }

  /* revert the driver pin */
  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_IN;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);

  /* the idea here is to reject wire-to-wire coupling, so require > 90% agreement */
  if ((vote * 100) > (samples * 90))
    return true;

  return false;
}
