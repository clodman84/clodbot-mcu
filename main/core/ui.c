#include "ui.h"

#include "bsp/esp-bsp.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "internet.h"
#include "lvgl.h"

static const char *TAG = "ui";

// ---------------------------------------------------------------------
// Event plumbing
//
// esp_event handlers run in the event loop's own task context, not the UI
// task. They must never block on the LVGL lock -- if they did, a slow UI
// update would stall every other subsystem's event delivery, not just
// UI's. So handlers only ever copy the payload into a fixed-size struct
// and push it onto s_ui_queue; the UI task is the sole consumer that
// drains the queue and does the actual bsp_display_lock()/lv_* work.
//
// Adding a new subsystem (sound, sip, ...) means: add a tag, add a union
// member, add a handler that copies+enqueues, add a case in
// ui_dispatch(). No other file needs to know LVGL exists.
// ---------------------------------------------------------------------

typedef enum
{
  UI_EVT_INTERNET_STATE,
  // UI_EVT_SOUND_STATE, UI_EVT_SIP_STATE, ... go here as subsystems arrive
} ui_event_tag_t;

typedef struct
{
  ui_event_tag_t tag;
  union
  {
    internet_state_t internet;
    // sound_state_t sound;
    // sip_state_t sip;
  } payload;
} ui_event_t;

#define UI_QUEUE_LEN 16

static QueueHandle_t s_ui_queue;
static TaskHandle_t  s_ui_task_handle;

// ---------------------------------------------------------------------
// Widget registry
//
// One struct holding every lv_obj_t the UI owns. A second widget (e.g. a
// SIP call indicator) is a new field here plus a new case in
// ui_dispatch() -- not a new global.
// ---------------------------------------------------------------------

typedef struct
{
  lv_obj_t *status_label;
} ui_widgets_t;

static ui_widgets_t s_widgets;

static void
build_widgets (void)
{
  lv_obj_set_style_bg_color (lv_scr_act (), lv_color_black (), 0);
  lv_obj_set_style_bg_opa (lv_scr_act (), LV_OPA_COVER, 0);

  s_widgets.status_label = lv_label_create (lv_scr_act ());
  lv_obj_set_width (s_widgets.status_label, LV_PCT (90));
  lv_label_set_long_mode (s_widgets.status_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align (s_widgets.status_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center (s_widgets.status_label);
  lv_label_set_text (s_widgets.status_label, "WiFi: disconnected");
}

// ---------------------------------------------------------------------
// Internet -> UI rendering
// ---------------------------------------------------------------------

static void
render_internet_state (internet_state_t state)
{
  char       buf[96];
  lv_color_t color;

  switch (state)
    {
    case INTERNET_STATE_CONNECTED:
      {
        char             ip_str[16];
        wifi_ap_record_t ap_info;
        esp_err_t        err = esp_wifi_sta_get_ap_info (&ap_info);

        if (err == ESP_OK && internet_get_ip (ip_str, sizeof (ip_str)) == ESP_OK)
          {
            snprintf (buf, sizeof (buf), "BoBo: connected to %s\n%s", ap_info.ssid,
                      ip_str);
          }
        else if (err == ESP_OK)
          {
            snprintf (buf, sizeof (buf), "BoBo: connected to %s", ap_info.ssid);
          }
        else
          {
            // ap_info lookup failed -- state says CONNECTED but we can't name
            // the AP. Still must write buf on every path.
            snprintf (buf, sizeof (buf), "BoBo: connected");
          }
        color = lv_palette_main (LV_PALETTE_GREEN);
        break;
      }
    case INTERNET_STATE_CONNECTED_NO_IP:
      snprintf (buf, sizeof (buf), "WiFi: connected, no IP");
      color = lv_palette_main (LV_PALETTE_YELLOW);
      break;
    case INTERNET_STATE_CONNECTING:
      snprintf (buf, sizeof (buf), "WiFi: connecting...");
      color = lv_palette_main (LV_PALETTE_YELLOW);
      break;
    case INTERNET_STATE_RECONNECTING:
      snprintf (buf, sizeof (buf), "WiFi: reconnecting...");
      color = lv_palette_main (LV_PALETTE_ORANGE);
      break;
    case INTERNET_STATE_DISCONNECTED:
    default:
      snprintf (buf, sizeof (buf), "WiFi: disconnected");
      color = lv_palette_main (LV_PALETTE_RED);
      break;
    }

  // bsp_display_lock/unlock wraps LVGL's mutex -- this function is only
  // ever called from the UI task, which is the single owner of that lock
  // (see ui_task() below).
  bsp_display_lock (0);
  lv_label_set_text (s_widgets.status_label, buf);
  lv_obj_set_style_text_color (s_widgets.status_label, color, 0);
  bsp_display_unlock ();
}

// ---------------------------------------------------------------------
// Dispatch + task
// ---------------------------------------------------------------------

static void
ui_dispatch (const ui_event_t *evt)
{
  switch (evt->tag)
    {
    case UI_EVT_INTERNET_STATE:
      render_internet_state (evt->payload.internet);
      break;
    // case UI_EVT_SOUND_STATE: render_sound_state(evt->payload.sound); break;
    default:
      ESP_LOGW (TAG, "unhandled ui event tag %d", evt->tag);
      break;
    }
}

static void
ui_task (void *arg)
{
  ui_event_t evt;
  for (;;)
    {
      if (xQueueReceive (s_ui_queue, &evt, portMAX_DELAY) == pdTRUE)
        {
          ui_dispatch (&evt);
        }
    }
}

// ---------------------------------------------------------------------
// Subsystem event handlers -- copy payload, enqueue, return immediately.
// Run in the esp_event loop task, NOT the UI task. Must never block on
// the LVGL lock.
// ---------------------------------------------------------------------

static void
on_internet_event (void            *arg,
                   esp_event_base_t base,
                   int32_t          id,
                   void            *event_data)
{
  ui_event_t evt = {
    .tag              = UI_EVT_INTERNET_STATE,
    .payload.internet = *(internet_state_t *) event_data,
  };

  // Non-blocking send: a full queue means the UI task is badly stuck, and
  // blocking the event loop task here would just cascade the stall to
  // every other subsystem waiting on esp_event delivery. Drop and log
  // instead.
  if (xQueueSend (s_ui_queue, &evt, 0) != pdTRUE)
    {
      ESP_LOGW (TAG, "ui queue full, dropped internet state event");
    }
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

esp_err_t
ui_start (void)
{
  s_ui_queue = xQueueCreate (UI_QUEUE_LEN, sizeof (ui_event_t));
  if (s_ui_queue == NULL)
    {
      return ESP_ERR_NO_MEM;
    }

  bsp_display_start ();
  bsp_display_backlight_on ();

  bsp_display_lock (0);
  build_widgets ();
  bsp_display_unlock ();

  // Priority 4: one below network_dependent_task (5) in clodbot-mcu.c, so
  // networked work isn't preempted by UI rendering. Revisit once
  // sound/sip tasks exist and there's an actual priority scheme to weigh
  // this against.
  BaseType_t ok = xTaskCreate (ui_task, "ui_task", 4096, NULL, 4,
                               &s_ui_task_handle);
  if (ok != pdPASS)
    {
      vQueueDelete (s_ui_queue);
      s_ui_queue = NULL;
      return ESP_ERR_NO_MEM;
    }

  return ESP_OK;
}

esp_err_t
ui_subscribe_internet (void)
{
  return esp_event_handler_register (INTERNET_EVENT,
                                     INTERNET_EVENT_STATE_CHANGED,
                                     &on_internet_event, NULL);
}
