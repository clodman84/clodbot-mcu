#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the UI subsystem.
 *
 * Initializes the display (bsp_display_start/backlight/lv_scr_act styling),
 * builds the widget registry, and starts a dedicated UI task that owns all
 * LVGL access from here on.
 *
 * ui.c is the ONLY module in this codebase permitted to call lv_* or
 * bsp_display_* functions. Every other subsystem (internet, sound, sip,
 * ...) communicates with UI exclusively by posting to its own esp_event
 * base; it never touches LVGL directly and never calls into ui.c's
 * internals.
 *
 * Must be called once, before ui_subscribe_internet() or any other
 * ui_subscribe_*() call, and before any subsystem that might post events
 * UI is meant to react to.
 *
 * @return ESP_OK on success, or an error from event loop / task creation.
 */
esp_err_t ui_start(void);

/**
 * @brief Subscribe the UI task to INTERNET_EVENT state changes.
 *
 * Call after ui_start() and after internet_connect() has created the
 * default event loop. Wires INTERNET_EVENT_STATE_CHANGED into the UI
 * task's internal queue; the UI task updates the status label as events
 * arrive.
 *
 * Each subsystem gets its own ui_subscribe_<subsystem>() entry point like
 * this one -- add ui_subscribe_sound(), ui_subscribe_sip(), etc. following
 * the same shape as new subsystems come online.
 *
 * @return ESP_OK on success, or an error from esp_event_handler registration.
 */
esp_err_t ui_subscribe_internet(void);

/**
 * @brief Subscribe the UI task to MICROPHONE_EVENT state changes.
 *
 * Call after ui_start() and after internet_connect() has created the
 * default event loop. Wires INTERNET_EVENT_STATE_CHANGED into the UI
 * task's internal queue; the UI task updates the status label as events
 * arrive.
 *
 * Each subsystem gets its own ui_subscribe_<subsystem>() entry point like
 * this one -- add ui_subscribe_sound(), ui_subscribe_sip(), etc. following
 * the same shape as new subsystems come online.
 *
 * @return ESP_OK on success, or an error from esp_event_handler registration.
 */
esp_err_t ui_subscribe_microphone(void);

#ifdef __cplusplus
}
#endif
