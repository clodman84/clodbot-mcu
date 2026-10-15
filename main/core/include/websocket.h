#pragma once

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define WEBSOCKET_CONNECTED_BIT BIT0
#define WEBSOCKET_TEXT_MAX 256

  typedef enum
  {
    WEBSOCKET_STATE_DISCONNECTED,
    WEBSOCKET_STATE_CONNECTING,
    WEBSOCKET_STATE_CONNECTED,
  } websocket_state_t;

  typedef struct
  {
    uint16_t len;       // bytes in text, excluding the NUL
    bool     truncated; // message was longer than WEBSOCKET_TEXT_MAX - 1
    char     text[WEBSOCKET_TEXT_MAX];
  } websocket_text_t;

  ESP_EVENT_DECLARE_BASE (WEBSOCKET_EVENT);

  enum
  {
    // event_data is internet_state_t*. This is the ONLY channel for observing
    // state changes -- there is no internet_get_state() poll. Subscribe via
    // esp_event_handler_instance_register(INTERNET_EVENT,
    // INTERNET_EVENT_STATE_CHANGED, ...) and track state locally if a
    // subsystem needs to check it later.
    WEBSOCKET_EVENT_STATE_CHANGED,
    WEBSOCKET_EVENT_RECEIVED_TEXT,
  };

  /**
   * @brief Block until WebSocket is connected.
   *
   * @param timeout_ms Max time to wait, or portMAX_DELAY to wait forever.
   * @return true if connected within timeout, false otherwise
   * */
  bool      websocket_wait_connected(uint32_t timeout_ms);

  esp_err_t ws_start (void);

#ifdef __cplusplus
}
#endif
