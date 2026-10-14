#include "esp_err.h"
#include "esp_log.h"
#include "esp_websocket_client.h"

#include "include/internet.h"
#include "include/websocket.h"
#include "string.h"

static const char                   *TAG = "ws";
static esp_websocket_client_handle_t ws;

ESP_EVENT_DEFINE_BASE (WEBSOCKET_EVENT);
static EventGroupHandle_t s_websocket_event_group = NULL;

static void
set_state (websocket_state_t new_state)
{
  if (new_state == WEBSOCKET_STATE_CONNECTED)
    {
      xEventGroupSetBits (s_websocket_event_group, WEBSOCKET_CONNECTED_BIT);
    }
  else
    {
      xEventGroupClearBits (s_websocket_event_group, WEBSOCKET_CONNECTED_BIT);
    }
  esp_event_post (WEBSOCKET_EVENT, WEBSOCKET_EVENT_STATE_CHANGED, &new_state,
                  sizeof (new_state), portMAX_DELAY);
}

static void
post_text (const esp_websocket_event_data_t *e)
{
  // A message bigger than buffer_size arrives as several events
  // (payload_offset > 0). Only whole messages are handled for now.
  if (e->payload_offset != 0 || e->data_len != e->payload_len)
    {
      ESP_LOGW (TAG, "fragmented message dropped (%d bytes)", e->payload_len);
      return;
    }

  websocket_text_t msg = { 0 };
  msg.len              = e->data_len < sizeof (msg.text) - 1 ? e->data_len : sizeof (msg.text) - 1;
  msg.truncated        = e->data_len > msg.len;
  memcpy (msg.text, e->data_ptr, msg.len);

  // This runs in the websocket task. Short timeout so a stuck event loop
  // can't wedge the socket.
  if (esp_event_post (WEBSOCKET_EVENT, WEBSOCKET_EVENT_RECEIVED_TEXT, &msg,
                      sizeof (msg), pdMS_TO_TICKS (100)) != ESP_OK)
    {
      ESP_LOGW (TAG, "event loop busy, dropped text message");
    }
}

static void
event_handler (void            *arg,
               esp_event_base_t base,
               int32_t          id,
               void            *event_data)
{
  esp_websocket_event_data_t *e = event_data;
  switch (id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
      set_state (WEBSOCKET_STATE_CONNECTED);
      ESP_LOGI (TAG, "connected");
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      set_state (WEBSOCKET_STATE_DISCONNECTED);
      ESP_LOGW (TAG, "disconnected");
      break;
    case WEBSOCKET_EVENT_BEFORE_CONNECT:
      set_state (WEBSOCKET_STATE_CONNECTING);
      ESP_LOGW (TAG, "connecting");
      break;
    case WEBSOCKET_EVENT_DATA:
      // e->op_code: 0x1 text, 0x2 binary, 0x9/0xA ping/pong
      // e->data_ptr / e->data_len; e->payload_offset / e->payload_len for fragments
      if (e->op_code == 0x1)
        {
          post_text (e);
        }
      break;
    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGE (TAG, "error");
      break;
    }
}

esp_err_t
ws_start (void)
{
  if (! internet_wait_connected (portMAX_DELAY))
    {
      ESP_LOGE (TAG, "network never came up");
      vTaskDelete (NULL);
      return ESP_ERR_TIMEOUT;
    }

  s_websocket_event_group = xEventGroupCreate ();
  if (s_websocket_event_group == NULL)
    {
      return ESP_ERR_NO_MEM;
    }

  esp_websocket_client_config_t cfg = {
    .uri                  = "ws://172.20.245.8:8765",
    .buffer_size          = 4096,
    .reconnect_timeout_ms = 2000,
    .network_timeout_ms   = 5000,
  };
  ws = esp_websocket_client_init (&cfg);
  esp_websocket_register_events (ws, WEBSOCKET_EVENT_ANY, event_handler, NULL);
  esp_websocket_client_start (ws);
  return ESP_OK;
}
