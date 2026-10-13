#include "esp_err.h"
#include "esp_log.h"
#include "esp_websocket_client.h"

#include "include/internet.h"
#include "include/websocket.h"

static const char                   *TAG = "ws";
static esp_websocket_client_handle_t ws;

static void
ws_event (void            *arg,
          esp_event_base_t base,
          int32_t          id,
          void            *data)
{
  esp_websocket_event_data_t *e = data;
  switch (id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI (TAG, "connected");
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW (TAG, "disconnected");
      break;
    case WEBSOCKET_EVENT_DATA:
      // e->op_code: 0x1 text, 0x2 binary, 0x9/0xA ping/pong
      // e->data_ptr / e->data_len; e->payload_offset / e->payload_len for fragments
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
  esp_websocket_client_config_t cfg = {
    .uri                  = "ws://172.20.245.8:8765",
    .buffer_size          = 4096,
    .reconnect_timeout_ms = 2000,
    .network_timeout_ms   = 5000,
  };
  ws = esp_websocket_client_init (&cfg);
  esp_websocket_register_events (ws, WEBSOCKET_EVENT_ANY, ws_event, NULL);
  esp_websocket_client_start (ws);
  return ESP_OK;
}
