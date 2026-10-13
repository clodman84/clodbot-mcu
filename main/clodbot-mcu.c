#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "internet.h"
#include "ui.h"
#include "websocket.h"

static const char *TAG = "clodbot-mcu";

void
app_main (void)
{
  // ui_start() must run first: it creates the queue+task that
  // ui_subscribe_internet()'s handler enqueues into.
  esp_err_t err = ui_start ();
  if (err != ESP_OK)
    {
      ESP_LOGE (TAG, "ui_start failed, halting");
      return;
    }
  err = internet_connect ();
  if (err != ESP_OK)
    {
      ESP_LOGE (TAG, "wifi connect failed, halting");
      return;
    }

  ui_subscribe_internet ();
  ws_start ();
}
