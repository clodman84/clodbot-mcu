#include "internet.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "internet";

ESP_EVENT_DEFINE_BASE (INTERNET_EVENT);
static EventGroupHandle_t           s_wifi_event_group = NULL;
static esp_event_handler_instance_t s_instance_any_id;
static esp_event_handler_instance_t s_instance_got_ip;
static esp_netif_t                 *s_sta_netif       = NULL;
static esp_timer_handle_t           s_reconnect_timer = NULL;

static bool                         s_initialized            = false;
static bool                         s_intentional_disconnect = false;
static int                          s_retry_num              = 0;

// Single writer for connection state. The event-group bit is derived
// mechanically from new_state so it can never disagree with the posted
// event -- callers never set/clear WIFI_CONNECTED_BIT themselves.
static void
set_state (internet_state_t new_state)
{
  if (new_state == INTERNET_STATE_CONNECTED)
    {
      xEventGroupSetBits (s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
  else
    {
      xEventGroupClearBits (s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
  esp_event_post (INTERNET_EVENT, INTERNET_EVENT_STATE_CHANGED, &new_state,
                  sizeof (new_state), portMAX_DELAY);
}

static uint32_t
backoff_ms (int retry)
{
  int      shift = retry < 6 ? retry : 6; // caps at 64x base before clamping
  uint32_t ms    = (uint32_t) RECONNECT_BASE_MS << shift;
  return ms > RECONNECT_MAX_MS ? RECONNECT_MAX_MS : ms;
}

static void
reconnect_timer_cb (void *arg)
{
  esp_wifi_connect ();
}

static void
schedule_reconnect (void)
{
  uint32_t delay_ms = backoff_ms (s_retry_num);
  s_retry_num++;
  ESP_LOGI (TAG, "reconnecting in %" PRIu32 " ms (attempt %d)", delay_ms,
            s_retry_num);
  esp_timer_start_once (s_reconnect_timer, (uint64_t) delay_ms * 1000);
}

static void
event_handler (void            *arg,
               esp_event_base_t event_base,
               int32_t          event_id,
               void            *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
      set_state (INTERNET_STATE_CONNECTING);
      esp_wifi_connect ();
    }
  else if (event_base == WIFI_EVENT &&
           event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
      wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *) event_data;

      if (s_intentional_disconnect)
        {
          ESP_LOGI (TAG, "disconnected (requested), reason %d", ev->reason);
          set_state (INTERNET_STATE_DISCONNECTED);
          return;
        }

      ESP_LOGW (TAG, "disconnected, reason %d, will retry", ev->reason);
      set_state (INTERNET_STATE_RECONNECTING);
      schedule_reconnect ();
    }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
      ESP_LOGI (TAG, "got ip: " IPSTR, IP2STR (&event->ip_info.ip));
      s_retry_num = 0;
      set_state (INTERNET_STATE_CONNECTED);
    }
}

static esp_err_t
nvs_init (void)
{
  esp_err_t ret = nvs_flash_init ();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK (nvs_flash_erase ());
      ret = nvs_flash_init ();
    }
  return ret;
}

static void
load_credentials (char  *ssid,
                  size_t ssid_len,
                  char  *pass,
                  size_t pass_len)
{
  nvs_handle_t h;
  if (nvs_open (WIFI_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK)
    {
      size_t    s_len = ssid_len;
      size_t    p_len = pass_len;
      esp_err_t s_err = nvs_get_str (h, "ssid", ssid, &s_len);
      esp_err_t p_err = nvs_get_str (h, "pass", pass, &p_len);
      nvs_close (h);
      if (s_err == ESP_OK && p_err == ESP_OK)
        {
          ESP_LOGI (TAG, "using NVS-provisioned credentials");
          return;
        }
    }
  snprintf (ssid, ssid_len, "%s", CONFIG_ESP_WIFI_SSID);
  snprintf (pass, pass_len, "%s", CONFIG_ESP_WIFI_PASSWORD);
  ESP_LOGI (TAG, "using Kconfig default credentials");
}

esp_err_t
internet_set_credentials (const char *ssid,
                          const char *password)
{
  nvs_handle_t h;
  esp_err_t    err = nvs_open (WIFI_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (err != ESP_OK)
    {
      return err;
    }
  err = nvs_set_str (h, "ssid", ssid);
  if (err == ESP_OK)
    {
      err = nvs_set_str (h, "pass", password);
    }
  if (err == ESP_OK)
    {
      err = nvs_commit (h);
    }
  nvs_close (h);
  return err;
}

esp_err_t
internet_connect (void)
{
  if (s_initialized)
    {
      ESP_LOGW (TAG, "internet_connect() called twice, ignoring");
      return ESP_ERR_INVALID_STATE;
    }

  ESP_ERROR_CHECK (nvs_init ());

  s_wifi_event_group       = xEventGroupCreate ();
  s_retry_num              = 0;
  s_intentional_disconnect = false;

  ESP_ERROR_CHECK (esp_netif_init ());
  ESP_ERROR_CHECK (esp_event_loop_create_default ());
  s_sta_netif = esp_netif_create_default_wifi_sta ();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT ();
  ESP_ERROR_CHECK (esp_wifi_init (&cfg));

  ESP_ERROR_CHECK (esp_event_handler_instance_register (
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &s_instance_any_id));
  ESP_ERROR_CHECK (esp_event_handler_instance_register (
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &s_instance_got_ip));

  const esp_timer_create_args_t timer_args = {
    .callback = &reconnect_timer_cb,
    .name     = "wifi_reconnect",
  };
  ESP_ERROR_CHECK (esp_timer_create (&timer_args, &s_reconnect_timer));

  wifi_config_t wifi_config;
  memset (&wifi_config, 0, sizeof (wifi_config));
  char ssid[32] = { 0 };
  char pass[64] = { 0 };
  load_credentials (ssid, sizeof (ssid), pass, sizeof (pass));
  snprintf ((char *) wifi_config.sta.ssid, sizeof (wifi_config.sta.ssid), "%s", ssid);
  snprintf ((char *) wifi_config.sta.password, sizeof (wifi_config.sta.password), "%s", pass);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.pmf_cfg.capable    = true;
  wifi_config.sta.pmf_cfg.required   = false;

  ESP_ERROR_CHECK (esp_wifi_set_mode (WIFI_MODE_STA));
  ESP_ERROR_CHECK (esp_wifi_set_config (WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK (esp_wifi_start ());

  ESP_LOGI (TAG, "connecting to SSID:%s", ssid);

  s_initialized = true;
  return ESP_OK;
}

bool
internet_wait_connected (uint32_t timeout_ms)
{
  if (s_wifi_event_group == NULL)
    {
      return false;
    }
  TickType_t ticks =
      (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS (timeout_ms);
  EventBits_t bits = xEventGroupWaitBits (s_wifi_event_group, WIFI_CONNECTED_BIT,
                                          pdFALSE, pdFALSE, ticks);
  return (bits & WIFI_CONNECTED_BIT) != 0;
}

void
internet_disconnect (void)
{
  if (! s_initialized)
    {
      return;
    }

  s_intentional_disconnect = true;
  esp_timer_stop (s_reconnect_timer);
  esp_timer_delete (s_reconnect_timer);
  s_reconnect_timer = NULL;

  esp_event_handler_instance_unregister (IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         s_instance_got_ip);
  esp_event_handler_instance_unregister (WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         s_instance_any_id);
  esp_wifi_disconnect ();
  esp_wifi_stop ();
  esp_wifi_deinit ();
  esp_netif_destroy_default_wifi (s_sta_netif);
  s_sta_netif = NULL;

  // Must run before the event group is torn down -- set_state() clears
  // WIFI_CONNECTED_BIT on the group as part of posting this transition.
  set_state (INTERNET_STATE_DISCONNECTED);

  vEventGroupDelete (s_wifi_event_group);
  s_wifi_event_group = NULL;

  s_initialized = false;
}

esp_err_t
internet_get_ip (char  *ip_str,
                 size_t len)
{
  if (s_sta_netif == NULL)
    {
      return ESP_FAIL;
    }

  esp_netif_ip_info_t ip_info;
  esp_err_t           err = esp_netif_get_ip_info (s_sta_netif, &ip_info);
  if (err != ESP_OK)
    {
      return err;
    }

  if (ip_info.ip.addr == 0)
    {
      return ESP_FAIL; // interface up but no IP yet
    }

  snprintf (ip_str, len, IPSTR, IP2STR (&ip_info.ip));
  return ESP_OK;
}

esp_err_t
internet_get_ssid (char  *ssid_str,
                   size_t len)
{
  if (s_sta_netif == NULL)
    {
      return ESP_FAIL;
    }

  esp_netif_ip_info_t ip_info;
  esp_err_t           err = esp_netif_get_ip_info (s_sta_netif, &ip_info);
  if (err != ESP_OK)
    {
      return err;
    }

  if (ip_info.ip.addr == 0)
    {
      return ESP_FAIL; // interface up but no IP yet
    }

  snprintf (ssid_str, len, IPSTR, IP2STR (&ip_info.ip));
  return ESP_OK;
}
