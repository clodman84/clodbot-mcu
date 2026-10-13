#pragma once

#include "esp_err.h"
#include "esp_event.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// TODO: Add an AP scan method, something blocking and simple, controlled by the
// caller

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_NVS_NAMESPACE "wifi_cfg"
#define RECONNECT_BASE_MS 1000
#define RECONNECT_MAX_MS 60000


typedef enum {
  INTERNET_STATE_DISCONNECTED,
  INTERNET_STATE_CONNECTING,
  INTERNET_STATE_CONNECTED_NO_IP,
  INTERNET_STATE_CONNECTED,
  INTERNET_STATE_RECONNECTING,
} internet_state_t;

ESP_EVENT_DECLARE_BASE(INTERNET_EVENT);

enum {
  // event_data is internet_state_t*. This is the ONLY channel for observing
  // state changes -- there is no internet_get_state() poll. Subscribe via
  // esp_event_handler_instance_register(INTERNET_EVENT,
  // INTERNET_EVENT_STATE_CHANGED, ...) and track state locally if a
  // subsystem needs to check it later.
  INTERNET_EVENT_STATE_CHANGED,
};

/**
 * @brief Get the current IPv4 address assigned to the station interface.
 *
 * @param[out] ip_str Buffer to write the dotted-decimal address into.
 * @param      len    Size of ip_str. Must be at least 16 bytes.
 * @return ESP_OK on success, ESP_FAIL if not connected or netif unavailable.
 */
esp_err_t internet_get_ip(char *ip_str, size_t len);

/**
 * @brief Initialize NVS, netif, and the default event loop, then start WiFi
 *        station mode and kick off a connection attempt.
 *
 * Does NOT block until connected. Returns once the connect attempt has been
 * started. Retries on disconnect happen indefinitely in the background with
 * exponential backoff -- they never permanently give up. Use
 * internet_wait_connected() to block until an IP is actually assigned.
 *
 * Credentials come from NVS ("wifi_cfg" namespace, keys "ssid"/"pass") if
 * present, falling back to CONFIG_ESP_WIFI_SSID/CONFIG_ESP_WIFI_PASSWORD.
 *
 * @return ESP_OK on successful init/start, ESP_ERR_INVALID_STATE if already
 *         initialized, or an error from the underlying esp_wifi/esp_netif
 * calls.
 */
esp_err_t internet_connect(void);

/**
 * @brief Tear down the WiFi connection, free event handlers, and deinit
 *        WiFi/netif so internet_connect() can be called again cleanly.
 */
void internet_disconnect(void);

/**
 * @brief Block until WiFi has an IP.
 *
 * @param timeout_ms Max time to wait, or portMAX_DELAY to wait forever.
 * @return true if connected within timeout, false otherwise (including if
 *         internet_disconnect() was called or internet_connect() was never
 *         called).
 */
bool internet_wait_connected(uint32_t timeout_ms);

/**
 * @brief Persist WiFi credentials to NVS for internet_connect() to use on
 *        the next call. Does not affect an already-running connection.
 *
 * @param ssid     Null-terminated SSID, max 32 bytes.
 * @param password Null-terminated password, max 64 bytes.
 */
esp_err_t internet_set_credentials(const char *ssid, const char *password);

#ifdef __cplusplus
}
#endif
