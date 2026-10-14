#include "ui.h"

#include <stdint.h>
#include <stdio.h>

#include "bsp/esp-bsp.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "internet.h"
#include "lvgl.h"
#include "misc/lv_types.h"
#include "websocket.h"

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
// Adding a new subsystem (sound, sip, ...) means:
//   1. add an event tag + union member below
//   2. add an entry to status_id_t and k_status_meta (UI section)
//   3. add a render_<x>_state() that ends in status_set()
//   4. add a handler that copies+enqueues, a case in ui_dispatch(), and a
//      ui_subscribe_<x>() entry point
// The status row and the taskbar tray indicator are built from the table,
// so there is no layout code to touch. No other file needs to know LVGL
// exists.
//
// Adding a window means adding an entry to win_id_t and s_win (window
// registry, UI section), then building it with make_window(). Its taskbar
// button, open/close behaviour and "only one window up at a time" rule
// come from the registry.
// ---------------------------------------------------------------------

typedef enum
{
  UI_EVT_INTERNET_STATE,
  UI_EVT_WEBSOCKET_STATE,
  UI_EVT_WEBSOCKET_TEXT,
  // UI_EVT_SOUND_STATE, UI_EVT_SIP_STATE, ... go here as subsystems arrive
} ui_event_tag_t;

typedef struct
{
  ui_event_tag_t tag;
  union
  {
    internet_state_t  internet;
    websocket_state_t websocket;
    websocket_text_t  ws_text;
    // sound_state_t sound;
    // sip_state_t sip;
  } payload;
} ui_event_t;

#define UI_QUEUE_LEN 16
#define MSG_LOG_MAX  20

static QueueHandle_t s_ui_queue;
static TaskHandle_t  s_ui_task_handle;

// ---------------------------------------------------------------------
// Status registry
//
// Every subsystem the user should be able to see at a glance is one entry
// here. Each entry owns a row in the status window (name + detail text) and
// a compact indicator in the taskbar tray. The tray lives on the top layer,
// so it stays visible if more screens (call view, ...) are added later.
// ---------------------------------------------------------------------

typedef enum
{
  STATUS_WIFI,
  STATUS_SOCKET,
  // STATUS_SOUND, STATUS_SIP, ...
  STATUS_COUNT,
} status_id_t;

typedef struct
{
  const char *name;      // status window row title
  const char *tray_name; // short label in the taskbar tray
} status_meta_t;

static const status_meta_t k_status_meta[STATUS_COUNT] = {
  [STATUS_WIFI]   = { "WiFi",   "WiFi" },
  [STATUS_SOCKET] = { "Socket", "Sock" },
};

typedef struct
{
  lv_obj_t *row_dot;
  lv_obj_t *detail;
  lv_obj_t *tray_dot;
} status_widgets_t;

static status_widgets_t s_status[STATUS_COUNT];

// ---------------------------------------------------------------------
// Window registry
//
// Every window is one entry here. The windows are full-screen, so opening
// one closes the others. Each gets a taskbar button (built from this
// table) whose label gains a '*' when something new arrives while the
// window is closed.
// ---------------------------------------------------------------------

typedef enum
{
  WIN_STATUS,
  WIN_MESSAGES,
  WIN_COUNT,
} win_id_t;

typedef struct
{
  const char *title;  // title bar text
  const char *button; // taskbar button label
  lv_obj_t   *win;
  lv_obj_t   *client; // scrolling list inside the window
  lv_obj_t   *btn;    // taskbar button
} app_window_t;

static app_window_t s_win[WIN_COUNT] = {
  [WIN_STATUS]   = { .title = "Status",   .button = "Status" },
  [WIN_MESSAGES] = { .title = "Messages", .button = "Messages" },
};

static lv_obj_t *s_msg_empty; // "No messages yet" placeholder, until the first one

// ---------------------------------------------------------------------
// Windows 95 theme
//
// LVGL has no bevel primitive, so a bevel is faked: the object itself draws
// the top/left border, and a click-through overlay child draws the
// bottom/right border. Raised = light top-left, dark bottom-right; sunken
// is the reverse. The overlay is resized to the parent's full outer box on
// SIZE_CHANGED, so bevelled objects may have padding and any size mode.
// Never bevel a scrolling container: the overlay is a child and would
// scroll away with the content. Bevel a static wrapper instead.
// ---------------------------------------------------------------------

#define W95_DESKTOP  lv_color_hex (0x008080)
#define W95_FACE     lv_color_hex (0xC0C0C0)
#define W95_WHITE    lv_color_hex (0xFFFFFF)
#define W95_SHADOW   lv_color_hex (0x808080)
#define W95_BLACK    lv_color_hex (0x000000)
#define W95_NAVY     lv_color_hex (0x000080)
#define W95_TEXT_DIM lv_color_hex (0x404040)
#define W95_TRACK    lv_color_hex (0xE0E0E0) // scrollbar track (flat stand-in)

// Status colors. Color is never the only signal: every state also has text.
#define W95_OK   lv_color_hex (0x00C000)
#define W95_BUSY lv_color_hex (0xFFFF00)
#define W95_WARN lv_color_hex (0xFF8000)
#define W95_BAD  lv_color_hex (0xFF0000)

// Pixel font if enabled in menuconfig (LV_FONT_UNSCII_16), else default.
#if LV_FONT_UNSCII_16
#define W95_FONT (&lv_font_unscii_16)
#else
#define W95_FONT LV_FONT_DEFAULT
#endif

// Layout. Everything else is derived from these, so a different panel only
// needs these touched.
#define SCREEN_PAD   4
#define TASKBAR_H    30
#define TASK_BTN_W   80
#define TASK_BTN_H   24
#define TASK_BTN_GAP 4
#define TITLEBAR_H   24
#define CLOSE_W      20
#define CLOSE_H      18
#define ROW_NAME_W   64
#define TRAY_H       22
#define TRAY_ITEM_W  58
#define TRAY_PAD     8
#define BEVEL_W      2
#define SB_W         16
#define SB_THUMB_MIN 16

// The overlay must cover the parent's whole outer box. LV_PCT(100) is
// relative to the parent's *content* area, so it breaks as soon as the
// parent has padding. Position is relative to the content area too, hence
// the negative offset by padding + border.
static void
bevel_fit_cb (lv_event_t *e)
{
  lv_obj_t *obj = lv_event_get_target (e);
  lv_obj_t *ov  = lv_event_get_user_data (e);
  int32_t   bw  = lv_obj_get_style_border_width (obj, LV_PART_MAIN);

  lv_obj_set_size (ov, lv_obj_get_width (obj), lv_obj_get_height (obj));
  lv_obj_set_pos (ov,
                  -(lv_obj_get_style_pad_left (obj, LV_PART_MAIN) + bw),
                  -(lv_obj_get_style_pad_top (obj, LV_PART_MAIN) + bw));
}

static void
bevel_set (lv_obj_t *obj,
           lv_obj_t *overlay,
           bool      raised)
{
  lv_obj_set_style_border_color (obj, raised ? W95_WHITE : W95_SHADOW, 0);
  lv_obj_set_style_border_color (overlay, raised ? W95_BLACK : W95_WHITE, 0);
}

// NOTE: set the object's own padding BEFORE calling this.
static lv_obj_t *
add_bevel (lv_obj_t *obj,
           bool      raised)
{
  lv_obj_set_style_border_width (obj, BEVEL_W, 0);
  lv_obj_set_style_border_opa (obj, LV_OPA_COVER, 0);
  lv_obj_set_style_border_side (obj, LV_BORDER_SIDE_TOP | LV_BORDER_SIDE_LEFT, 0);
  // The real border only covers top/left; mirror it as padding on
  // bottom/right so content is centred inside the bevelled box.
  lv_obj_set_style_pad_right (obj, lv_obj_get_style_pad_right (obj, LV_PART_MAIN) + BEVEL_W, 0);
  lv_obj_set_style_pad_bottom (obj, lv_obj_get_style_pad_bottom (obj, LV_PART_MAIN) + BEVEL_W, 0);

  lv_obj_t *ov = lv_obj_create (obj);
  lv_obj_remove_style_all (ov);
  lv_obj_add_flag (ov, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_clear_flag (ov, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_border_width (ov, BEVEL_W, 0);
  lv_obj_set_style_border_opa (ov, LV_OPA_COVER, 0);
  lv_obj_set_style_border_side (ov, LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT, 0);
  lv_obj_add_event_cb (obj, bevel_fit_cb, LV_EVENT_SIZE_CHANGED, ov);

  bevel_set (obj, ov, raised);
  return ov;
}

// Small square status light with a black outline.
static lv_obj_t *
make_indicator (lv_obj_t *parent,
                int32_t   size)
{
  lv_obj_t *dot = lv_obj_create (parent);
  lv_obj_remove_style_all (dot);
  lv_obj_set_size (dot, size, size);
  lv_obj_set_style_bg_color (dot, W95_BAD, 0);
  lv_obj_set_style_bg_opa (dot, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width (dot, 1, 0);
  lv_obj_set_style_border_color (dot, W95_BLACK, 0);
  lv_obj_set_style_border_opa (dot, LV_OPA_COVER, 0);
  lv_obj_set_style_border_side (dot, LV_BORDER_SIDE_FULL, 0);
  lv_obj_clear_flag (dot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
  return dot;
}

static void
button_state_cb (lv_event_t *e)
{
  lv_obj_t *btn     = lv_event_get_target (e);
  lv_obj_t *ov      = lv_event_get_user_data (e);
  lv_obj_t *lbl     = lv_obj_get_child (btn, 0);
  bool      pressed = (lv_event_get_code (e) == LV_EVENT_PRESSED);

  bevel_set (btn, ov, ! pressed);
  lv_obj_set_style_translate_x (lbl, pressed ? 1 : 0, 0);
  lv_obj_set_style_translate_y (lbl, pressed ? 1 : 0, 0);
}

// Raised button that sinks while pressed. Child 0 is the label and child 1
// is the bevel overlay -- button_state_cb and taskbar_sync rely on that.
// user_data is passed through to on_click.
static lv_obj_t *
make_button (lv_obj_t     *parent,
             const char   *text,
             int32_t       w,
             int32_t       h,
             lv_event_cb_t on_click,
             void         *user_data)
{
  lv_obj_t *btn = lv_obj_create (parent);
  lv_obj_remove_style_all (btn);
  lv_obj_set_size (btn, w, h);
  lv_obj_set_ext_click_area (btn, 4);
  lv_obj_set_style_bg_color (btn, W95_FACE, 0);
  lv_obj_set_style_bg_opa (btn, LV_OPA_COVER, 0);
  lv_obj_clear_flag (btn, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *lbl = lv_label_create (btn);
  lv_label_set_text (lbl, text);
  lv_obj_set_style_text_color (lbl, W95_BLACK, 0);
  lv_obj_center (lbl);

  lv_obj_t *ov = add_bevel (btn, true);
  lv_obj_add_event_cb (btn, button_state_cb, LV_EVENT_PRESSED, ov);
  lv_obj_add_event_cb (btn, button_state_cb, LV_EVENT_RELEASED, ov);
  lv_obj_add_event_cb (btn, button_state_cb, LV_EVENT_PRESS_LOST, ov);
  lv_obj_add_event_cb (btn, on_click, LV_EVENT_CLICKED, user_data);
  return btn;
}

// ---------------------------------------------------------------------
// Themed scrollbar
//
// LVGL's native scrollbar is a single flat thumb with one border color, so
// it can't be bevelled, and it overlays the client's right edge. Instead
// the native bar is turned off and a track + bevelled thumb live in their
// own column next to the list. The whole column is hidden while there is
// nothing to scroll, so the list gets the full width back. The thumb is
// display-only; touch scrolling still works on the content.
// ---------------------------------------------------------------------

static void
scrollbar_sync (lv_obj_t *client)
{
  lv_obj_t *thumb = lv_obj_get_user_data (client);

  if (thumb == NULL)
    return;

  lv_obj_t *track  = lv_obj_get_parent (thumb);
  int32_t   top    = lv_obj_get_scroll_top (client);
  int32_t   bottom = lv_obj_get_scroll_bottom (client);
  int32_t   view   = lv_obj_get_height (client);
  int32_t   range  = top + bottom;

  if (range <= 0 || view <= 0)
    {
      if (! lv_obj_has_flag (track, LV_OBJ_FLAG_HIDDEN))
        lv_obj_add_flag (track, LV_OBJ_FLAG_HIDDEN); // nothing to scroll
      return;
    }

  if (lv_obj_has_flag (track, LV_OBJ_FLAG_HIDDEN))
    lv_obj_clear_flag (track, LV_OBJ_FLAG_HIDDEN);

  int32_t trk     = lv_obj_get_height (track);
  int32_t thumb_h = trk * view / (view + range);
  if (thumb_h < SB_THUMB_MIN)
    thumb_h = SB_THUMB_MIN;
  if (thumb_h > trk)
    thumb_h = trk;

  lv_obj_set_size (thumb, SB_W, thumb_h);
  lv_obj_set_pos (thumb, 0, (trk - thumb_h) * top / range);
}

static void
scrollbar_sync_cb (lv_event_t *e)
{
  scrollbar_sync (lv_event_get_target (e));
}

// ---------------------------------------------------------------------
// Window management
// ---------------------------------------------------------------------

// A taskbar button is "down" (sunken) while its window is open, like Win95.
static void
taskbar_sync (void)
{
  for (int i = 0; i < WIN_COUNT; i++)
    {
      bool      open = ! lv_obj_has_flag (s_win[i].win, LV_OBJ_FLAG_HIDDEN);
      lv_obj_t *lbl  = lv_obj_get_child (s_win[i].btn, 0);

      bevel_set (s_win[i].btn, lv_obj_get_child (s_win[i].btn, 1), ! open);
      lv_obj_set_style_translate_x (lbl, open ? 1 : 0, 0);
      lv_obj_set_style_translate_y (lbl, open ? 1 : 0, 0);
    }
}

// Mark a window's taskbar button as having something new ("Messages*").
static void
window_set_unread (win_id_t id,
                   bool     unread)
{
  lv_obj_t *lbl = lv_obj_get_child (s_win[id].btn, 0);

  if (unread)
    lv_label_set_text_fmt (lbl, "%s*", s_win[id].button);
  else
    lv_label_set_text (lbl, s_win[id].button);
}

static void
window_set_open (win_id_t id,
                 bool     open)
{
  if (open)
    {
      // Windows are full-screen, so only one can be up at a time.
      for (int i = 0; i < WIN_COUNT; i++)
        {
          if (i != (int) id)
            lv_obj_add_flag (s_win[i].win, LV_OBJ_FLAG_HIDDEN);
        }
      lv_obj_clear_flag (s_win[id].win, LV_OBJ_FLAG_HIDDEN);
      lv_obj_update_layout (s_win[id].win);
      scrollbar_sync (s_win[id].client);
      window_set_unread (id, false);
    }
  else
    {
      lv_obj_add_flag (s_win[id].win, LV_OBJ_FLAG_HIDDEN);
    }
  taskbar_sync ();
}

// user_data of both callbacks is the win_id_t.
static void
on_close_clicked (lv_event_t *e)
{
  window_set_open ((win_id_t) (intptr_t) lv_event_get_user_data (e), false);
}

static void
on_task_clicked (lv_event_t *e)
{
  win_id_t id = (win_id_t) (intptr_t) lv_event_get_user_data (e);

  window_set_open (id, lv_obj_has_flag (s_win[id].win, LV_OBJ_FLAG_HIDDEN));
}

// Finger slid off the button: button_state_cb popped it up, restore the
// state that matches the window.
static void
on_task_press_lost (lv_event_t *e)
{
  taskbar_sync ();
}

// A window: raised frame, navy title bar with a close button, and a
// sunken white list field (vertical flex column) with a themed scrollbar
// beside it. Rows added beyond what fits simply scroll, so the window
// never has to be resized when a new item appears. Fills s_win[id].win
// and s_win[id].client.
static void
make_window (lv_obj_t *parent,
             win_id_t  id)
{
  lv_obj_t *win = lv_obj_create (parent);
  lv_obj_remove_style_all (win);
  lv_obj_set_size (win, LV_PCT (100), LV_PCT (100));
  lv_obj_set_style_bg_color (win, W95_FACE, 0);
  lv_obj_set_style_bg_opa (win, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all (win, 3, 0);
  lv_obj_set_style_pad_row (win, 3, 0);
  lv_obj_set_flex_flow (win, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag (win, LV_OBJ_FLAG_SCROLLABLE);
  add_bevel (win, true);

  // Title bar: navy, title on the left, close on the right.
  lv_obj_t *bar = lv_obj_create (win);
  lv_obj_remove_style_all (bar);
  lv_obj_set_size (bar, LV_PCT (100), TITLEBAR_H);
  lv_obj_set_style_bg_color (bar, W95_NAVY, 0);
  lv_obj_set_style_bg_opa (bar, LV_OPA_COVER, 0);
  lv_obj_clear_flag (bar, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *t = lv_label_create (bar);
  lv_label_set_text (t, s_win[id].title);
  lv_obj_set_style_text_color (t, W95_WHITE, 0);
  lv_obj_align (t, LV_ALIGN_LEFT_MID, 6, 0);

  lv_obj_t *x = make_button (bar, "X", CLOSE_W, CLOSE_H, on_close_clicked,
                             (void *) (intptr_t) id);
  lv_obj_align (x, LV_ALIGN_RIGHT_MID, -2, 0);

  // Body row: list field on the left, scrollbar column on the right.
  lv_obj_t *body = lv_obj_create (win);
  lv_obj_remove_style_all (body);
  lv_obj_set_width (body, LV_PCT (100));
  lv_obj_set_flex_grow (body, 1);
  lv_obj_set_style_pad_column (body, 3, 0);
  lv_obj_set_flex_flow (body, LV_FLEX_FLOW_ROW);
  lv_obj_clear_flag (body, LV_OBJ_FLAG_SCROLLABLE);

  // Sunken field. It is static, so its bevel overlay doesn't scroll; the
  // scrolling client sits inside it.
  lv_obj_t *field = lv_obj_create (body);
  lv_obj_remove_style_all (field);
  lv_obj_set_height (field, LV_PCT (100));
  lv_obj_set_flex_grow (field, 1);
  lv_obj_set_style_bg_color (field, W95_WHITE, 0);
  lv_obj_set_style_bg_opa (field, LV_OPA_COVER, 0);
  lv_obj_clear_flag (field, LV_OBJ_FLAG_SCROLLABLE);
  add_bevel (field, false);

  lv_obj_t *c = lv_obj_create (field);
  lv_obj_remove_style_all (c);
  lv_obj_set_size (c, LV_PCT (100), LV_PCT (100));
  lv_obj_set_flex_flow (c, LV_FLEX_FLOW_COLUMN);
  lv_obj_add_flag (c, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir (c, LV_DIR_VER);
  lv_obj_set_scrollbar_mode (c, LV_SCROLLBAR_MODE_OFF); // we draw our own

  lv_obj_t *track = lv_obj_create (body);
  lv_obj_remove_style_all (track);
  lv_obj_set_size (track, SB_W, LV_PCT (100));
  lv_obj_set_style_bg_color (track, W95_TRACK, 0);
  lv_obj_set_style_bg_opa (track, LV_OPA_COVER, 0);
  lv_obj_clear_flag (track, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *thumb = lv_obj_create (track);
  lv_obj_remove_style_all (thumb);
  lv_obj_add_flag (thumb, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_clear_flag (thumb, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size (thumb, SB_W, SB_THUMB_MIN);
  lv_obj_set_style_bg_color (thumb, W95_FACE, 0);
  lv_obj_set_style_bg_opa (thumb, LV_OPA_COVER, 0);
  add_bevel (thumb, true);

  lv_obj_set_user_data (c, thumb);
  lv_obj_add_event_cb (c, scrollbar_sync_cb, LV_EVENT_SCROLL, NULL);
  lv_obj_add_event_cb (c, scrollbar_sync_cb, LV_EVENT_SIZE_CHANGED, NULL);
  lv_obj_add_event_cb (c, scrollbar_sync_cb, LV_EVENT_LAYOUT_CHANGED, NULL);

  s_win[id].win    = win;
  s_win[id].client = c;
}

// One status row in the list: light, name, then the detail text (state,
// SSID/IP, ...) filling the rest of the width. The row sizes itself to its
// text, so one-line and two-line statuses sit tight together, separated by
// a hairline.
static void
make_status_row (lv_obj_t   *parent,
                 status_id_t id)
{
  lv_obj_t *row = lv_obj_create (parent);
  lv_obj_remove_style_all (row);
  lv_obj_set_width (row, LV_PCT (100));
  lv_obj_set_height (row, LV_SIZE_CONTENT);
  lv_obj_set_style_pad_hor (row, 8, 0);
  lv_obj_set_style_pad_ver (row, 6, 0);
  lv_obj_set_style_pad_column (row, 8, 0);
  lv_obj_set_style_border_width (row, 1, 0);
  lv_obj_set_style_border_color (row, W95_FACE, 0);
  lv_obj_set_style_border_opa (row, LV_OPA_COVER, 0);
  lv_obj_set_style_border_side (row, LV_BORDER_SIDE_BOTTOM, 0);
  lv_obj_clear_flag (row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow (row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align (row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START,
                         LV_FLEX_ALIGN_START);

  s_status[id].row_dot = make_indicator (row, 12);
  lv_obj_set_style_translate_y (s_status[id].row_dot, 2, 0); // optical align with first text line

  lv_obj_t *name = lv_label_create (row);
  lv_obj_set_width (name, ROW_NAME_W);
  lv_label_set_text (name, k_status_meta[id].name);
  lv_obj_set_style_text_color (name, W95_BLACK, 0);

  lv_obj_t *detail = lv_label_create (row);
  lv_obj_set_flex_grow (detail, 1);
  lv_label_set_long_mode (detail, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color (detail, W95_TEXT_DIM, 0);
  lv_label_set_text (detail, "Disconnected");
  s_status[id].detail = detail;
}

// Taskbar on the top layer: it persists across screens. The right side is
// the tray, one compact "name + light" per status entry, so status stays
// visible even with every window closed. The left side holds one button
// per window in the registry; add global actions (like mute) here as they
// appear.
static void
build_taskbar (void)
{
  lv_obj_t *bar = lv_obj_create (lv_layer_top ());
  lv_obj_remove_style_all (bar);
  lv_obj_set_size (bar, LV_PCT (100), TASKBAR_H);
  lv_obj_align (bar, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color (bar, W95_FACE, 0);
  lv_obj_set_style_bg_opa (bar, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width (bar, 2, 0);
  lv_obj_set_style_border_color (bar, W95_WHITE, 0);
  lv_obj_set_style_border_opa (bar, LV_OPA_COVER, 0);
  lv_obj_set_style_border_side (bar, LV_BORDER_SIDE_TOP, 0);
  // The top layer is not a child of the screen, so it does not inherit the
  // screen's font.
  lv_obj_set_style_text_font (bar, W95_FONT, 0);
  lv_obj_clear_flag (bar, LV_OBJ_FLAG_SCROLLABLE);

  // One button per window, left to right.
  int32_t x = 4;
  for (int i = 0; i < WIN_COUNT; i++)
    {
      s_win[i].btn = make_button (bar, s_win[i].button, TASK_BTN_W, TASK_BTN_H,
                                  on_task_clicked, (void *) (intptr_t) i);
      lv_obj_align (s_win[i].btn, LV_ALIGN_LEFT_MID, x, 0);
      lv_obj_add_event_cb (s_win[i].btn, on_task_press_lost,
                           LV_EVENT_PRESS_LOST, NULL);
      x += TASK_BTN_W + TASK_BTN_GAP;
    }

  // Width = left/right pad + the bevel's border and its mirrored padding
  // (add_bevel adds BEVEL_W to the right padding) + the items.
  lv_obj_t *tray = lv_obj_create (bar);
  lv_obj_remove_style_all (tray);
  lv_obj_set_size (tray, 2 * TRAY_PAD + 2 * BEVEL_W + STATUS_COUNT * TRAY_ITEM_W,
                   TRAY_H);
  lv_obj_align (tray, LV_ALIGN_RIGHT_MID, -4, 0);
  lv_obj_set_style_bg_color (tray, W95_FACE, 0);
  lv_obj_set_style_bg_opa (tray, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_left (tray, TRAY_PAD, 0);
  lv_obj_set_style_pad_right (tray, TRAY_PAD, 0);
  lv_obj_clear_flag (tray, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow (tray, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align (tray, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                         LV_FLEX_ALIGN_CENTER);
  add_bevel (tray, false);

  for (int i = 0; i < STATUS_COUNT; i++)
    {
      lv_obj_t *item = lv_obj_create (tray);
      lv_obj_remove_style_all (item);
      lv_obj_set_size (item, TRAY_ITEM_W, LV_PCT (100));
      lv_obj_set_style_pad_column (item, 6, 0);
      lv_obj_clear_flag (item, LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_set_flex_flow (item, LV_FLEX_FLOW_ROW);
      lv_obj_set_flex_align (item, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                             LV_FLEX_ALIGN_CENTER);

      lv_obj_t *lbl = lv_label_create (item);
      lv_label_set_text (lbl, k_status_meta[i].tray_name);
      lv_obj_set_style_text_color (lbl, W95_BLACK, 0);

      s_status[i].tray_dot = make_indicator (item, 10);
    }
}

static void
build_widgets (void)
{
  lv_obj_t *scr = lv_scr_act ();
  lv_obj_set_style_bg_color (scr, W95_DESKTOP, 0);
  lv_obj_set_style_bg_opa (scr, LV_OPA_COVER, 0);
  lv_obj_set_style_text_font (scr, W95_FONT, 0);
  lv_obj_clear_flag (scr, LV_OBJ_FLAG_SCROLLABLE);
  // Desktop margin, with the bottom margin clearing the taskbar.
  lv_obj_set_style_pad_all (scr, SCREEN_PAD, 0);
  lv_obj_set_style_pad_bottom (scr, TASKBAR_H + SCREEN_PAD, 0);

  // Status window: open at boot.
  make_window (scr, WIN_STATUS);
  for (int i = 0; i < STATUS_COUNT; i++)
    {
      make_status_row (s_win[WIN_STATUS].client, (status_id_t) i);
    }

  // Messages window: closed at boot, filled by render_websocket_text().
  make_window (scr, WIN_MESSAGES);
  lv_obj_add_flag (s_win[WIN_MESSAGES].win, LV_OBJ_FLAG_HIDDEN);

  s_msg_empty = lv_label_create (s_win[WIN_MESSAGES].client);
  lv_label_set_text (s_msg_empty, "No messages yet");
  lv_obj_set_style_text_color (s_msg_empty, W95_TEXT_DIM, 0);
  lv_obj_set_style_pad_all (s_msg_empty, 8, 0);

  build_taskbar ();
  taskbar_sync ();

  // Nothing has scrolled yet, so no SCROLL event will position the thumbs;
  // resolve the layout and sync them once.
  lv_obj_update_layout (scr);
  for (int i = 0; i < WIN_COUNT; i++)
    {
      scrollbar_sync (s_win[i].client);
    }
}

// Single writer for a status item's appearance. Row and tray indicator are
// updated together so they can never disagree. Only called from the UI task.
static void
status_set (status_id_t id,
            lv_color_t  color,
            const char *detail)
{
  // bsp_display_lock/unlock wraps LVGL's mutex -- this function is only
  // ever called from the UI task, which is the single owner of that lock
  // (see ui_task() below).
  bsp_display_lock (0);
  lv_label_set_text (s_status[id].detail, detail);
  lv_obj_set_style_bg_color (s_status[id].row_dot, color, 0);
  lv_obj_set_style_bg_color (s_status[id].tray_dot, color, 0);
  // Row height follows its text; re-resolve and refresh the scrollbar now
  // rather than waiting for the next LVGL timer tick.
  lv_obj_update_layout (s_win[WIN_STATUS].win);
  scrollbar_sync (s_win[WIN_STATUS].client);
  bsp_display_unlock ();
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
            snprintf (buf, sizeof (buf), "Connected to %s\n%s",
                      (const char *) ap_info.ssid, ip_str);
          }
        else if (err == ESP_OK)
          {
            snprintf (buf, sizeof (buf), "Connected to %s",
                      (const char *) ap_info.ssid);
          }
        else
          {
            // ap_info lookup failed -- state says CONNECTED but we can't name
            // the AP. Still must write buf on every path.
            snprintf (buf, sizeof (buf), "Connected");
          }
        color = W95_OK;
        break;
      }
    case INTERNET_STATE_CONNECTED_NO_IP:
      snprintf (buf, sizeof (buf), "Connected, no IP");
      color = W95_BUSY;
      break;
    case INTERNET_STATE_CONNECTING:
      snprintf (buf, sizeof (buf), "Connecting...");
      color = W95_BUSY;
      break;
    case INTERNET_STATE_RECONNECTING:
      snprintf (buf, sizeof (buf), "Reconnecting...");
      color = W95_WARN;
      break;
    case INTERNET_STATE_DISCONNECTED:
    default:
      snprintf (buf, sizeof (buf), "Disconnected");
      color = W95_BAD;
      break;
    }

  status_set (STATUS_WIFI, color, buf);
}

// ---------------------------------------------------------------------
// Websocket -> UI rendering
// ---------------------------------------------------------------------

static void
render_websocket_state (websocket_state_t state)
{
  const char *text;
  lv_color_t  color;

  switch (state)
    {
    case WEBSOCKET_STATE_CONNECTED:
      text  = "Connected";
      color = W95_OK;
      break;
    case WEBSOCKET_STATE_CONNECTING:
      text  = "Connecting...";
      color = W95_BUSY;
      break;
    case WEBSOCKET_STATE_DISCONNECTED:
    default:
      text  = "Disconnected";
      color = W95_BAD;
      break;
    }

  status_set (STATUS_SOCKET, color, text);
}

// Appends one message to the Messages window. The log keeps the newest
// MSG_LOG_MAX lines and follows the latest one. If the window is closed,
// its taskbar button gets a '*' until it is opened.
static void
render_websocket_text (const websocket_text_t *msg)
{
  lv_obj_t *client = s_win[WIN_MESSAGES].client;

  bsp_display_lock (0);

  if (s_msg_empty != NULL)
    {
      lv_obj_del (s_msg_empty); // lv_obj_delete on LVGL 9
      s_msg_empty = NULL;
    }

  // Cap the log by dropping the oldest line.
  if (lv_obj_get_child_cnt (client) >= MSG_LOG_MAX)
    lv_obj_del (lv_obj_get_child (client, 0));

  lv_obj_t *lbl = lv_label_create (client);
  lv_obj_set_width (lbl, LV_PCT (100));
  lv_label_set_long_mode (lbl, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color (lbl, W95_BLACK, 0);
  lv_obj_set_style_pad_hor (lbl, 8, 0);
  lv_obj_set_style_pad_ver (lbl, 5, 0);
  lv_obj_set_style_border_width (lbl, 1, 0);
  lv_obj_set_style_border_color (lbl, W95_FACE, 0);
  lv_obj_set_style_border_opa (lbl, LV_OPA_COVER, 0);
  lv_obj_set_style_border_side (lbl, LV_BORDER_SIDE_BOTTOM, 0);
  if (msg->truncated)
    lv_label_set_text_fmt (lbl, "%s...", msg->text);
  else
    lv_label_set_text (lbl, msg->text); // copies the string

  lv_obj_update_layout (client);
  lv_obj_scroll_to_view (lbl, LV_ANIM_OFF); // follow the newest line
  scrollbar_sync (client);

  if (lv_obj_has_flag (s_win[WIN_MESSAGES].win, LV_OBJ_FLAG_HIDDEN))
    window_set_unread (WIN_MESSAGES, true);

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
    case UI_EVT_WEBSOCKET_STATE:
      render_websocket_state (evt->payload.websocket);
      break;
    case UI_EVT_WEBSOCKET_TEXT:
      render_websocket_text (&evt->payload.ws_text);
      break;
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

static void
on_websocket_event (void            *arg,
                    esp_event_base_t base,
                    int32_t          id,
                    void            *event_data)
{
  ui_event_t evt = {
    .tag               = UI_EVT_WEBSOCKET_STATE,
    .payload.websocket = *(websocket_state_t *) event_data,
  };

  if (xQueueSend (s_ui_queue, &evt, 0) != pdTRUE)
    {
      ESP_LOGW (TAG, "ui queue full, dropped websocket state event");
    }
}

static void
on_websocket_text_event (void            *arg,
                         esp_event_base_t base,
                         int32_t          id,
                         void            *event_data)
{
  ui_event_t evt      = { .tag = UI_EVT_WEBSOCKET_TEXT };
  evt.payload.ws_text = *(websocket_text_t *) event_data;

  if (xQueueSend (s_ui_queue, &evt, 0) != pdTRUE)
    {
      ESP_LOGW (TAG, "ui queue full, dropped websocket text");
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

esp_err_t
ui_subscribe_websocket (void)
{
  esp_err_t err = esp_event_handler_register (WEBSOCKET_EVENT,
                                              WEBSOCKET_EVENT_STATE_CHANGED,
                                              &on_websocket_event, NULL);
  if (err != ESP_OK)
    return err;

  return esp_event_handler_register (WEBSOCKET_EVENT,
                                     WEBSOCKET_EVENT_RECEIVED_TEXT,
                                     &on_websocket_text_event, NULL);
}
