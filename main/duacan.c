#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <hal/cpu_hal.h>
#include <stdatomic.h>
#include <esp_timer.h>

#include "duacan.h"

// keep task handles
TaskHandle_t pump_task;
TaskHandle_t nanny_task;

// twai handles usable by other modules
twai_handle_t can0=NULL, can1=NULL;

static uint32_t rx0ok=0, rx1ok=0, rx0err=0, rx1err=0;
static uint32_t can_cycles = 0;

static const char *TAG = "duacan";

static void dump_status(twai_status_info_t info, int id) {
  char *state;
  switch (info.state) {
  case TWAI_STATE_STOPPED:
    state = "STOPPED";
    break;
  case TWAI_STATE_RUNNING:
    state = "RUNNING";
    break;
  case TWAI_STATE_BUS_OFF:
    state = "OFF";
    break;
  case TWAI_STATE_RECOVERING:
    state = "RECOVERING";
    break;
  default:
    state = "UNKNOWN";
    ESP_LOGE(TAG, "can%d unknown twai state!", id);
  }
#define LOGSTUFF "can%d state=%s,msgs_to_tx=%" PRIu32 ",msgs_to_rx=%" PRIu32 "," \
           "tx_error_counter=%" PRIu32 ",rx_error_counter=%" PRIu32 ",tx_failed_count=%" PRIu32 "," \
           "rx_missed_count=%" PRIu32 ",rx_overrun_count=%" PRIu32 ",arb_lost_count=%" PRIu32 "," \
           "bus_error_count=%" PRIu32, id, state, info.msgs_to_tx, info.msgs_to_rx, info.tx_error_counter, \
           info.rx_error_counter, info.tx_failed_count, info.rx_missed_count, \
           info.rx_overrun_count, info.arb_lost_count, info.bus_error_count
  if ( (info.state != TWAI_STATE_RUNNING)
      || (info.tx_error_counter > 0)
      || (info.rx_error_counter > 0)
      || (info.tx_failed_count > 0)
      || (info.rx_missed_count > 0)
      || (info.rx_overrun_count > 0)
      || (info.arb_lost_count > 0)
      || (info.bus_error_count > 0) )
    ESP_LOGW(TAG, LOGSTUFF);
  else
    ESP_LOGD(TAG, LOGSTUFF);
#undef LOGSTUFF
}

twai_status_info_t duacan_log_status(twai_handle_t can)
{
  twai_status_info_t status;
  twai_get_status_info_v2(can, &status);
  dump_status(status, can == can0 ? 0 : 1);
  switch (status.state) {
  case TWAI_STATE_BUS_OFF:
    ESP_LOGW(TAG, "can%d twai in BUS_OFF, initiating recovery",
             can == can0 ? 0 : 1);
    twai_initiate_recovery_v2(can);
    break;
  case TWAI_STATE_RECOVERING:
    ESP_LOGW(TAG, "can%d twai recovering...", can == can0 ? 0 : 1);
    break;
  case TWAI_STATE_STOPPED:
    ESP_LOGW(TAG, "can%d twai stopped, starting...", can == can0 ? 0 : 1);
    twai_start_v2(can);
    break;
  case TWAI_STATE_RUNNING:
    break;
  default:
    break;
  }
  return status;
}

void duacan_log_alerts(twai_handle_t can)
{
  int id = can == can0 ? 0 : 1;
  uint32_t alerts;
  esp_err_t alert_ret =
      twai_read_alerts_v2(can, &alerts, pdMS_TO_TICKS(1));
  if (alert_ret == ESP_OK) {
#define log_alert(flag)                                                        \
  if (alerts & flag)                                                           \
  ESP_LOGW(TAG, "(id=%d) twai alert %s", id, #flag)
    /* log_alert(TWAI_ALERT_TX_IDLE); */
    /* log_alert(TWAI_ALERT_TX_SUCCESS); */
    /* log_alert(TWAI_ALERT_RX_DATA); */
    log_alert(TWAI_ALERT_BELOW_ERR_WARN);
    log_alert(TWAI_ALERT_ERR_ACTIVE);
    log_alert(TWAI_ALERT_RECOVERY_IN_PROGRESS);
    log_alert(TWAI_ALERT_BUS_RECOVERED);
    log_alert(TWAI_ALERT_ARB_LOST);
    log_alert(TWAI_ALERT_ABOVE_ERR_WARN);
    log_alert(TWAI_ALERT_BUS_ERROR);
    log_alert(TWAI_ALERT_TX_FAILED);
    log_alert(TWAI_ALERT_RX_QUEUE_FULL);
    log_alert(TWAI_ALERT_ERR_PASS);
    log_alert(TWAI_ALERT_BUS_OFF);
    log_alert(TWAI_ALERT_RX_FIFO_OVERRUN);
    log_alert(TWAI_ALERT_TX_RETRIED);
    log_alert(TWAI_ALERT_PERIPH_RESET);
#undef log_alert
  } else if (alert_ret == ESP_ERR_TIMEOUT) {
    ESP_LOGD(TAG, "(id=%d) no alerts", id);
  } else {
    ESP_LOGE(TAG, "(id=%d) unable to read alerts", id);
  }
}


// to ensure handler is fast, processing is slow,
// XXX next step is to pull frames into a ring buffer and have another task
// process, so we prioritize RX; TX can be split into slow/fast
// we'll use xtasknotify to event the processing task with the current head
// but the task should maintain its own head.  since buffer is 1024 entries,
// use an rshift to figure out the current index, e.g. `index = head & 0x3FF`
// and lap count can be `head >> 10`.

#include <stdatomic.h>

typedef struct rb_slot {
  atomic_uint seq; // even -> ok, odd -> being written
  twai_message_t msg;
} rb_slot_t;

typedef struct duacan_rbuf {
  atomic_size_t head;
  rb_slot_t buf[1024];
} rbuf_t;

static rbuf_t rx0buf = {0}, rx1buf = {0};

// assume high priority writer, low priority reader, readers may not keep up
IRAM_ATTR static void rb_write(rbuf_t *rb, const twai_message_t *frame)
{
  size_t head = atomic_load_explicit(&rb->head, memory_order_relaxed);
  uint32_t idx = head & 0x3FF; // == head % 1024
  rb_slot_t *slot = &rb->buf[idx];
  // seq stores most of head, last bit is write-in-progress flag
  atomic_store_explicit(&slot->seq, (head << 1) | 1, memory_order_relaxed);
  atomic_thread_fence(memory_order_release);
  // do non-atmomic write
  slot->msg = *frame;
  // clear last bit -> write finished
  atomic_store_explicit(&slot->seq, head << 1, memory_order_release);
  // advance head
  atomic_store_explicit(&rb->head, head + 1, memory_order_release);
}

IRAM_ATTR static esp_err_t rb_read(rbuf_t *rb, size_t *tail, twai_message_t *frame)
{
  size_t head = atomic_load_explicit(&rb->head, memory_order_acquire);
  if (*tail == head) return ESP_ERR_TIMEOUT; // not really timeout but no new data
  if (head - *tail > 1024) // skip ahead for slow reader TODO stats
    *tail = head - 1024;
  rb_slot_t *slot = &rb->buf[*tail & 0x3FF];
  uint32_t seq0=0, seq1=0;
  do {
    seq0 = atomic_load_explicit(&slot->seq, memory_order_acquire);
    if (seq0 & 1) continue; // being written
    *frame = slot->msg; // non-atomic read protected by seq even/odd
    atomic_thread_fence(memory_order_acquire);
    seq1 = atomic_load_explicit(&slot->seq, memory_order_relaxed);
  } while (seq0 != seq1); // changed during read, try again
  size_t src_head = seq0 >> 1;
  if (src_head != *tail)
    *tail = src_head + 1; // skip ahead due to lap
  else
    (*tail)++; // advance reader
  return ESP_OK;
}

IRAM_ATTR esp_err_t duacan_read(twai_handle_t c, size_t *tail, twai_message_t *frame)
{
  rbuf_t *rb = (c == can0) ? &rx0buf : &rx1buf;
  return rb_read(rb, tail, frame);
}

TaskHandle_t subs[2][4] = {0};

esp_err_t duacan_subscribe(twai_handle_t can, TaskHandle_t reader_task)
{
  int id = can == can0 ? 0 : 1;
  for (int i=0; i<4; i++) {
    if (subs[id][i] == NULL) {
      subs[id][i] = reader_task;
      return ESP_OK;
    }
  }
  ESP_LOGW(TAG, "duacan_subscribe: can%d has no more subscription slots", id);
  return ESP_ERR_NO_MEM;
}

// if handler is NULL, pump writes frames into 
IRAM_ATTR static void default_handler(const twai_handle_t can, const twai_message_t *frame)
{
  int id = can == can0 ? 0 : 1;
  rbuf_t *rb = can == can0 ? &rx0buf : &rx1buf;
  rb_write(rb, frame);
  // notify
  for (int i=0; i<4; i++)
    if (subs[id][i] != NULL)
      xTaskNotify(subs[id][i], rb->head, eSetValueWithOverwrite);
}

IRAM_ATTR static void pump(duacan_handler_t caller_handler)
{
  TickType_t patience = pdMS_TO_TICKS(10.f);
  duacan_handler_t handler = caller_handler != NULL ? caller_handler : default_handler;
  while (1) {
    bool worked = false;
    uint32_t tik = cpu_hal_get_cycle_count();
    {
      twai_message_t frame = {0};
      esp_err_t r0 = twai_receive_v2(can0, &frame, patience);
      if (r0 == ESP_OK) {
        rx0ok++;
        handler(can0, &frame);
        worked = true;
      } else if (r0 != ESP_ERR_TIMEOUT) {
        rx0err++;
      }
    }
    {
      twai_message_t frame = {0};
      esp_err_t r1 = twai_receive_v2(can1, &frame, patience);
      if (r1 == ESP_OK) {
        rx1ok++;
        handler(can1, &frame);
        worked = true;
      } else if (r1 != ESP_ERR_TIMEOUT) {
        rx1err++;
      }
    }
    can_cycles += cpu_hal_get_cycle_count() - tik;
    if (!worked) {
      vTaskDelay(patience);
    }
  }
}

IRAM_ATTR static void nanny()
{
  const TickType_t period = pdMS_TO_TICKS(1000);
  TickType_t last_wake = xTaskGetTickCount();
  // Clear initial counters so first print is accurate to the first second
  rx0ok = 0; rx1ok = 0; rx0err = 0; rx1err = 0; can_cycles = 0;
  while(1) {
    // This ensures the loop starts exactly 'period' ticks after the previous start
    vTaskDelayUntil(&last_wake, period);
    duacan_log_status(can0);
    duacan_log_alerts(can0);
    duacan_log_status(can1);
    duacan_log_alerts(can1);
    // simple integer math: cycles / 160M = cpu seconds used.
    // since period is 1s, cpu seconds * 100 = pct.
    // (cycles * 100) / 160000000 -> cycles / 1600000
    uint32_t can_cpu_pct = (uint32_t)(can_cycles / 1600000ULL);
    // Snapshots for logging
    uint32_t r0 = rx0ok, r1 = rx1ok, e0 = rx0err, e1 = rx1err;
    // Reset counters for the next second
    can_cycles = 0;
    rx0ok = 0; rx1ok = 0; rx0err = 0; rx1err = 0;
    ESP_LOGI(TAG, "can_cpu=%" PRIu32 "%%,pump_stack=%d,nanny_stack=%d,"
             "rx0=%" PRIu32 "/s,err0=%" PRIu32 "/s,rx1=%" PRIu32 "/s,err1=%" PRIu32 "/s",
        can_cpu_pct,
        uxTaskGetStackHighWaterMark(pump_task),
        uxTaskGetStackHighWaterMark(nanny_task),
        r0, e0, r1, e1
        );
  }
}

static esp_err_t start_one(int id, int tx, int rx, twai_timing_config_t time)
{
  twai_handle_t *can = id == 0 ? &can0 : &can1;
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g.alerts_enabled = TWAI_ALERT_ALL;
  g.controller_id = id;
  g.rx_queue_len = 128;
  g.tx_queue_len = 128;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t install_err, start_err;

  ESP_LOGW(TAG, "twai_driver_install_v2(can%d) -> %s", id,
      esp_err_to_name(install_err=twai_driver_install_v2(&g, &time, &f, can)));

  ESP_LOGW(TAG, "twai_start_v2(can%d) -> %s", id,
      esp_err_to_name(start_err=twai_start_v2(*can)));
  vTaskDelay(pdMS_TO_TICKS(100));
  twai_status_info_t status = duacan_log_status(*can);
  duacan_log_alerts(*can);
  if (status.state != TWAI_STATE_RUNNING) {
    ESP_LOGE(TAG, "can%d not running after start!", id);
    return ESP_FAIL;
  }
  return (install_err == ESP_OK && start_err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t duacan_start(
    int tx0, int rx0, twai_timing_config_t time0,
    int tx1, int rx1, twai_timing_config_t time1,
    duacan_handler_t handler)
{
  esp_err_t e0 = start_one(0, tx0, rx0, time0);
  esp_err_t e1 = start_one(1, tx1, rx1, time1);
  if (e0 != ESP_OK || e1 != ESP_OK)
    return ESP_FAIL;
  BaseType_t pump_ret = xTaskCreate((void(*)(void*)) pump, "duacan_pump", 8192, (void*) handler, 15, &pump_task);
  if (pump_ret != pdPASS)
    ESP_LOGE(TAG, "Failed to create duacan_pump task");
  BaseType_t nanny_ret = xTaskCreate(nanny, "duacan_nanny", 8192, NULL, 5, &nanny_task);
  if (nanny_ret != pdPASS)
    ESP_LOGE(TAG, "Failed to create duacan_nanny task");
  return (pump_ret == pdPASS && nanny_ret == pdPASS) ? ESP_OK : ESP_FAIL;
}
