#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <hal/cpu_hal.h>

#include "duacan.h"

// keep task handles
TaskHandle_t pump_task;
TaskHandle_t nanny_task;

// twai handles usable by other modules
twai_handle_t can0, can1;

static UBaseType_t pump_stack_left;
static uint32_t rx0ok=0, rx1ok=0, rx0err=0, rx1err=0;
static uint64_t can_cycles = 0;

static const char *TAG = "duacan";

static esp_err_t start_one(int id, int tx, int rx, twai_timing_config_t time)
{
  twai_handle_t can = id == 0 ? can0 : can1;
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g.alerts_enabled = TWAI_ALERT_ALL;
  g.controller_id = id;
  g.rx_queue_len = 128;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t install_err, start_err;
  ESP_LOGW(TAG, "twai_driver_install_v2(can%d) -> %s", id,
      esp_err_to_name(install_err=twai_driver_install_v2(&g, &time, &f, &can)));
  ESP_LOGW(TAG, "twai_start_v2(can%d) -> %s", id,
      esp_err_to_name(start_err=twai_start_v2(can)));
  return (install_err == ESP_OK && start_err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

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
    ESP_LOGE(TAG, "(id=%d) unknown twai state!", id);
  }
  ESP_LOGI(TAG,
           "(id=%d) twai status state=%s, "
           "msgs_to_tx=%" PRIu32 ", "
           "msgs_to_rx=%" PRIu32 ", "
           "tx_error_counter=%" PRIu32 ", "
           "rx_error_counter=%" PRIu32 ", "
           "tx_failed_count=%" PRIu32 ", "
           "rx_missed_count=%" PRIu32 ", "
           "rx_overrun_count=%" PRIu32 ", "
           "arb_lost_count=%" PRIu32 ", "
           "bus_error_count=%" PRIu32,
           id, state, info.msgs_to_tx, info.msgs_to_rx, info.tx_error_counter,
           info.rx_error_counter, info.tx_failed_count, info.rx_missed_count,
           info.rx_overrun_count, info.arb_lost_count, info.bus_error_count);
}

void duacan_log_status(twai_handle_t can)
{
  twai_status_info_t status_info;
  twai_get_status_info_v2(can, &status_info);
  dump_status(status_info, can == can0 ? 0 : 1);
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

typedef struct rbuf {
  uint32_t head;
  twai_message_t buf[1024];
} rbuf_t;

IRAM_ATTR static void duacan_pump(duacan_handler_t handler)
{
  while (1) {
    uint32_t tik = cpu_hal_get_cycle_count();
    {
      twai_message_t frame = {0};
      esp_err_t r0 = twai_receive_v2(can0, &frame, pdMS_TO_TICKS(10));
      if (r0 == ESP_OK) {
        rx0ok++;
        handler(can0, &frame);
      } else if (r0 != ESP_ERR_TIMEOUT) {
        rx0err++;
      }
    }
    {
      twai_message_t frame = {0};
      esp_err_t r1 = twai_receive_v2(can1, &frame, pdMS_TO_TICKS(10));
      if (r1 == ESP_OK) {
        rx1ok++;
        handler(can1, &frame);
      } else if (r1 != ESP_ERR_TIMEOUT) {
        rx1err++;
      }
    }
    can_cycles += cpu_hal_get_cycle_count() - tik;
  }
}

IRAM_ATTR static void duacan_nanny()
{
  float delay_s = 0.5f;
  while(1) {
    duacan_log_alerts(can0);
    duacan_log_alerts(can1);
    pump_stack_left = uxTaskGetStackHighWaterMark(NULL);
    // esp32c6 is 160 MHz, so can percentage cpu load is num cycles
    // divied by 160 million cycles*delay time in seconds
    float can_cpu_pct = (can_cycles / (delay_s*160*1000*1000.0f)) * 100.0f;
    can_cycles = 0;
    ESP_LOGI(TAG, "can_cycles = %0.2f %% , stack left %d ", can_cpu_pct, pump_stack_left);
    // log rx0ok/err and rx1ok/err
    ESP_LOGI(TAG, "can0 rx ok=%" PRIu32 ", err=%" PRIu32 ", can1 rx ok=%" PRIu32 ", err=%" PRIu32,
             rx0ok, rx0err, rx1ok, rx1err);
    // XXX TODO recover from bus error/off states
    vTaskDelay(pdMS_TO_TICKS((int) 1000*delay_s));
  }
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
  BaseType_t pump_ret = xTaskCreate((void(*)(void*)) duacan_pump, "duacan_pump", 8192, (void*) handler, 15, &pump_task);
  if (pump_ret != pdPASS)
    ESP_LOGE(TAG, "Failed to create duacan_pump task");
  BaseType_t nanny_ret = xTaskCreate(duacan_nanny, "duacan_nanny", 4096, NULL, 5, &nanny_task);
  if (nanny_ret != pdPASS)
    ESP_LOGE(TAG, "Failed to create duacan_nanny task");
  return (pump_ret == pdPASS && nanny_ret == pdPASS) ? ESP_OK : ESP_FAIL;
}
