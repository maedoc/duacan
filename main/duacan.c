#include "hal/twai_types.h"
#include "portmacro.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/twai.h>
#include <hal/cpu_hal.h>

// GPIOs
#define CAN0_TX 2
#define CAN0_RX 3
#define CAN1_TX 4
#define CAN1_RX 8

static void start_one_can(int id, int tx, int rx, twai_handle_t *handle);
static void start_pings();
static void duacan_pump();
static void can_log_status(int id);
static void can_log_alerts(int id);

static const char *TAG = "duacan";

static twai_handle_t can0, can1;
static int rx0count=0, rx1count=0;
static uint64_t can_cycles = 0;

IRAM_ATTR static void pingpong(const twai_handle_t can, const twai_message_t *frame) {
  if (frame->identifier == 0x123 && frame->data[0] > 0) {
    twai_message_t next = *frame;
    esp_err_t e_ping = twai_transmit_v2(can, &next, pdMS_TO_TICKS(1000));
    if (e_ping != ESP_OK)
      ESP_LOGE(TAG, "ping fail: %s", esp_err_to_name(e_ping));
  } else {
    ESP_LOGI(TAG, "pong! (rx counts are %d and %d)", rx0count, rx1count);
  }
}

UBaseType_t pump_stack_left;

// XXX next step is to pull frames into a ring buffer and have another task
// process, so we prioritize RX; TX can be split into slow/fast

IRAM_ATTR static void duacan_pump()
{
  while (1) {
    bool worked = false;
    uint32_t tik = cpu_hal_get_cycle_count();
    {
      twai_message_t frame = {0};
      esp_err_t r0 = twai_receive_v2(can0, &frame, pdMS_TO_TICKS(10));
      if (r0 == ESP_OK) {
        rx0count++;
        frame.data[0]--;
        pingpong(can0, &frame);
        worked = true;
      } else if (r0 != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Error receiving from CAN0: %s", esp_err_to_name(r0));
      }
    }
    {
      twai_message_t frame = {0};
      esp_err_t r1 = twai_receive_v2(can1, &frame, pdMS_TO_TICKS(10));
      if (r1 == ESP_OK) {
        rx1count++;
        frame.data[1]--;
        pingpong(can1, &frame);
        worked = true;
      } else if (r1 != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Error receiving from CAN1: %s", esp_err_to_name(r1));
      }
    }
    if (!worked) {
      can_log_alerts(0);
      can_log_alerts(1);
      pump_stack_left = uxTaskGetStackHighWaterMark(NULL);
    }
    can_cycles += cpu_hal_get_cycle_count() - tik;
  }
}


IRAM_ATTR static void read_twai_alerts_and_print()
{
  float delay_s = 0.2f;
  while(1) {
    /* can_log_status(0); */
    /* can_log_status(1); */
    // esp32c6 is 160 MHz, so can percentage cpu load is num cycles
    // divied by 160 million cycles*delay time in seconds
    float can_cpu_pct = (can_cycles / (delay_s*160*1000*1000.0f)) * 100.0f;
    ESP_LOGI(TAG, "can_cycles = %0.2f %% , stack left %d ", can_cpu_pct, pump_stack_left);
    can_cycles = 0;
    vTaskDelay(pdMS_TO_TICKS((int) 1000*delay_s));
  }
}

void app_main(void) {
  start_one_can(0, CAN0_TX, CAN0_RX, &can0);
  start_one_can(1, CAN1_TX, CAN1_RX, &can1);
  xTaskCreate(duacan_pump, "duacan_pump", 8192, NULL, 15, NULL);
  xTaskCreate(start_pings, "start_pings", 4096, NULL, 5, NULL);
  xTaskCreate(read_twai_alerts_and_print, "read_twai_alerts_and_print", 4096, NULL, 5, NULL);
}

static void start_one_can(int id, int tx, int rx, twai_handle_t *handle)
{
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g.alerts_enabled = TWAI_ALERT_ALL;
  g.controller_id = id;
  g.rx_queue_len = 128;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  ESP_LOGW(TAG, "twai_driver_install_v2(can%d) -> %s", id,
      esp_err_to_name(twai_driver_install_v2(&g, &t, &f, handle)));
  ESP_LOGW(TAG, "twai_start_v2(can%d) -> %s", id,
      esp_err_to_name(twai_start_v2(*handle)));
}

static void start_pings(void)
{
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "duacan test starting ping");
  int npings = 50;
  for (int i=0; i<npings; i++) {
    twai_message_t frame = {0};
    frame.identifier = 0x123;
    frame.data_length_code = 8;
    frame.data[0] = 250;
    frame.data[1] = 250;
    ESP_LOGW(TAG, "ping! transmit -> %s",
        esp_err_to_name(twai_transmit_v2(can0, &frame, pdMS_TO_TICKS(5000)))); // XXX high transmit patience is important to not timing out
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  ESP_LOGW(TAG, "should have total of %d*250=%d on each rx count", npings, npings*250);
  vTaskDelete(NULL);
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

static void can_log_status(int id)
{
  twai_handle_t can = id == 0 ? can0 : can1;
  twai_status_info_t status_info;
  twai_get_status_info_v2(can, &status_info);
  dump_status(status_info, id);
}

static void can_log_alerts(int id)
{
  twai_handle_t can = id == 0 ? can0 : can1;
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
