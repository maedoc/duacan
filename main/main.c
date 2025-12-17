#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "duacan.h"

// GPIOs
#define CAN0_TX 2
#define CAN0_RX 3
#define CAN1_TX 4
#define CAN1_RX 8

static void start_pings();

static const char *TAG = "main";

static int rx0=0, rx1=0;

// this can be provided as a handler to be called directly in pump hotspot but needs to be fast
IRAM_ATTR static void pingpong(const twai_handle_t can, const twai_message_t *frame) {
  twai_message_t next = *frame;
  if (can == can0)
  {
    rx0++;
    next.data[0]--;
  }
  else
  {
    rx1++;
    next.data[1]--;
  }
  if (frame->identifier == 0x123 && frame->data[0] > 0) {
    esp_err_t e_ping = twai_transmit_v2(can, &next, pdMS_TO_TICKS(1000));
    if (e_ping != ESP_OK)
      ESP_LOGE(TAG, "ping fail: %s", esp_err_to_name(e_ping));
  } else {
    ESP_LOGI(TAG, "pong! (rx counts are %d and %d)", rx0, rx1);
  }
}

// this can be used for high latency processing.
IRAM_ATTR static void reader()
{
  size_t tail0=0, tail1=0;
  while (1)
  {
    {
      twai_message_t frame = {0};
      while (duacan_read(can0, &tail0, &frame) == ESP_OK)
        pingpong(can0, &frame);
    }
    {
      twai_message_t frame = {0};
      while (duacan_read(can1, &tail1, &frame) == ESP_OK)
        pingpong(can1, &frame);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// tasknotify-based subscriber
IRAM_ATTR static void sub0(void*)
{
  duacan_subscribe(can0, xTaskGetCurrentTaskHandle());
  size_t tail=0;
  uint32_t notif_val;
  twai_message_t frame;
  while (1)
  {
    xTaskNotifyWait(0, 0xffffffff, &notif_val, portMAX_DELAY);
    while (duacan_read(can0, &tail, &frame) == ESP_OK)
      pingpong(can0, &frame);
  }
}

IRAM_ATTR static void sub1(void*)
{
  duacan_subscribe(can1, xTaskGetCurrentTaskHandle());
  size_t tail=0;
  uint32_t notif_val;
  twai_message_t frame;
  while (1)
  {
    xTaskNotifyWait(0, 0xffffffff, &notif_val, portMAX_DELAY);
    while (duacan_read(can1, &tail, &frame) == ESP_OK)
      pingpong(can1, &frame);
  }
}

void app_main(void) {
  twai_timing_config_t can_time = TWAI_TIMING_CONFIG_500KBITS();
  assert(ESP_OK == duacan_start(
        CAN0_TX, CAN0_RX, can_time,
        CAN1_TX, CAN1_RX, can_time,
        NULL));
  /* xTaskCreate(reader, "reader", 4096, NULL, 10, NULL); */
  xTaskCreate(sub0, "sub0", 4096, NULL, 10, NULL);
  xTaskCreate(sub1, "sub1", 4096, NULL, 10, NULL);
  xTaskCreate(start_pings, "start_pings", 4096, NULL, 5, NULL);
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
  vTaskDelay(pdMS_TO_TICKS(10000));
  ESP_LOGI(TAG, "starting regular pings but spaced by 2s");
  while (1) {
    twai_message_t frame = {0};
    frame.identifier = 0x123;
    frame.data_length_code = 8;
    frame.data[0] = 250;
    frame.data[1] = 250;
    ESP_LOGW(TAG, "ping! transmit -> %s",
        esp_err_to_name(twai_transmit_v2(can0, &frame, pdMS_TO_TICKS(5000))));
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  vTaskDelete(NULL);
}

