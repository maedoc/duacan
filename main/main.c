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

// XXX next step is to pull frames into a ring buffer and have another task
// process, so we prioritize RX; TX can be split into slow/fast
// we'll use xtasknotify to event the processing task with the current head
// but the task should maintain its own head.  since buffer is 1024 entries,
// use an rshift to figure out the current index, e.g. `index = head & 0x3FF`
// and lap count can be `head >> 10`.
typedef struct rbuf { uint32_t head; } rbuf_t;
IRAM_ATTR static uint32_t rbuf_idx(rbuf_t *b) { return b->head & 0x3FF; }
IRAM_ATTR static uint32_t rbuf_lap(rbuf_t *b) { return b->head >> 10; }

// well macro versions are better
#define RBUF_IDX(b) ((b)->head & 0x3FF)
#define RBUF_LAP(b) ((b)->head >> 10)

// build a xtasknotify multicast to multiple readers to keep pump fast
// use uint32_t atomic instructions to avoid critical sections or ISR hit
// suggestions in the gemini chat c6-can redesign
// segment tx into tx_urgent and tx_lazy
// on single core, critical section is ok

void app_main(void) {
  twai_timing_config_t can_time = TWAI_TIMING_CONFIG_500KBITS();
  assert(ESP_OK == duacan_start(
        CAN0_TX, CAN0_RX, can_time,
        CAN1_TX, CAN1_RX, can_time,
        pingpong));
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
  vTaskDelete(NULL);
}

