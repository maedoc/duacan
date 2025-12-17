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

typedef struct rbuf {
  atomic_size_t head;
  rb_slot_t buf[1024];
} rbuf_t;

static rbuf_t rx0buf = {0}, rx1buf = {0};

IRAM_ATTR static uint32_t rbuf_idx(rbuf_t *b) { return b->head & 0x3FF; }
IRAM_ATTR static uint32_t rbuf_lap(rbuf_t *b) { return b->head >> 10; }

// well macro versions are better
#define RBUF_IDX(b) ((b)->head & 0x3FF)
#define RBUF_LAP(b) ((b)->head >> 10)

// assume high priority writer, low priority reader, readers may not keep up
IRAM_ATTR void rb_write(rbuf_t *rb, const twai_message_t *frame)
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

IRAM_ATTR esp_err_t rb_read(rbuf_t *rb, size_t *tail, twai_message_t *frame)
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

