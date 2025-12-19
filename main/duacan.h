#pragma once
#include <esp_err.h>
#include <driver/twai.h>

// twai handles usable by other modules
extern twai_handle_t can0, can1;

// debug
twai_status_info_t duacan_log_status(twai_handle_t can);
void duacan_log_alerts(twai_handle_t can);

// start duacan tasks after creating can0 and can1.
// nanny is prio 5.
esp_err_t duacan_start(
    int tx0, int rx0, twai_timing_config_t time0,
    int tx1, int rx1, twai_timing_config_t time1);

typedef struct duacan_buf duacan_buf_t;

// read a frame from the ring buffer, with reader index *tail
esp_err_t duacan_read(twai_handle_t can, size_t *tail, twai_message_t *frame);

// register a task to be notified of new frames
esp_err_t duacan_subscribe(twai_handle_t can, TaskHandle_t reader_task);
