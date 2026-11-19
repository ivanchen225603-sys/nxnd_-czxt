#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ladar_bridge_frame_callback)(
    void* context,
    void* device,
    const uint8_t* frame,
    int frame_length);

typedef void (*ladar_bridge_status_callback)(
    void* context,
    void* device,
    uint32_t old_status,
    uint32_t status);

void* bridge_create(
    ladar_bridge_frame_callback frame_callback,
    ladar_bridge_status_callback status_callback,
    void* context);

void bridge_destroy(void* poller);
void* bridge_add_ladar(void* poller, const char* address, uint16_t port);
void bridge_run(void* poller);
void bridge_stop(void* poller);
int bridge_is_connected(void* device);
void bridge_want_connect(void* device);
int bridge_standby(void* device, uint16_t mode);
int bridge_parse118(
    void* device,
    const uint8_t* frame,
    int frame_length,
    void** points,
    int* point_count);
void bridge_free_points(void* points);

#ifdef __cplusplus
}
#endif
