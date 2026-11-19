#include "ladar_bridge.h"

#include <ladar_api.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

namespace {

struct bridge_device;

struct bridge_poller {
    ILadarPoller* poller = nullptr;
    ladar_bridge_frame_callback frame_callback = nullptr;
    ladar_bridge_status_callback status_callback = nullptr;
    void* context = nullptr;
    std::mutex mutex;
    std::vector<std::unique_ptr<bridge_device>> devices;
};

struct bridge_device {
    bridge_poller* owner = nullptr;
    ILadar* ladar = nullptr;
};

bridge_device* find_device(bridge_poller* owner, ILadar* ladar)
{
    if (owner == nullptr || ladar == nullptr) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(owner->mutex);
    for (const auto& device : owner->devices) {
        if (device->ladar == ladar) {
            return device.get();
        }
    }
    return nullptr;
}

} // namespace

extern "C" void* bridge_create(
    ladar_bridge_frame_callback frame_callback,
    ladar_bridge_status_callback status_callback,
    void* context)
{
    auto* result = new bridge_poller();
    result->frame_callback = frame_callback;
    result->status_callback = status_callback;
    result->context = context;
    result->poller = CreateLadarPoller();
    if (result->poller == nullptr) {
        delete result;
        return nullptr;
    }
    return result;
}

extern "C" void bridge_destroy(void* opaque_poller)
{
    auto* owner = static_cast<bridge_poller*>(opaque_poller);
    if (owner == nullptr) {
        return;
    }

    if (owner->poller != nullptr) {
        owner->poller->Stop();
        owner->poller->ClearAllLadar();
        DeleteLadarPoller(owner->poller);
        owner->poller = nullptr;
    }
    delete owner;
}

extern "C" void* bridge_add_ladar(
    void* opaque_poller,
    const char* address,
    uint16_t port)
{
    auto* owner = static_cast<bridge_poller*>(opaque_poller);
    if (owner == nullptr || owner->poller == nullptr || address == nullptr) {
        return nullptr;
    }

    auto device = std::make_unique<bridge_device>();
    device->owner = owner;
    auto* device_result = device.get();

    auto on_frame = [owner](ILadar* ladar, const uint8_t* frame, int frame_length) {
        auto* device = find_device(owner, ladar);
        if (device != nullptr && owner->frame_callback != nullptr) {
            owner->frame_callback(owner->context, device, frame, frame_length);
        }
    };
    auto on_status = [owner](ILadar* ladar, uint32_t old_status, uint32_t status) {
        auto* device = find_device(owner, ladar);
        if (device != nullptr && owner->status_callback != nullptr) {
            owner->status_callback(owner->context, device, old_status, status);
        }
    };

    device->ladar = owner->poller->AddLadar(address, port, on_frame, on_status);
    if (device->ladar == nullptr) {
        return nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(owner->mutex);
        owner->devices.emplace_back(std::move(device));
    }
    return device_result;
}

extern "C" void bridge_run(void* opaque_poller)
{
    auto* owner = static_cast<bridge_poller*>(opaque_poller);
    if (owner != nullptr && owner->poller != nullptr) {
        owner->poller->Run();
    }
}

extern "C" void bridge_stop(void* opaque_poller)
{
    auto* owner = static_cast<bridge_poller*>(opaque_poller);
    if (owner != nullptr && owner->poller != nullptr) {
        owner->poller->Stop();
    }
}

extern "C" int bridge_is_connected(void* opaque_device)
{
    auto* device = static_cast<bridge_device*>(opaque_device);
    return device != nullptr && device->ladar != nullptr && device->ladar->IsConnected();
}

extern "C" void bridge_want_connect(void* opaque_device)
{
    auto* device = static_cast<bridge_device*>(opaque_device);
    if (device != nullptr && device->ladar != nullptr) {
        device->ladar->WantConnect();
    }
}

extern "C" int bridge_standby(void* opaque_device, uint16_t mode)
{
    auto* device = static_cast<bridge_device*>(opaque_device);
    if (device == nullptr || device->ladar == nullptr) {
        return 0;
    }
    return device->ladar->Standby(mode);
}

extern "C" int bridge_parse118(
    void* opaque_device,
    const uint8_t* frame,
    int frame_length,
    void** points,
    int* point_count)
{
    if (points == nullptr || point_count == nullptr) {
        return -1;
    }
    *points = nullptr;
    *point_count = 0;

    auto* device = static_cast<bridge_device*>(opaque_device);
    if (device == nullptr || device->ladar == nullptr || frame == nullptr || frame_length <= 0) {
        return -1;
    }

    ladar_data3d_t data;
    const auto result = device->ladar->GetData(data, frame, frame_length);
    if (result <= 0 || data.points.empty()) {
        return result;
    }

    const auto bytes = data.points.size() * sizeof(float3_t);
    auto* copy = std::malloc(bytes);
    if (copy == nullptr) {
        return -1;
    }
    std::memcpy(copy, data.points.data(), bytes);
    *points = copy;
    *point_count = static_cast<int>(data.points.size());
    return result;
}

extern "C" void bridge_free_points(void* points)
{
    std::free(points);
}
