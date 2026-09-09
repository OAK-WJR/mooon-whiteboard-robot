#pragma once
#include "FreeRTOS.h"
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#define CONFIG_FREERTOS_UNICORE 0

namespace task_mock {
struct Task {
    std::mutex mutex;
    std::condition_variable changed;
    unsigned notifications = 0;
    std::thread thread;
};
struct Exit {};
inline thread_local Task* current = nullptr;
inline std::vector<std::unique_ptr<Task>> tasks;
inline bool failCreate = false;
inline void joinAll() {
    for (auto& task : tasks) if (task->thread.joinable()) task->thread.join();
    tasks.clear();
}
}
using TaskHandle_t = task_mock::Task*;
inline BaseType_t xTaskCreate(void(*entry)(void*), const char*, uint32_t stackBytes,
                              void* argument, UBaseType_t priority, TaskHandle_t* handle) {
    assert(stackBytes == 4096 && priority == 1);
    if (task_mock::failCreate) return pdFALSE;
    auto task = std::make_unique<task_mock::Task>();
    *handle = task.get();
    task->thread = std::thread([entry, argument, self = task.get()] {
        task_mock::current = self;
        try { entry(argument); } catch (const task_mock::Exit&) {}
    });
    task_mock::tasks.push_back(std::move(task));
    return pdPASS;
}
inline void xTaskNotifyGive(TaskHandle_t task) {
    std::lock_guard<std::mutex> guard(task->mutex);
    ++task->notifications;
    task->changed.notify_one();
}
inline BaseType_t xTaskCreatePinnedToCore(void(*entry)(void*), const char* name, uint32_t stackBytes,
    void* argument, UBaseType_t priority, TaskHandle_t* handle, BaseType_t core) {
    assert(core == 1);
    return xTaskCreate(entry, name, stackBytes, argument, priority, handle);
}
inline uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks) {
    auto& task = *task_mock::current;
    std::unique_lock<std::mutex> guard(task.mutex);
    task.changed.wait_for(guard, std::chrono::milliseconds(ticks), [&] { return task.notifications != 0; });
    const auto count = task.notifications;
    if (clear) task.notifications = 0;
    else if (count) --task.notifications;
    return count;
}
[[noreturn]] inline void vTaskDelete(TaskHandle_t self) {
    assert(self == nullptr);
    throw task_mock::Exit{};
}
