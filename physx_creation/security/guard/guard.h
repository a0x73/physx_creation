#pragma once
#include <atomic>
#include <Windows.h>

class guard
{
public:

    guard(std::atomic<bool>& application_running);
    ~guard();

private:

    std::atomic<bool>& application_running;

    static DWORD WINAPI worker(void* self);

    HANDLE thread{};
    DWORD thread_id{};
    std::atomic<bool> active{};
};
