#include "camera/camera.h"
#include "memory/memory.h"
#include "physx/physx.h"
#include "renderer/renderer.h"
#include "renderer/window.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

int main(int argc, char** argv)
{
    if (!mem.load("EscapeFromTarkov.exe", true))
    {
        return 1;
    }

    std::atomic<std::shared_ptr<const camera_state>> camera_snapshot{std::make_shared<camera_state>()};
    std::atomic<std::shared_ptr<const physx::render_snapshot>> physics_snapshot{std::make_shared<physx::render_snapshot>()};

    std::jthread data_thread([&](const std::stop_token stop)
    {
        physx physics{};
        auto camera_scatter = mem.create_scatter_handle();
        auto snapshot = physics.get_render_snapshot();
        if (!snapshot->triangles.empty())
        {
            physics_snapshot.store(std::move(snapshot), std::memory_order_release);
        }

        constexpr auto update_interval = std::chrono::milliseconds(8);
        auto next_update = std::chrono::steady_clock::now();
        bool had_valid_camera = false;
        while (!stop.stop_requested())
        {
            if (camera_scatter)
            {
                auto camera = std::make_shared<camera_state>(read_camera_once(camera_scatter));
                camera_snapshot.store(std::move(camera), std::memory_order_release);
            }
            const auto current_camera = camera_snapshot.load(std::memory_order_acquire);
            const bool camera_valid = current_camera->projection_valid && current_camera->position_valid;
            if (camera_valid != had_valid_camera)
            {
                physics.request_regrab();
            }
            had_valid_camera = camera_valid;
            if (physics.tick(current_camera->position, camera_valid))
            {
                snapshot = physics.get_render_snapshot();
                physics_snapshot.store(snapshot, std::memory_order_release);
            }
            else if (!camera_valid)
            {
                physics_snapshot.store(physics.get_render_snapshot(), std::memory_order_release);
            }

            next_update += update_interval;
            if (next_update < std::chrono::steady_clock::now())
            {
                next_update = std::chrono::steady_clock::now();
            }
            std::this_thread::sleep_until(next_update);
        }
        if (camera_scatter)
        {
            mem.close_scatter_handle(camera_scatter);
        }
    });

    render_window window{};
    renderer graphics(window.handle());
    while (window.process_messages())
    {
        const auto camera = camera_snapshot.load(std::memory_order_acquire);
        const auto physics = physics_snapshot.load(std::memory_order_acquire);
        graphics.render_frame(*camera, physics);
    }

    data_thread.request_stop();
    return 0;
}
