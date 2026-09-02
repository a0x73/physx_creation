#pragma once

#include <cstdint>

#include "../physx/physx_types.h"

struct camera_state
{
    float view_projection[4][4]{};
    vec3 position{};
    vec3 forward{0.0f, 0.0f, 1.0f};
    std::uint64_t camera_address{};
    std::uint64_t matrix_address{};
    bool projection_valid{};
    bool position_valid{};

    bool world_to_screen(const vec3& world, float width, float height, float& x, float& y) const;
};

camera_state read_camera_once(void* scatter_handle);
