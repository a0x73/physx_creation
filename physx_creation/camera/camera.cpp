#include "camera.h"

#include "../memory/memory.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
constexpr std::uint64_t all_cameras_offset = 0x19F3080;
constexpr std::uint64_t camera_game_object_offset = 0x58;
constexpr std::uint64_t game_object_name_offset = 0x88;
constexpr std::uint64_t game_object_internal_link_offset = 0x58;
constexpr std::uint64_t matrix_object_link_offset = 0x18;
constexpr std::uint64_t view_matrix_offset = 0x128;

struct matrix4x4 { float values[4][4]{}; };
struct camera_array
{
    std::uint64_t cameras{};
    std::uint64_t minimum_count{};
    std::uint64_t current_count{};
    std::uint64_t maximum_count{};
};

bool extract_position(const matrix4x4& view, vec3& position)
{
    const float a00 = view.values[0][0], a01 = view.values[1][0], a02 = view.values[2][0];
    const float a10 = view.values[0][1], a11 = view.values[1][1], a12 = view.values[2][1];
    const float a20 = view.values[0][3], a21 = view.values[1][3], a22 = view.values[2][3];
    const float bx = view.values[3][0], by = view.values[3][1], bz = view.values[3][3];
    const float determinant =
        a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
    if (!std::isfinite(determinant) || std::fabs(determinant) < 1e-6f)
    {
        return false;
    }
    const float inverse = 1.0f / determinant;
    position = {-((a11 * a22 - a12 * a21) * bx + (a02 * a21 - a01 * a22) * by +
                  (a01 * a12 - a02 * a11) * bz) * inverse,
                -((a12 * a20 - a10 * a22) * bx + (a00 * a22 - a02 * a20) * by +
                  (a02 * a10 - a00 * a12) * bz) * inverse,
                -((a10 * a21 - a11 * a20) * bx + (a01 * a20 - a00 * a21) * by +
                  (a00 * a11 - a01 * a10) * bz) * inverse};
    return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}

vec3 extract_forward(const float view_projection[4][4])
{
    // Derive the center ray from the view-projection frustum planes. This is
    // the same convention used by the previous working raycast implementation.
    const vec3 x_plane{view_projection[0][0], view_projection[0][1], view_projection[0][2]};
    const vec3 y_plane{view_projection[1][0], view_projection[1][1], view_projection[1][2]};
    vec3 result{x_plane.y * y_plane.z - x_plane.z * y_plane.y,
                x_plane.z * y_plane.x - x_plane.x * y_plane.z,
                x_plane.x * y_plane.y - x_plane.y * y_plane.x};
    const vec3 w_plane{view_projection[3][0], view_projection[3][1], view_projection[3][2]};
    if (w_plane.x * result.x + w_plane.y * result.y + w_plane.z * result.z < 0.0f)
        result = result * -1.0f;
    const float length = std::sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
    if (!std::isfinite(length) || length < 1e-5f)
    {
        return {0.0f, 0.0f, 1.0f};
    }
    return result * (1.0f / length);
}
}

bool camera_state::world_to_screen(const vec3& world, const float width, const float height, float& x, float& y) const
{
    if (!projection_valid || width <= 0.0f || height <= 0.0f)
    {
        return false;
    }
    const float clip_x = view_projection[0][0] * world.x + view_projection[0][1] * world.y +
                         view_projection[0][2] * world.z + view_projection[0][3];
    const float clip_y = view_projection[1][0] * world.x + view_projection[1][1] * world.y +
                         view_projection[1][2] * world.z + view_projection[1][3];
    const float clip_w = view_projection[3][0] * world.x + view_projection[3][1] * world.y +
                         view_projection[3][2] * world.z + view_projection[3][3];
    if (!std::isfinite(clip_x) || !std::isfinite(clip_y) || !std::isfinite(clip_w) || clip_w <= 0.01f)
    {
        return false;
    }
    x = width * 0.5f * (1.0f + clip_x / clip_w);
    y = height * 0.5f * (1.0f - clip_y / clip_w);
    return std::isfinite(x) && std::isfinite(y);
}

camera_state read_camera_once(void* scatter_handle)
{
    camera_state result{};
    const auto scatter = static_cast<VMMDLL_SCATTER_HANDLE>(scatter_handle);
    const std::uint64_t unity = mem.get_base("UnityPlayer.dll");
    if (!scatter || !unity || unity > std::numeric_limits<std::uint64_t>::max() - all_cameras_offset)
    {
        return result;
    }

    std::uint64_t array_address{};
    if (!mem.add_scatter_read(scatter, unity + all_cameras_offset, &array_address, sizeof(array_address)) ||
        !mem.execute_read_scatter(scatter) || !array_address)
    {
        return result;
    }
    camera_array array{};
    if (!mem.add_scatter_read(scatter, array_address, &array, sizeof(array)) || !mem.execute_read_scatter(scatter) ||
        !array.cameras || !array.current_count || array.current_count > 512)
    {
        return result;
    }

    std::vector<std::uint64_t> cameras(array.current_count);
    if (!mem.add_scatter_read(scatter, array.cameras, cameras.data(), cameras.size() * sizeof(cameras.front())) ||
        !mem.execute_read_scatter(scatter))
    {
        return result;
    }
    std::vector<std::uint64_t> game_objects(cameras.size());
    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        if (cameras[index]) mem.add_scatter_read(scatter, cameras[index] + camera_game_object_offset,
                                                 &game_objects[index], sizeof(game_objects[index]));
    }
    if (!mem.execute_read_scatter(scatter))
    {
        return result;
    }

    std::vector<std::uint64_t> names(cameras.size());
    std::vector<std::uint64_t> links(cameras.size());
    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        if (!game_objects[index])
        {
            continue;
        }
        mem.add_scatter_read(scatter, game_objects[index] + game_object_name_offset, &names[index], sizeof(names[index]));
        mem.add_scatter_read(scatter, game_objects[index] + game_object_internal_link_offset, &links[index], sizeof(links[index]));
    }
    if (!mem.execute_read_scatter(scatter))
    {
        return result;
    }

    struct fixed_name { char value[sizeof("FPS Camera")]{}; };
    std::vector<fixed_name> camera_names(cameras.size());
    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        if (names[index]) mem.add_scatter_read(scatter, names[index], &camera_names[index], sizeof(camera_names[index]));
    }
    if (!mem.execute_read_scatter(scatter))
    {
        return result;
    }

    std::vector<std::uint64_t> matrices(cameras.size());
    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        if (links[index] && std::memcmp(camera_names[index].value, "FPS Camera", sizeof("FPS Camera")) == 0)
        {
            mem.add_scatter_read(scatter, links[index] + matrix_object_link_offset, &matrices[index], sizeof(matrices[index]));
        }
    }
    if (!mem.execute_read_scatter(scatter))
    {
        return result;
    }

    std::vector<matrix4x4> views(cameras.size());
    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        if (matrices[index]) mem.add_scatter_read(scatter, matrices[index] + view_matrix_offset, &views[index], sizeof(views[index]));
    }
    if (!mem.execute_read_scatter(scatter))
    {
        return result;
    }

    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        if (!matrices[index])
        {
            continue;
        }
        result.camera_address = cameras[index];
        result.matrix_address = matrices[index];
        float magnitude = 0.0f;
        result.projection_valid = true;
        for (std::size_t row = 0; row < 4; ++row)
        {
            for (std::size_t column = 0; column < 4; ++column)
            {
                const float value = views[index].values[column][row];
                result.view_projection[row][column] = value;
                result.projection_valid = result.projection_valid && std::isfinite(value);
                magnitude += std::fabs(value);
            }
        }
        result.projection_valid = result.projection_valid && magnitude > 1e-4f;
        result.position_valid = extract_position(views[index], result.position);
        result.forward = extract_forward(result.view_projection);
        if (result.projection_valid)
        {
            return result;
        }
    }
    return {};
}
