#include "physx.h"

#include "../dependencies/logging/logging.h"
#include "../memory/memory.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <limits>
#include <new>
#include <numeric>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace
{
constexpr std::uint32_t max_scenes = 64;
constexpr std::uint32_t max_scene_actors = 10000000;
constexpr std::size_t max_total_actors = 40000000;
constexpr std::uint32_t max_actor_shapes = 655350;
constexpr std::uint32_t max_mesh_elements = 40000000;
constexpr std::size_t max_heightfield_samples = 40000000;
constexpr std::size_t max_payload_batch_bytes = 64ull * 1024ull * 1024ull;
constexpr std::size_t max_payload_batch_requests = 256;
// VMM scatter requests are page-backed, so queue more independent requests per
// execution instead of inflating unrelated reads into one fragile range.
constexpr std::size_t max_discovery_batch_requests = 1024;
constexpr float spatial_cell_size = 10.0f;
constexpr std::uint64_t spatial_coordinate_mask = (1ull << 21u) - 1u;
constexpr std::uint16_t rigid_dynamic_type = 5;
constexpr std::uint16_t rigid_static_type = 6;
constexpr float pi = 3.14159265358979323846f;

struct array_t
{
    std::uint64_t start{};
    std::uint32_t size{};
    std::uint32_t capacity{};
};

struct np_physics_t
{
    std::byte padding[0x8]{};
    array_t scene_array{};
};

struct scene_t
{
    std::byte padding[0x23C8]{};
    array_t rigid_actors{};
};

struct ptr_table_t
{
    std::uint64_t data{};
    std::uint16_t count{};
    bool owns_memory{};
    bool buffer_used{};
    std::byte padding[0x4]{};
};

struct rigid_actor_t
{
    std::byte padding_0[0x8]{};
    std::uint16_t type{};
    std::byte padding_1[0x1E]{};
    ptr_table_t shapes{};
    std::byte padding_2[0x28]{};
};

static_assert(sizeof(array_t) == 0x10);
static_assert(offsetof(np_physics_t, scene_array) == 0x8);
static_assert(offsetof(scene_t, rigid_actors) == 0x23C8);
static_assert(sizeof(ptr_table_t) == 0x10);
static_assert(offsetof(rigid_actor_t, shapes) == 0x28);
static_assert(sizeof(rigid_actor_t) == 0x60);

struct scatter_request
{
    std::uint64_t address{};
    void* destination{};
    std::size_t size{};
    DWORD bytes_read{};
};

std::uint64_t fingerprint(const std::vector<std::uint64_t>& addresses)
{
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto address : addresses)
    {
        hash ^= address;
        hash *= 1099511628211ull;
    }
    return hash;
}

void log_request_summary(const std::uint64_t diagnostic_id, const char* stage,
                         const std::vector<scatter_request>& requests)
{
    std::size_t exact = 0;
    std::size_t partial = 0;
    std::size_t failed = 0;
    std::uint64_t requested_bytes = 0;
    std::uint64_t returned_bytes = 0;
    for (const auto& request : requests)
    {
        requested_bytes += request.size;
        returned_bytes += request.bytes_read;
        if (request.bytes_read == request.size)
        {
            ++exact;
        }
        else if (request.bytes_read == 0)
        {
            ++failed;
        }
        else
        {
            ++partial;
        }
    }

    LOG_INFO("[physx:%llu] read stage=%s requests=%zu exact=%zu partial=%zu failed=%zu requested_bytes=%llu returned_bytes=%llu",
             diagnostic_id, stage, requests.size(), exact, partial, failed,
             static_cast<unsigned long long>(requested_bytes), static_cast<unsigned long long>(returned_bytes));
    std::size_t samples = 0;
    for (std::size_t index = 0; index < requests.size() && samples < 12; ++index)
    {
        const auto& request = requests[index];
        if (request.bytes_read != request.size)
        {
            LOG_WARN("[physx:%llu] read failure stage=%s index=%zu address=0x%llx bytes=%lu expected=%zu",
                     diagnostic_id, stage, index, request.address, request.bytes_read, request.size);
            ++samples;
        }
    }
}

bool execute_batched_reads(VMMDLL_SCATTER_HANDLE scatter, std::vector<scatter_request>& requests,
                           const char* stage, const bool allow_partial = false)
{
    std::size_t partial_reads = 0;
    for (std::size_t first = 0; first < requests.size(); first += max_discovery_batch_requests)
    {
        const std::size_t last = std::min(first + max_discovery_batch_requests, requests.size());
        for (std::size_t index = first; index < last; ++index)
        {
            auto& request = requests[index];
            request.bytes_read = 0;
            if (!mem.add_scatter_read(scatter, request.address, request.destination, request.size,
                                      &request.bytes_read))
            {
                LOG_WARN("[physx] %s prepare failed request=%zu/%zu address=0x%llx size=%zu", stage, index,
                         requests.size(), request.address, request.size);
                return false;
            }
        }
        if (!mem.execute_read_scatter(scatter))
        {
            LOG_WARN("[physx] %s execute failed batch=%zu-%zu", stage, first, last);
            return false;
        }
        for (std::size_t index = first; index < last; ++index)
        {
            if (requests[index].bytes_read != requests[index].size)
            {
                if (!allow_partial)
                {
                    LOG_WARN("[physx] %s partial read request=%zu/%zu bytes=%u/%zu address=0x%llx", stage, index,
                             requests.size(), requests[index].bytes_read, requests[index].size,
                             requests[index].address);
                    return false;
                }
                std::memset(requests[index].destination, 0, requests[index].size);
                ++partial_reads;
            }
        }
    }
    if (partial_reads)
    {
        LOG_WARN("[physx] %s skipped partial reads=%zu/%zu", stage, partial_reads, requests.size());
    }
    return true;
}

std::size_t recover_partial_reads(std::vector<scatter_request>& requests, const char* stage)
{
    std::size_t recovered = 0;
    std::size_t failed = 0;
    for (auto& request : requests)
    {
        if (request.bytes_read == request.size)
        {
            continue;
        }
        if (mem.read(request.address, request.destination, request.size))
        {
            request.bytes_read = static_cast<DWORD>(request.size);
            ++recovered;
        }
        else
        {
            std::memset(request.destination, 0, request.size);
            ++failed;
        }
    }
    if (recovered || failed)
    {
        LOG_INFO("[physx] %s fallback recovered=%zu failed=%zu", stage, recovered, failed);
    }
    return recovered;
}

bool read_metadata(VMMDLL_SCATTER_HANDLE scatter, const std::uint64_t address, void* destination,
                   const std::size_t size, const char* stage)
{
    std::memset(destination, 0, size);
    DWORD bytes_read = 0;
    if (mem.add_scatter_read(scatter, address, destination, size, &bytes_read) &&
        mem.execute_read_scatter(scatter) && bytes_read == size)
    {
        return true;
    }

    std::memset(destination, 0, size);
    if (mem.read(address, destination, size))
    {
        LOG_INFO("[physx] %s recovered through multi-mode read address=0x%llx size=%zu", stage, address, size);
        return true;
    }

    LOG_WARN("[physx] %s unreadable in scatter and fallback modes address=0x%llx scatter_bytes=%u/%zu", stage,
             address, bytes_read, size);
    return false;
}

bool valid_array(const array_t& array, const std::uint32_t maximum)
{
    return array.size <= maximum && array.capacity >= array.size && array.capacity <= maximum &&
           (array.size == 0 || array.start != 0);
}

bool valid_address(const std::uint64_t address)
{
    return address >= 0x10000 && address < 0x0000800000000000 && (address & 0x7) == 0;
}

bool finite_positive(const float value)
{
    return std::isfinite(value) && value > 0.0f && value < 100000.0f;
}

template <typename type>
type geometry_as(const physx_layout::shape& shape)
{
    type result{};
    static_assert(sizeof(result) <= sizeof(shape.core.geometry));
    std::memcpy(&result, shape.core.geometry.bytes, sizeof(result));
    return result;
}

transform normalized_pose(transform pose)
{
    if (pose.rotation.sane())
    {
        pose.rotation.normalize();
    }
    return pose;
}

void add_triangle(std::vector<triangle>& output, const transform& pose, const vec3& a, const vec3& b, const vec3& c,
                  const px_geometry_type type, const std::uint64_t actor_address = 0)
{
    output.push_back({pose.point(a), pose.point(b), pose.point(c), type, actor_address});
}

float ray_triangle(const vec3& origin, const vec3& direction, const triangle& triangle_value)
{
    const vec3 edge1 = triangle_value.v1 + triangle_value.v0 * -1.0f;
    const vec3 edge2 = triangle_value.v2 + triangle_value.v0 * -1.0f;
    const vec3 p{direction.y * edge2.z - direction.z * edge2.y,
                 direction.z * edge2.x - direction.x * edge2.z,
                 direction.x * edge2.y - direction.y * edge2.x};
    const float determinant = edge1.x * p.x + edge1.y * p.y + edge1.z * p.z;
    if (std::fabs(determinant) < 1e-6f)
    {
        return -1.0f;
    }
    const float inverse = 1.0f / determinant;
    const vec3 offset = origin + triangle_value.v0 * -1.0f;
    const float u = (offset.x * p.x + offset.y * p.y + offset.z * p.z) * inverse;
    if (u < 0.0f || u > 1.0f)
    {
        return -1.0f;
    }
    const vec3 q{offset.y * edge1.z - offset.z * edge1.y,
                 offset.z * edge1.x - offset.x * edge1.z,
                 offset.x * edge1.y - offset.y * edge1.x};
    const float v = (direction.x * q.x + direction.y * q.y + direction.z * q.z) * inverse;
    if (v < 0.0f || u + v > 1.0f)
    {
        return -1.0f;
    }
    const float distance = (edge2.x * q.x + edge2.y * q.y + edge2.z * q.z) * inverse;
    return distance >= 0.0f && std::isfinite(distance) ? distance : -1.0f;
}

std::uint64_t spatial_key(const int x, const int y, const int z)
{
    return (static_cast<std::uint64_t>(x) & spatial_coordinate_mask) |
           ((static_cast<std::uint64_t>(y) & spatial_coordinate_mask) << 21u) |
           ((static_cast<std::uint64_t>(z) & spatial_coordinate_mask) << 42u);
}

int spatial_coordinate(const float value)
{
    return static_cast<int>(std::floor(value / spatial_cell_size));
}
}

physx::physx() : physx(config{})
{
}

physx::physx(const config configuration) : m_config(configuration)
{
    m_scatter_handle = mem.create_scatter_handle();
    if (!m_scatter_handle)
    {
        return;
    }

    refresh();
}

void physx::reset_object_data()
{
    if (m_scatter_handle)
    {
        mem.close_scatter_handle(static_cast<VMMDLL_SCATTER_HANDLE>(m_scatter_handle));
        m_scatter_handle = nullptr;
    }
    m_cached_actors.clear();
    m_cached_actor_shapes.clear();
    m_dynamic_triangle_ranges.clear();
    m_dynamic_actor_active.clear();
    m_render_snapshot = std::make_shared<render_snapshot>();
    m_topology_generation = 0;
}

bool physx::regrab_object_data()
{
    reset_object_data();
    m_scatter_handle = mem.create_scatter_handle();
    if (!m_scatter_handle)
    {
        return false;
    }
    return refresh();
}

void physx::request_regrab()
{
    m_regrab_requested.store(true, std::memory_order_release);
}

bool physx::tick(const vec3& query_origin, const bool query_origin_valid)
{
    if (m_regrab_requested.exchange(false, std::memory_order_acq_rel))
    {
        return regrab_object_data();
    }
    return refresh_dynamic(query_origin, query_origin_valid);
}

bool physx::refresh()
{
    using namespace std::chrono_literals;
    const auto refresh_started = std::chrono::steady_clock::now();
    m_diagnostic_id = ++m_refresh_sequence;
    LOG_INFO("[physx:%llu] refresh begin cached_actors=%zu cached_actor_groups=%zu", m_diagnostic_id,
             m_cached_actors.size(), m_cached_actor_shapes.size());
    const std::size_t previous_actor_count = m_cached_actors.size();
    std::size_t previous_shape_count = 0;
    for (const auto& actor_shapes : m_cached_actor_shapes)
    {
        previous_shape_count += actor_shapes.shapes.size();
    }

    for (std::size_t attempt = 1; attempt <= 3; ++attempt)
    {
        const auto attempt_started = std::chrono::steady_clock::now();
        LOG_INFO("[physx:%llu] refresh attempt=%zu/3 begin", m_diagnostic_id, attempt);
        if (!cache_actors(m_scatter_handle))
        {
            LOG_WARN("[physx] actor discovery failed attempt=%zu/3", attempt);
        }
        else if (!cache_actor_shapes(m_scatter_handle))
        {
            LOG_WARN("[physx] actor shape caching failed attempt=%zu/3 actors=%zu", attempt,
                     m_cached_actors.size());
        }
        else
        {
            std::size_t current_shape_count = 0;
            for (const auto& actor_shapes : m_cached_actor_shapes)
            {
                current_shape_count += actor_shapes.shapes.size();
            }
            const bool topology_changed = m_cached_actors.size() != previous_actor_count ||
                                          current_shape_count != previous_shape_count ||
                                          m_render_snapshot->triangles.empty();
            if (topology_changed && !dump_shapes(m_scatter_handle))
            {
                LOG_WARN("[physx] geometry dumping failed actors=%zu", m_cached_actors.size());
                return false;
            }
            LOG_INFO("[physx] incremental refresh new_actors=%zu new_shapes=%zu cumulative_actors=%zu cumulative_shapes=%zu rebuilt=%s",
                      static_cast<long long>(m_cached_actors.size()) - static_cast<long long>(previous_actor_count),
                      static_cast<long long>(current_shape_count) - static_cast<long long>(previous_shape_count),
                       m_cached_actors.size(), current_shape_count, topology_changed ? "yes" : "no");
            const auto attempt_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - attempt_started);
            const auto refresh_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - refresh_started);
            LOG_INFO("[physx:%llu] refresh complete attempt=%zu actors=%zu shapes=%zu triangles=%zu cells=%zu attempt_ms=%lld total_ms=%lld",
                     m_diagnostic_id, attempt, m_cached_actors.size(), current_shape_count,
                     m_render_snapshot->triangles.size(), m_render_snapshot->spatial_cells.size(),
                     static_cast<long long>(attempt_elapsed.count()), static_cast<long long>(refresh_elapsed.count()));
            return true;
        }
        LOG_WARN("[physx] unstable or incomplete actor snapshot; retrying attempt=%zu/3", attempt);
        if (attempt < 3)
        {
            mem.close_scatter_handle(static_cast<VMMDLL_SCATTER_HANDLE>(m_scatter_handle));
            m_scatter_handle = nullptr;
            const bool refreshed = mem.refresh_process_memory();
            std::this_thread::sleep_for(100ms);
            m_scatter_handle = mem.create_scatter_handle();
            LOG_INFO("[physx] refreshed process translations=%s scatter=%s", refreshed ? "yes" : "no",
                     m_scatter_handle ? "recreated" : "failed");
            if (!m_scatter_handle)
            {
                return false;
            }
        }
    }
    const auto refresh_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - refresh_started);
    LOG_ERROR("[physx:%llu] refresh failed total_ms=%lld", m_diagnostic_id,
              static_cast<long long>(refresh_elapsed.count()));
    return false;
}

bool physx::refresh_dynamic(const vec3& query_origin, const bool query_origin_valid)
{
    const auto refresh_started = std::chrono::steady_clock::now();
    const auto scatter = static_cast<VMMDLL_SCATTER_HANDLE>(m_scatter_handle);
    if (!scatter || m_dynamic_triangle_ranges.empty())
    {
        return false;
    }

    struct dynamic_pose_read
    {
        physx_layout::body_core body{};
        transform buffered_body_to_world{};
    };
    struct static_pose_read
    {
        physx_layout::rigid_core rigid{};
    };
    std::vector<dynamic_pose_read> dynamic_actors;
    std::vector<static_pose_read> static_actors;
    std::vector<scatter_request> requests;
    struct actor_read_index
    {
        bool dynamic{};
        std::size_t index{};
    };
    std::unordered_map<std::uint64_t, actor_read_index> indices;
    dynamic_actors.reserve(m_dynamic_triangle_ranges.size());
    static_actors.reserve(m_dynamic_triangle_ranges.size());
    requests.reserve(m_dynamic_triangle_ranges.size());
    indices.reserve(m_dynamic_triangle_ranges.size());
    constexpr float activate_distance = 100.0f;
    const float activate_squared = activate_distance * activate_distance;
    const float deactivate_squared = activate_squared;
    for (const auto& range : m_dynamic_triangle_ranges)
    {
        if (indices.contains(range.actor_address))
        {
            continue;
        }
        if (query_origin_valid && range.actor_pose.sane())
        {
            const float dx = range.actor_pose.position.x - query_origin.x;
            const float dy = range.actor_pose.position.y - query_origin.y;
            const float dz = range.actor_pose.position.z - query_origin.z;
            const float distance_squared = dx * dx + dy * dy + dz * dz;
            if (distance_squared > activate_squared)
            {
                continue;
            }
        }
        if (range.actor_type == rigid_dynamic_type)
        {
            indices.emplace(range.actor_address, actor_read_index{true, dynamic_actors.size()});
            dynamic_actors.emplace_back();
            requests.push_back({range.actor_address + offsetof(physx_layout::dynamic_actor, body),
                                &dynamic_actors.back(), sizeof(dynamic_actors.back())});
        }
        else
        {
            indices.emplace(range.actor_address, actor_read_index{false, static_actors.size()});
            static_actors.emplace_back();
            requests.push_back({range.actor_address + offsetof(physx_layout::static_actor, rigid),
                                &static_actors.back().rigid, sizeof(static_actors.back().rigid)});
        }
    }
    if (!execute_batched_reads(scatter, requests, "dynamic actor transforms", true))
    {
        return false;
    }

    std::unordered_map<std::uint64_t, bool> active_actors;
    active_actors.reserve(indices.size());
    std::unordered_set<std::uint64_t> changed_actors;
    changed_actors.reserve(indices.size());
    bool changed = false;
    std::vector<transform> poses((std::max)(dynamic_actors.size(), static_actors.size()));
    for (const auto& range : m_dynamic_triangle_ranges)
    {
        const auto actor = indices.find(range.actor_address);
        if (actor == indices.end())
        {
            continue;
        }
        transform pose{};
        if (actor->second.dynamic)
        {
            const auto& actor_data = dynamic_actors[actor->second.index];
            pose = actor_data.buffered_body_to_world;
            if (!pose.sane()) pose = actor_data.body.rigid.body_to_world;
            const auto body_to_actor = actor_data.body.body_to_actor;
            if (pose.sane() && body_to_actor.sane())
                pose = normalized_pose(pose) * normalized_pose(body_to_actor).inverse();
            poses[actor->second.index] = normalized_pose(pose);
        }
        else
        {
            pose = static_actors[actor->second.index].rigid.body_to_world;
            poses[actor->second.index] = normalized_pose(pose);
        }
        if (!active_actors.contains(range.actor_address))
        {
            bool active = m_dynamic_actor_active.contains(range.actor_address) &&
                          m_dynamic_actor_active[range.actor_address];
            if (query_origin_valid && poses[actor->second.index].sane())
            {
                const float dx = poses[actor->second.index].position.x - query_origin.x;
                const float dy = poses[actor->second.index].position.y - query_origin.y;
                const float dz = poses[actor->second.index].position.z - query_origin.z;
                const float distance_squared = dx * dx + dy * dy + dz * dz;
                active = active ? distance_squared <= deactivate_squared : distance_squared <= activate_squared;
            }
            else if (!query_origin_valid)
            {
                active = true;
            }
            active_actors.emplace(range.actor_address, active);
        }
        if (!active_actors[range.actor_address])
        {
            continue;
        }
        if (poses[actor->second.index].sane() &&
            (std::fabs(poses[actor->second.index].position.x - range.actor_pose.position.x) > 1e-4f ||
             std::fabs(poses[actor->second.index].position.y - range.actor_pose.position.y) > 1e-4f ||
             std::fabs(poses[actor->second.index].position.z - range.actor_pose.position.z) > 1e-4f ||
             std::fabs(poses[actor->second.index].rotation.x - range.actor_pose.rotation.x) > 1e-4f ||
             std::fabs(poses[actor->second.index].rotation.y - range.actor_pose.rotation.y) > 1e-4f ||
             std::fabs(poses[actor->second.index].rotation.z - range.actor_pose.rotation.z) > 1e-4f ||
             std::fabs(poses[actor->second.index].rotation.w - range.actor_pose.rotation.w) > 1e-4f))
        {
             changed = true;
             changed_actors.insert(range.actor_address);
        }
    }
    m_dynamic_actor_active = active_actors;
    if (!changed)
    {
        return true;
    }
    auto snapshot = std::make_shared<render_snapshot>(*m_render_snapshot);
    snapshot->updated_dynamic_actors.clear();
    const auto remove_triangle_from_cell = [](auto& cells, const std::uint64_t key, const std::uint32_t index)
    {
        const auto cell = cells.find(key);
        if (cell == cells.end())
        {
            return;
        }
        auto& indices = cell->second;
        indices.erase(std::remove(indices.begin(), indices.end(), index), indices.end());
        if (indices.empty()) cells.erase(cell);
    };
    for (auto& range : m_dynamic_triangle_ranges)
    {
        const auto actor = indices.find(range.actor_address);
        if (actor == indices.end())
        {
            continue;
        }

        if (!active_actors[range.actor_address] || !changed_actors.contains(range.actor_address))
        {
            continue;
        }
        const transform pose = poses[actor->second.index];
        if (!pose.sane() || !range.actor_pose.sane())
        {
            continue;
        }
        const transform delta = pose * range.actor_pose.inverse();
        if (range.first > snapshot->triangles.size() || range.count > snapshot->triangles.size() - range.first)
        {
            continue;
        }
        for (std::size_t index = range.first; index < range.first + range.count; ++index)
        {
            auto& value = snapshot->triangles[index];
            value.dynamic = true;
            const vec3 old_center = triangle_center(value);
            value.v0 = delta.point(value.v0);
            value.v1 = delta.point(value.v1);
            value.v2 = delta.point(value.v2);
            const vec3 new_center = triangle_center(value);
            if (std::isfinite(new_center.x) && std::isfinite(new_center.y) && std::isfinite(new_center.z))
            {
                const auto old_key = spatial_key(spatial_coordinate(old_center.x), spatial_coordinate(old_center.y),
                                                 spatial_coordinate(old_center.z));
                const auto new_key = spatial_key(spatial_coordinate(new_center.x), spatial_coordinate(new_center.y),
                                                 spatial_coordinate(new_center.z));
                if (old_key != new_key)
                {
                    remove_triangle_from_cell(snapshot->spatial_cells, old_key, static_cast<std::uint32_t>(index));
                    snapshot->spatial_cells[new_key].push_back(static_cast<std::uint32_t>(index));
                }
            }
        }
        if (std::find(snapshot->updated_dynamic_actors.begin(), snapshot->updated_dynamic_actors.end(),
                      range.actor_address) == snapshot->updated_dynamic_actors.end())
            snapshot->updated_dynamic_actors.push_back(range.actor_address);
    }

    for (const auto actor_address : changed_actors)
    {
        const auto actor = indices.find(actor_address);
        if (actor != indices.end() && poses[actor->second.index].sane())
        {
            for (auto& range : m_dynamic_triangle_ranges)
                if (range.actor_address == actor_address) range.actor_pose = poses[actor->second.index];
        }
    }

    m_render_snapshot = std::move(snapshot);
    static std::uint32_t diagnostic_ticks = 0;
    if ((++diagnostic_ticks % 100u) == 0u)
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - refresh_started);
        LOG_INFO("[physx:%llu] dynamic update actors=%zu changed=%zu elapsed_ms=%lld", m_diagnostic_id,
                 indices.size(), changed_actors.size(), static_cast<long long>(elapsed.count()));
    }
    return true;
}

bool physx::render_snapshot::raycast(const vec3& origin, const vec3& direction, const float max_distance, vec3& hit) const
{
    const float length = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    if (!std::isfinite(length) || length < 1e-5f || max_distance <= 0.0f)
    {
        return false;
    }
    const vec3 ray = direction * (1.0f / length);
    float nearest = max_distance;
    bool found = false;
    for (const auto& value : triangles)
    {
        const float distance = ray_triangle(origin, ray, value);
        if (distance >= 0.0f && distance < nearest)
        {
            nearest = distance;
            found = true;
        }
    }
    if (found) hit = origin + ray * nearest;
    return found;
}

bool physx::raycast(const vec3& origin, const vec3& direction, const float max_distance, vec3& hit) const
{
    return m_render_snapshot->raycast(origin, direction, max_distance, hit);
}

physx::~physx()
{
    if (m_scatter_handle)
    {
        mem.close_scatter_handle(static_cast<VMMDLL_SCATTER_HANDLE>(m_scatter_handle));
    }
}

bool physx::cache_actors(void* scatter_handle)
{
    const auto started = std::chrono::steady_clock::now();
    const auto scatter = static_cast<VMMDLL_SCATTER_HANDLE>(scatter_handle);
    const std::uint64_t unity_player = mem.get_base("UnityPlayer.dll");
    if (!scatter || !unity_player)
    {
        LOG_WARN("[physx] actor discovery missing scatter or UnityPlayer base");
        return false;
    }

    const std::uint64_t sdk_slot = unity_player + sdk_offset;
    LOG_INFO("[physx:%llu] discovery begin unity=0x%llx sdk_slot=0x%llx sdk_offset=0x%llx actor_array_offset=0x%llx",
             m_diagnostic_id, unity_player, sdk_slot, sdk_offset, offsetof(scene_t, rigid_actors));
    std::uint64_t sdk{};
    if (!read_metadata(scatter, sdk_slot, &sdk, sizeof(sdk), "PhysX SDK pointer") || !valid_address(sdk))
    {
        LOG_WARN("[physx] invalid PhysX SDK pointer slot=0x%llx value=0x%llx", sdk_slot, sdk);
        return false;
    }

    np_physics_t physics{};
    if (!read_metadata(scatter, sdk, &physics, sizeof(physics), "PhysX SDK header"))
    {
        return false;
    }
    LOG_INFO("[physx:%llu] sdk=0x%llx scene_array start=0x%llx size=%u capacity=%u valid=%s", m_diagnostic_id,
             sdk, physics.scene_array.start, physics.scene_array.size, physics.scene_array.capacity,
             valid_array(physics.scene_array, max_scenes) ? "yes" : "no");
    if (!valid_array(physics.scene_array, max_scenes))
    {
        LOG_ERROR("[physx] invalid scene array start=0x%llx size=%u capacity=%u", physics.scene_array.start,
                  physics.scene_array.size, physics.scene_array.capacity);
        return false;
    }

    if (!physics.scene_array.size)
    {
        m_cached_actors.clear();
        return true;
    }

    std::vector<std::uint64_t> scene_addresses(physics.scene_array.size);
    std::vector<scatter_request> scene_pointer_requests;
    scene_pointer_requests.reserve(scene_addresses.size());
    for (std::size_t index = 0; index < scene_addresses.size(); ++index)
    {
        scene_pointer_requests.push_back({physics.scene_array.start + index * sizeof(scene_addresses.front()),
                                          &scene_addresses[index], sizeof(scene_addresses[index])});
    }
    if (!execute_batched_reads(scatter, scene_pointer_requests, "scene pointer slots", true))
    {
        return false;
    }
    log_request_summary(m_diagnostic_id, "scene_pointer_slots", scene_pointer_requests);
    std::size_t scatter_readable_scene_slots = 0;
    std::size_t fallback_readable_scene_slots = 0;
    for (std::size_t index = 0; index < scene_pointer_requests.size(); ++index)
    {
        if (scene_pointer_requests[index].bytes_read == sizeof(std::uint64_t))
        {
            ++scatter_readable_scene_slots;
            continue;
        }

        const std::uint64_t slot_address = physics.scene_array.start + index * sizeof(scene_addresses[index]);
        if (mem.read(slot_address, &scene_addresses[index], sizeof(scene_addresses[index])))
        {
            ++fallback_readable_scene_slots;
            LOG_INFO("[physx] scene pointer fallback slot=%zu address=0x%llx value=0x%llx", index, slot_address,
                     scene_addresses[index]);
        }
        else
        {
            LOG_WARN("[physx] scene pointer unreadable in all modes slot=%zu address=0x%llx", index, slot_address);
        }
    }

    np_physics_t verified_physics{};
    if (!mem.read(sdk, &verified_physics, sizeof(verified_physics)) ||
        verified_physics.scene_array.start != physics.scene_array.start ||
        verified_physics.scene_array.size != physics.scene_array.size)
    {
        LOG_WARN("[physx] scene array changed during pointer reads old_start=0x%llx old_size=%u new_start=0x%llx new_size=%u",
                 physics.scene_array.start, physics.scene_array.size, verified_physics.scene_array.start,
                 verified_physics.scene_array.size);
        return false;
    }

    const std::size_t readable_scene_slots = scatter_readable_scene_slots + fallback_readable_scene_slots;
    const std::size_t valid_scene_slots = static_cast<std::size_t>(
        std::count_if(scene_addresses.begin(), scene_addresses.end(), valid_address));
    if (!valid_scene_slots)
    {
        LOG_WARN("[physx] scene pointer array has no valid entries readable=%zu/%zu start=0x%llx",
                 readable_scene_slots, scene_addresses.size(), physics.scene_array.start);
        return false;
    }
    LOG_INFO("[physx] scene pointers readable=%zu/%zu scatter=%zu fallback=%zu valid=%zu", readable_scene_slots,
              scene_addresses.size(), scatter_readable_scene_slots, fallback_readable_scene_slots, valid_scene_slots);
    for (std::size_t index = 0; index < scene_addresses.size(); ++index)
    {
        LOG_INFO("[physx:%llu] scene[%zu] pointer=0x%llx valid=%s actor_header=0x%llx", m_diagnostic_id, index,
                 scene_addresses[index], valid_address(scene_addresses[index]) ? "yes" : "no",
                 scene_addresses[index] ? scene_addresses[index] + offsetof(scene_t, rigid_actors) : 0);
    }

    std::vector<array_t> actor_arrays(scene_addresses.size());
    std::vector<bool> usable_scenes(scene_addresses.size());
    std::vector<std::size_t> actor_array_scene_indices;
    std::vector<scatter_request> actor_array_requests;
    for (std::size_t index = 0; index < scene_addresses.size(); ++index)
    {
        if (!valid_address(scene_addresses[index]))
        {
            continue;
        }

        actor_array_requests.push_back({scene_addresses[index] + offsetof(scene_t, rigid_actors), &actor_arrays[index],
                                        sizeof(actor_arrays[index])});
        actor_array_scene_indices.push_back(index);
    }

    if (!execute_batched_reads(scatter, actor_array_requests, "actor array headers", true))
    {
        return false;
    }
    log_request_summary(m_diagnostic_id, "actor_array_headers", actor_array_requests);
    for (std::size_t request = 0; request < actor_array_requests.size(); ++request)
    {
        const std::size_t scene_index = actor_array_scene_indices[request];
        if (actor_array_requests[request].bytes_read == sizeof(array_t))
        {
            usable_scenes[scene_index] = true;
            continue;
        }

        const std::uint64_t header_address = scene_addresses[scene_index] + offsetof(scene_t, rigid_actors);
        if (mem.read(header_address, &actor_arrays[scene_index], sizeof(actor_arrays[scene_index])))
        {
            usable_scenes[scene_index] = true;
            LOG_INFO("[physx] actor array header fallback scene=%zu address=0x%llx", scene_index, header_address);
        }
        else
        {
            LOG_WARN("[physx] actor array header unreadable in all modes scene=%zu address=0x%llx", scene_index,
                     header_address);
        }
    }

    struct actor_range
    {
        std::size_t first{};
        std::size_t count{};
    };

    std::vector<actor_range> actor_ranges(actor_arrays.size());
    std::size_t total_actor_count = 0;
    for (std::size_t index = 0; index < actor_arrays.size(); ++index)
    {
        if (!usable_scenes[index])
        {
            continue;
        }

        const auto& actors = actor_arrays[index];
        LOG_INFO("[physx:%llu] scene[%zu] actor_array readable=%s start=0x%llx size=%u capacity=%u valid=%s",
                 m_diagnostic_id, index, usable_scenes[index] ? "yes" : "no", actors.start, actors.size,
                 actors.capacity, valid_array(actors, max_scene_actors) ? "yes" : "no");
        if (!valid_array(actors, max_scene_actors) || actors.size > max_total_actors - total_actor_count)
        {
            LOG_WARN("[physx] skipping invalid scene actor array scene=%zu start=0x%llx size=%u capacity=%u", index,
                     actors.start, actors.size, actors.capacity);
            usable_scenes[index] = false;
            continue;
        }

        actor_ranges[index] = {total_actor_count, actors.size};
        total_actor_count += actors.size;
    }

    std::vector<std::uint64_t> actor_addresses(total_actor_count);
    std::vector<scatter_request> actor_address_requests;
    for (std::size_t index = 0; index < actor_arrays.size(); ++index)
    {
        const auto range = actor_ranges[index];
        if (!range.count)
        {
            continue;
        }

        std::size_t slot = 0;
        while (slot < range.count)
        {
            const std::uint64_t address = actor_arrays[index].start + slot * sizeof(actor_addresses.front());
            const std::size_t bytes_until_page = 0x1000u - static_cast<std::size_t>(address & 0xfffu);
            const std::size_t page_slots = std::max<std::size_t>(1u, bytes_until_page / sizeof(actor_addresses.front()));
            const std::size_t slots = std::min(page_slots, range.count - slot);
            actor_address_requests.push_back({address, actor_addresses.data() + range.first + slot,
                                              slots * sizeof(actor_addresses.front())});
            slot += slots;
        }
    }

    if (!execute_batched_reads(scatter, actor_address_requests, "actor pointer pages", true))
    {
        return false;
    }
    log_request_summary(m_diagnostic_id, "actor_pointer_pages_scatter", actor_address_requests);
    recover_partial_reads(actor_address_requests, "actor pointer pages");
    log_request_summary(m_diagnostic_id, "actor_pointer_pages_after_fallback", actor_address_requests);
    const std::size_t readable_actor_pages = static_cast<std::size_t>(std::count_if(
        actor_address_requests.begin(), actor_address_requests.end(), [](const scatter_request& request)
        {
            return request.bytes_read == request.size;
        }));
    // A heavily incomplete pointer array is not a valid scene snapshot. Fail
    // this attempt so refresh() rebuilds VMM's process, TLB, and memory caches.
    if (!actor_address_requests.empty() && readable_actor_pages * 10 < actor_address_requests.size() * 9)
    {
        LOG_WARN("[physx:%llu] actor pointer page coverage too low readable=%zu/%zu; refreshing translations",
                 m_diagnostic_id, readable_actor_pages, actor_address_requests.size());
        return false;
    }

    std::vector<array_t> verified_actor_arrays(actor_arrays.size());
    std::vector<scatter_request> verification_requests;
    for (std::size_t index = 0; index < scene_addresses.size(); ++index)
    {
        if (usable_scenes[index])
        {
            verification_requests.push_back({scene_addresses[index] + offsetof(scene_t, rigid_actors),
                                             &verified_actor_arrays[index], sizeof(verified_actor_arrays[index])});
        }
    }
    if (!execute_batched_reads(scatter, verification_requests, "actor array verification", true))
    {
        return false;
    }
    log_request_summary(m_diagnostic_id, "actor_array_verification", verification_requests);
    std::size_t verification_request = 0;
    for (std::size_t index = 0; index < scene_addresses.size(); ++index)
    {
        if (!usable_scenes[index])
        {
            continue;
        }
        if (verification_requests[verification_request].bytes_read != sizeof(array_t))
        {
            const std::uint64_t header_address = scene_addresses[index] + offsetof(scene_t, rigid_actors);
            if (!mem.read(header_address, &verified_actor_arrays[index], sizeof(verified_actor_arrays[index])))
            {
                LOG_WARN("[physx] actor array verification unreadable scene=%zu address=0x%llx", index,
                         header_address);
                return false;
            }
            LOG_INFO("[physx] actor array verification fallback scene=%zu address=0x%llx", index, header_address);
        }
        ++verification_request;
    }
    for (std::size_t index = 0; index < actor_arrays.size(); ++index)
    {
        if (!usable_scenes[index])
        {
            continue;
        }
        if (verified_actor_arrays[index].start != actor_arrays[index].start ||
            verified_actor_arrays[index].size != actor_arrays[index].size)
        {
            LOG_WARN("[physx] scene actor array changed during discovery scene=%zu old=%u new=%u", index,
                     actor_arrays[index].size, verified_actor_arrays[index].size);
            return false;
        }
    }

    actor_addresses.erase(std::remove_if(actor_addresses.begin(), actor_addresses.end(),
                                         [](const std::uint64_t address)
                                         {
                                             return !valid_address(address);
                                         }),
                          actor_addresses.end());
    std::sort(actor_addresses.begin(), actor_addresses.end());
    actor_addresses.erase(std::unique(actor_addresses.begin(), actor_addresses.end()), actor_addresses.end());
    LOG_INFO("[physx:%llu] actor addresses slots=%zu valid_unique=%zu fingerprint=0x%llx", m_diagnostic_id,
             total_actor_count, actor_addresses.size(), fingerprint(actor_addresses));

    std::vector<rigid_actor_t> actors(actor_addresses.size());
    std::vector<scatter_request> actor_requests;
    actor_requests.reserve(actor_addresses.size());
    for (std::size_t index = 0; index < actor_addresses.size(); ++index)
    {
        actor_requests.push_back({actor_addresses[index], &actors[index], sizeof(actors[index])});
    }

    if (!execute_batched_reads(scatter, actor_requests, "actor objects", true))
    {
        return false;
    }
    log_request_summary(m_diagnostic_id, "actor_objects_scatter", actor_requests);
    recover_partial_reads(actor_requests, "actor objects");
    log_request_summary(m_diagnostic_id, "actor_objects_after_fallback", actor_requests);

    std::vector<cached_actor> refreshed_actors;
    refreshed_actors.reserve(actors.size());
    std::array<std::size_t, 32> actor_type_counts{};
    std::array<std::size_t, 10> shape_count_histogram{};
    std::size_t actor_type_out_of_range = 0;
    std::size_t shape_count_overflow = 0;
    std::size_t invalid_shape_table = 0;
    for (std::size_t index = 0; index < actors.size(); ++index)
    {
        const auto& actor = actors[index];
        if (actor.type < actor_type_counts.size())
        {
            ++actor_type_counts[actor.type];
        }
        else
        {
            ++actor_type_out_of_range;
        }
        if (actor.type != rigid_dynamic_type && actor.type != rigid_static_type)
        {
            continue;
        }
        if (actor.shapes.count > max_actor_shapes || (actor.shapes.count > 1 && !valid_address(actor.shapes.data)))
        {
            if (actor.shapes.count > max_actor_shapes)
            {
                ++shape_count_overflow;
            }
            else
            {
                ++invalid_shape_table;
            }
            continue;
        }

        const auto bucket = (std::min<std::size_t>)(actor.shapes.count, shape_count_histogram.size() - 1u);
        ++shape_count_histogram[bucket];

        refreshed_actors.push_back({actor_addresses[index], actor.type, actor.shapes.data, actor.shapes.count});
    }
    for (std::size_t type = 0; type < actor_type_counts.size(); ++type)
    {
        if (actor_type_counts[type])
        {
            LOG_INFO("[physx:%llu] actor type=%zu count=%zu", m_diagnostic_id, type, actor_type_counts[type]);
        }
    }
    LOG_INFO("[physx:%llu] actor classification accepted=%zu type_out_of_range=%zu shape_count_overflow=%zu invalid_shape_table=%zu shape_histogram=0:%zu,1:%zu,2:%zu,3:%zu,4:%zu,5:%zu,6:%zu,7:%zu,8:%zu,9plus:%zu",
             m_diagnostic_id, refreshed_actors.size(), actor_type_out_of_range, shape_count_overflow,
             invalid_shape_table, shape_count_histogram[0], shape_count_histogram[1], shape_count_histogram[2],
             shape_count_histogram[3], shape_count_histogram[4], shape_count_histogram[5],
             shape_count_histogram[6], shape_count_histogram[7], shape_count_histogram[8],
             shape_count_histogram[9]);

    const std::size_t actors_before_merge = m_cached_actors.size();
    std::unordered_map<std::uint64_t, std::size_t> actor_indices;
    actor_indices.reserve(m_cached_actors.size() + refreshed_actors.size());
    for (std::size_t index = 0; index < m_cached_actors.size(); ++index)
    {
        actor_indices.emplace(m_cached_actors[index].address, index);
    }
    for (auto& actor : refreshed_actors)
    {
        const auto existing = actor_indices.find(actor.address);
        if (existing == actor_indices.end())
        {
            actor_indices.emplace(actor.address, m_cached_actors.size());
            m_cached_actors.push_back(std::move(actor));
        }
        else
        {
            m_cached_actors[existing->second] = std::move(actor);
        }
    }
    const std::size_t usable_scene_count = static_cast<std::size_t>(
        std::count(usable_scenes.begin(), usable_scenes.end(), true));
    if (!usable_scene_count)
    {
        LOG_WARN("[physx] no readable PhysX scenes found total=%zu", scene_addresses.size());
        return false;
    }
    LOG_INFO("[physx] cached actors=%zu discovered=%zu new=%zu slots=%zu scenes=%zu/%zu skipped=%zu",
             m_cached_actors.size(), actor_addresses.size(), m_cached_actors.size() - actors_before_merge,
              total_actor_count, usable_scene_count, scene_addresses.size(), scene_addresses.size() - usable_scene_count);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started);
    LOG_INFO("[physx:%llu] discovery complete cached=%zu discovered=%zu scenes=%zu/%zu elapsed_ms=%lld",
             m_diagnostic_id, m_cached_actors.size(), actor_addresses.size(), usable_scene_count,
             scene_addresses.size(), static_cast<long long>(elapsed.count()));
    return true;
}

bool physx::cache_actor_shapes(void* scatter_handle)
{
    const auto started = std::chrono::steady_clock::now();
    const auto scatter = static_cast<VMMDLL_SCATTER_HANDLE>(scatter_handle);
    if (!scatter)
    {
        return false;
    }

    std::unordered_map<std::uint64_t, const cached_actor_shapes*> previous_actor_shapes;
    previous_actor_shapes.reserve(m_cached_actor_shapes.size());
    for (const auto& actor_shapes : m_cached_actor_shapes)
    {
        previous_actor_shapes.emplace(actor_shapes.actor_address, &actor_shapes);
    }

    std::vector<cached_actor_shapes> refreshed_shapes(m_cached_actors.size());
    std::vector<scatter_request> shape_list_requests;
    shape_list_requests.reserve(m_cached_actors.size());
    std::size_t reused_shape_lists = 0;
    for (std::size_t index = 0; index < m_cached_actors.size(); ++index)
    {
        const auto& actor = m_cached_actors[index];
        auto& shapes = refreshed_shapes[index];
        shapes.actor_address = actor.address;
        shapes.actor_pose = {};
        shapes.shape_data = actor.shape_data;
        shapes.shape_count = actor.shape_count;
        const auto previous_actor = previous_actor_shapes.find(actor.address);
        const bool can_reuse_shape_list = previous_actor != previous_actor_shapes.end() &&
                                           previous_actor->second->shape_data == actor.shape_data &&
                                           previous_actor->second->shape_count == actor.shape_count;
        if (can_reuse_shape_list)
        {
            shapes.addresses = previous_actor->second->addresses;
            ++reused_shape_lists;
            continue;
        }
        shapes.addresses.resize(actor.shape_count);
        if (!actor.shape_count)
        {
            continue;
        }
        if (actor.shape_count == 1)
        {
            shapes.addresses[0] = actor.shape_data;
            continue;
        }

        shape_list_requests.push_back({actor.shape_data, shapes.addresses.data(),
                                       shapes.addresses.size() * sizeof(shapes.addresses.front())});
    }

    if (!execute_batched_reads(scatter, shape_list_requests, "shape pointer arrays", true))
    {
        return false;
    }
    log_request_summary(m_diagnostic_id, "shape_pointer_arrays_scatter", shape_list_requests);
    recover_partial_reads(shape_list_requests, "shape pointer arrays");
    log_request_summary(m_diagnostic_id, "shape_pointer_arrays_after_fallback", shape_list_requests);

    std::size_t shape_count = 0;
    std::vector<std::uint64_t> all_shape_addresses;
    all_shape_addresses.reserve(std::accumulate(
        m_cached_actors.begin(), m_cached_actors.end(), std::size_t{0},
        [](const std::size_t count, const cached_actor& actor) { return count + actor.shape_count; }));
    for (auto& shapes : refreshed_shapes)
    {
        shapes.addresses.erase(std::remove_if(shapes.addresses.begin(), shapes.addresses.end(),
                                              [](const std::uint64_t address)
                                              {
                                                  return !valid_address(address);
                                              }),
                               shapes.addresses.end());
        std::sort(shapes.addresses.begin(), shapes.addresses.end());
        shapes.addresses.erase(std::unique(shapes.addresses.begin(), shapes.addresses.end()), shapes.addresses.end());
        shape_count += shapes.addresses.size();
        all_shape_addresses.insert(all_shape_addresses.end(), shapes.addresses.begin(), shapes.addresses.end());
    }
    std::sort(all_shape_addresses.begin(), all_shape_addresses.end());
    LOG_INFO("[physx:%llu] shape addresses requested=%zu valid=%zu fingerprint=0x%llx", m_diagnostic_id,
             std::accumulate(m_cached_actors.begin(), m_cached_actors.end(), std::size_t{0},
                             [](const std::size_t count, const cached_actor& actor)
                             {
                                 return count + actor.shape_count;
                             }),
             shape_count, fingerprint(all_shape_addresses));

    std::vector<physx_layout::dynamic_actor> dynamic_actors(m_cached_actors.size());
    std::vector<physx_layout::static_actor> static_actors(m_cached_actors.size());
    std::vector<transform> actor_poses(m_cached_actors.size());
    std::vector<scatter_request> detail_requests;
    detail_requests.reserve(m_cached_actors.size() + shape_count);
    for (std::size_t index = 0; index < m_cached_actors.size(); ++index)
    {
        const auto& actor = m_cached_actors[index];
        void* destination = actor.type == rigid_dynamic_type ? static_cast<void*>(&dynamic_actors[index])
                                                             : static_cast<void*>(&static_actors[index]);
        const std::size_t size = actor.type == rigid_dynamic_type ? sizeof(dynamic_actors[index])
                                                                  : sizeof(static_actors[index]);
        detail_requests.push_back({actor.address, destination, size});
    }

    struct shape_read
    {
        std::uint64_t address{};
        physx_layout::shape data{};
    };
    std::vector<shape_read> shape_reads;
    shape_reads.reserve(shape_count);
    std::unordered_map<std::uint64_t, std::size_t> shape_read_indices;
    shape_read_indices.reserve(shape_count);
    for (auto& actor_shapes : refreshed_shapes)
    {
        actor_shapes.shapes.resize(actor_shapes.addresses.size());
        for (std::size_t index = 0; index < actor_shapes.addresses.size(); ++index)
        {
            auto& shape = actor_shapes.shapes[index];
            shape.address = actor_shapes.addresses[index];
            shape.actor_address = actor_shapes.actor_address;
            const auto [read, inserted] = shape_read_indices.emplace(shape.address, shape_reads.size());
            if (inserted)
            {
                shape_reads.push_back({shape.address, {}});
                detail_requests.push_back({shape.address, &shape_reads.back().data, sizeof(shape_reads.back().data)});
            }
        }
    }
    if (!execute_batched_reads(scatter, detail_requests, "actor and shape details", true))
    {
        return false;
    }
    log_request_summary(m_diagnostic_id, "actor_shape_details_scatter", detail_requests);
    recover_partial_reads(detail_requests, "actor and shape details");
    log_request_summary(m_diagnostic_id, "actor_shape_details_after_fallback", detail_requests);

    for (auto& actor_shapes : refreshed_shapes)
    {
        for (auto& shape : actor_shapes.shapes)
        {
            const auto read = shape_read_indices.find(shape.address);
            if (read != shape_read_indices.end())
            {
                shape.data = shape_reads[read->second].data;
            }
        }
    }
    LOG_INFO("[physx:%llu] shape cache reuse lists=%zu unique_reads=%zu", m_diagnostic_id, reused_shape_lists,
             shape_reads.size());

    std::size_t invalid_actor_pose = 0;
    std::size_t invalid_local_pose = 0;
    std::size_t invalid_geometry = 0;
    std::size_t invalid_world_pose = 0;
    std::array<std::size_t, static_cast<std::size_t>(px_geometry_type::count)> geometry_counts{};
    std::array<std::size_t, static_cast<std::size_t>(px_geometry_type::count)> simulation_geometry_counts{};
    for (std::size_t index = 0; index < m_cached_actors.size(); ++index)
    {
        if (m_cached_actors[index].type == rigid_dynamic_type)
        {
            auto pose = dynamic_actors[index].buffered_body_to_world;
            if (!pose.sane())
            {
                pose = dynamic_actors[index].body.rigid.body_to_world;
            }
            const auto body_to_actor = dynamic_actors[index].body.body_to_actor;
            if (pose.sane() && body_to_actor.sane())
            {
                pose = normalized_pose(pose) * normalized_pose(body_to_actor).inverse();
            }
            actor_poses[index] = pose;
        }
        else
        {
            actor_poses[index] = static_actors[index].rigid.body_to_world;
        }
        actor_poses[index] = normalized_pose(actor_poses[index]);

        auto& actor_shapes = refreshed_shapes[index];
        actor_shapes.actor_pose = actor_poses[index];
        actor_shapes.shapes.erase(
            std::remove_if(actor_shapes.shapes.begin(), actor_shapes.shapes.end(),
                           [&](auto& shape)
                           {
                                shape.type = shape.data.core.geometry.type();
                                auto local_pose = normalized_pose(shape.data.core.local_pose);
                                if (!actor_poses[index].sane())
                                {
                                    ++invalid_actor_pose;
                                    return true;
                                }
                                if (!local_pose.sane())
                                {
                                    ++invalid_local_pose;
                                    return true;
                                }
                                if (shape.type == px_geometry_type::invalid)
                                {
                                    ++invalid_geometry;
                                    return true;
                                }
                                const auto type_index = static_cast<std::size_t>(shape.type);
                                if (type_index < geometry_counts.size())
                                {
                                    ++geometry_counts[type_index];
                                    if ((shape.data.core.shape_flags & 0x01u) != 0)
                                    {
                                        ++simulation_geometry_counts[type_index];
                                    }
                                }
                                shape.world_pose = actor_poses[index] * local_pose;
                                if (!shape.world_pose.sane())
                                {
                                    ++invalid_world_pose;
                                    return true;
                                }
                                return false;
                            }),
            actor_shapes.shapes.end());
    }

    std::size_t newly_cached_shapes = 0;
    for (auto& actor_shapes : refreshed_shapes)
    {
        const auto previous_actor = previous_actor_shapes.find(actor_shapes.actor_address);
        if (previous_actor == previous_actor_shapes.end())
        {
            newly_cached_shapes += actor_shapes.shapes.size();
            continue;
        }

        std::unordered_map<std::uint64_t, std::size_t> current_shape_indices;
        current_shape_indices.reserve(actor_shapes.shapes.size() + previous_actor->second->shapes.size());
        for (std::size_t index = 0; index < actor_shapes.shapes.size(); ++index)
        {
            current_shape_indices.emplace(actor_shapes.shapes[index].address, index);
        }
        for (const auto& previous_shape : previous_actor->second->shapes)
        {
            if (!current_shape_indices.contains(previous_shape.address))
            {
                current_shape_indices.emplace(previous_shape.address, actor_shapes.shapes.size());
                actor_shapes.shapes.push_back(previous_shape);
            }
        }
        newly_cached_shapes += actor_shapes.shapes.size() - previous_actor->second->shapes.size();
    }

    std::size_t valid_shape_count = 0;
    for (const auto& actor_shapes : refreshed_shapes)
    {
        valid_shape_count += actor_shapes.shapes.size();
    }
    m_cached_actor_shapes = std::move(refreshed_shapes);
    LOG_INFO("[physx] cached actor shapes=%zu requested=%zu new=%zu actors=%zu", valid_shape_count, shape_count,
              newly_cached_shapes, m_cached_actors.size());
    LOG_INFO("[physx:%llu] shape outcomes valid=%zu invalid_actor_pose=%zu invalid_local_pose=%zu invalid_geometry=%zu invalid_world_pose=%zu",
             m_diagnostic_id, valid_shape_count, invalid_actor_pose, invalid_local_pose, invalid_geometry,
             invalid_world_pose);
    LOG_INFO("[physx:%llu] geometry accepted sphere=%zu plane=%zu capsule=%zu box=%zu convex=%zu mesh=%zu heightfield=%zu",
             m_diagnostic_id, geometry_counts[0], geometry_counts[1], geometry_counts[2], geometry_counts[3],
             geometry_counts[4], geometry_counts[5], geometry_counts[6]);
    LOG_INFO("[physx:%llu] geometry simulation sphere=%zu plane=%zu capsule=%zu box=%zu convex=%zu mesh=%zu heightfield=%zu",
             m_diagnostic_id, simulation_geometry_counts[0], simulation_geometry_counts[1],
             simulation_geometry_counts[2], simulation_geometry_counts[3], simulation_geometry_counts[4],
             simulation_geometry_counts[5], simulation_geometry_counts[6]);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started);
    LOG_INFO("[physx:%llu] shape cache complete actors=%zu shapes=%zu elapsed_ms=%lld", m_diagnostic_id,
             m_cached_actors.size(), valid_shape_count, static_cast<long long>(elapsed.count()));
    return true;
}

bool physx::dump_shapes(void* scatter_handle)
{
    const auto started = std::chrono::steady_clock::now();
    const auto scatter = static_cast<VMMDLL_SCATTER_HANDLE>(scatter_handle);
    if (!scatter)
    {
        return false;
    }

    struct triangle_mesh_record
    {
        struct instance
        {
            transform pose{};
            std::uint64_t actor_address{};
            physx_layout::triangle_geometry geometry{};
        };

        std::uint64_t address{};
        std::vector<instance> instances{};
        physx_layout::triangle_mesh mesh{};
        std::vector<vec3> vertices{};
        std::vector<std::uint16_t> indices16{};
        std::vector<std::uint32_t> indices32{};
        std::vector<std::array<std::uint32_t, 3>> valid_faces{};
        DWORD header_bytes{};
        bool valid{};
    };
    struct convex_record
    {
        struct instance
        {
            transform pose{};
            std::uint64_t actor_address{};
            physx_layout::convex_geometry geometry{};
        };

        std::uint64_t address{};
        std::vector<instance> instances{};
        physx_layout::convex_mesh mesh{};
        std::vector<physx_layout::hull_polygon> polygons{};
        std::vector<vec3> vertices{};
        std::vector<std::uint8_t> references{};
        std::vector<std::array<std::uint8_t, 3>> valid_faces{};
        std::uint64_t references_address{};
        DWORD header_bytes{};
        bool valid{};
    };
    struct heightfield_record
    {
        struct instance
        {
            transform pose{};
            std::uint64_t actor_address{};
            physx_layout::heightfield_geometry geometry{};
        };

        std::uint64_t address{};
        std::vector<instance> instances{};
        physx_layout::heightfield heightfield{};
        std::vector<physx_layout::heightfield_sample> samples{};
        DWORD header_bytes{};
        bool valid{};
    };

    std::vector<triangle_mesh_record> triangle_meshes;
    std::vector<convex_record> convexes;
    std::vector<heightfield_record> heightfields;
    std::unordered_map<std::uint64_t, std::size_t> triangle_mesh_assets;
    std::unordered_map<std::uint64_t, std::size_t> convex_assets;
    std::unordered_map<std::uint64_t, std::size_t> heightfield_assets;
    std::vector<const cached_actor_shapes::cached_shape*> planes;
    std::vector<const cached_actor_shapes::cached_shape*> boxes;
    std::vector<const cached_actor_shapes::cached_shape*> capsules;
    std::vector<const cached_actor_shapes::cached_shape*> spheres;
    std::array<std::size_t, static_cast<std::size_t>(px_geometry_type::count)> shape_counts{};
    std::size_t mesh_pointer_candidates = 0;
    std::size_t convex_pointer_candidates = 0;
    std::size_t invalid_mesh_pointer = 0;
    std::size_t invalid_convex_pointer = 0;
    std::size_t invalid_heightfield_geometry = 0;
    std::size_t invalid_box_geometry = 0;
    std::size_t invalid_capsule_geometry = 0;
    std::size_t invalid_sphere_geometry = 0;

    for (const auto& actor : m_cached_actor_shapes)
    {
        for (const auto& shape : actor.shapes)
        {
            const auto type_index = static_cast<std::size_t>(shape.type);
            if (type_index < shape_counts.size())
            {
                ++shape_counts[type_index];
            }

            if (shape.type == px_geometry_type::triangle_mesh && m_config.triangle_meshes)
            {
                const auto geometry = geometry_as<physx_layout::triangle_geometry>(shape.data);
                if (valid_address(geometry.mesh))
                {
                    const auto [asset, inserted] = triangle_mesh_assets.emplace(geometry.mesh, triangle_meshes.size());
                    if (inserted)
                    {
                        triangle_mesh_record record{};
                        record.address = geometry.mesh;
                        triangle_meshes.push_back(std::move(record));
                    }
                    triangle_meshes[asset->second].instances.push_back({shape.world_pose, actor.actor_address,
                                                                         geometry});
                    ++mesh_pointer_candidates;
                }
                else
                {
                    ++invalid_mesh_pointer;
                }
            }
            else if (shape.type == px_geometry_type::convex_mesh && m_config.convex_meshes)
            {
                const auto geometry = geometry_as<physx_layout::convex_geometry>(shape.data);
                if (valid_address(geometry.mesh))
                {
                    const auto [asset, inserted] = convex_assets.emplace(geometry.mesh, convexes.size());
                    if (inserted)
                    {
                        convex_record record{};
                        record.address = geometry.mesh;
                        convexes.push_back(std::move(record));
                    }
                    convexes[asset->second].instances.push_back({shape.world_pose, actor.actor_address,
                                                                  geometry});
                    ++convex_pointer_candidates;
                }
                else
                {
                    ++invalid_convex_pointer;
                }
            }
            else if (shape.type == px_geometry_type::heightfield && m_config.heightfields)
            {
                const auto geometry = geometry_as<physx_layout::heightfield_geometry>(shape.data);
                if (valid_address(geometry.heightfield) && finite_positive(geometry.height_scale) &&
                    finite_positive(geometry.row_scale) && finite_positive(geometry.column_scale))
                {
                    const auto [asset, inserted] =
                        heightfield_assets.emplace(geometry.heightfield, heightfields.size());
                    if (inserted)
                    {
                        heightfield_record record{};
                        record.address = geometry.heightfield;
                        heightfields.push_back(std::move(record));
                    }
                    heightfields[asset->second].instances.push_back({shape.world_pose, actor.actor_address,
                                                                     geometry});
                }
                else
                {
                    ++invalid_heightfield_geometry;
                }
            }
            else if (shape.type == px_geometry_type::box && m_config.boxes)
            {
                const auto geometry = geometry_as<physx_layout::box_geometry>(shape.data);
                if (finite_positive(geometry.half_extents.x) && finite_positive(geometry.half_extents.y) &&
                    finite_positive(geometry.half_extents.z))
                {
                    boxes.push_back(&shape);
                }
                else
                {
                    ++invalid_box_geometry;
                }
            }
            else if (shape.type == px_geometry_type::capsule && m_config.capsules)
            {
                const auto geometry = geometry_as<physx_layout::capsule_geometry>(shape.data);
                if (finite_positive(geometry.radius) && std::isfinite(geometry.half_height) &&
                    geometry.half_height >= 0.0f && geometry.half_height < 100000.0f)
                {
                    capsules.push_back(&shape);
                }
                else
                {
                    ++invalid_capsule_geometry;
                }
            }
            else if (shape.type == px_geometry_type::sphere && m_config.spheres)
            {
                const auto geometry = geometry_as<physx_layout::sphere_geometry>(shape.data);
                if (finite_positive(geometry.radius))
                {
                    spheres.push_back(&shape);
                }
                else
                {
                    ++invalid_sphere_geometry;
                }
            }
            else if (shape.type == px_geometry_type::plane && m_config.planes &&
                     finite_positive(m_config.plane_extent))
            {
                planes.push_back(&shape);
            }
        }
    }

    LOG_INFO("[physx] geometry instances mesh=%zu unique=%zu convex=%zu unique=%zu heightfield=%zu unique=%zu",
             mesh_pointer_candidates, triangle_meshes.size(), convex_pointer_candidates, convexes.size(),
              shape_counts[static_cast<std::size_t>(px_geometry_type::heightfield)], heightfields.size());
    LOG_INFO("[physx:%llu] geometry selection raw sphere=%zu plane=%zu capsule=%zu box=%zu convex=%zu mesh=%zu heightfield=%zu selected sphere=%zu plane=%zu capsule=%zu box=%zu convex_instances=%zu mesh_instances=%zu heightfield_instances=%zu rejected mesh_pointer=%zu convex_pointer=%zu heightfield=%zu box=%zu capsule=%zu sphere=%zu",
             m_diagnostic_id, shape_counts[0], shape_counts[1], shape_counts[2], shape_counts[3], shape_counts[4],
             shape_counts[5], shape_counts[6], spheres.size(), planes.size(), capsules.size(), boxes.size(),
             convex_pointer_candidates, mesh_pointer_candidates,
             std::accumulate(heightfields.begin(), heightfields.end(), std::size_t{0},
                             [](const std::size_t count, const heightfield_record& record)
                             {
                                 return count + record.instances.size();
                             }),
             invalid_mesh_pointer, invalid_convex_pointer, invalid_heightfield_geometry, invalid_box_geometry,
             invalid_capsule_geometry, invalid_sphere_geometry);

    // Pass 1 reads every selected complex-geometry header in one scatter execution.
    for (auto& record : triangle_meshes)
    {
        if (!mem.add_scatter_read(scatter, record.address, &record.mesh, sizeof(record.mesh), &record.header_bytes))
        {
            return false;
        }
    }
    for (auto& record : convexes)
    {
        if (!mem.add_scatter_read(scatter, record.address, &record.mesh, sizeof(record.mesh), &record.header_bytes))
        {
            return false;
        }
    }
    for (auto& record : heightfields)
    {
        if (!mem.add_scatter_read(scatter, record.address, &record.heightfield, sizeof(record.heightfield),
                                  &record.header_bytes))
        {
            return false;
        }
    }
    if ((!triangle_meshes.empty() || !convexes.empty() || !heightfields.empty()) &&
        !mem.execute_read_scatter(scatter))
    {
        return false;
    }
    LOG_INFO("[physx] geometry header scatter complete assets=%zu", triangle_meshes.size() + convexes.size() +
                                                                    heightfields.size());
    const auto log_header_group = [&](const char* type, const auto& records, const std::size_t expected)
    {
        std::size_t exact = 0;
        std::size_t partial = 0;
        std::size_t failed = 0;
        std::size_t samples = 0;
        for (std::size_t index = 0; index < records.size(); ++index)
        {
            if (records[index].header_bytes == expected)
            {
                ++exact;
            }
            else if (records[index].header_bytes == 0)
            {
                ++failed;
            }
            else
            {
                ++partial;
            }
            if (records[index].header_bytes != expected && samples < 8)
            {
                LOG_WARN("[physx:%llu] geometry header failure type=%s index=%zu address=0x%llx bytes=%lu expected=%zu instances=%zu",
                         m_diagnostic_id, type, index, records[index].address, records[index].header_bytes, expected,
                         records[index].instances.size());
                ++samples;
            }
        }
        LOG_INFO("[physx:%llu] geometry headers type=%s requests=%zu exact=%zu partial=%zu failed=%zu expected=%zu",
                 m_diagnostic_id, type, records.size(), exact, partial, failed, expected);
    };
    log_header_group("mesh", triangle_meshes, sizeof(physx_layout::triangle_mesh));
    log_header_group("convex", convexes, sizeof(physx_layout::convex_mesh));
    log_header_group("heightfield", heightfields, sizeof(physx_layout::heightfield));

    // Pass 2 uses contiguous requests, but executes bounded batches so VMM never receives one enormous scatter list.
    std::size_t payload_batch_bytes = 0;
    std::size_t payload_batch_requests = 0;
    std::size_t payload_total_bytes = 0;
    std::size_t payload_total_requests = 0;
    std::size_t payload_batches = 0;
    const auto flush_payload_batch = [&]() -> bool
    {
        if (!payload_batch_requests)
        {
            return true;
        }
        if (!mem.execute_read_scatter(scatter))
        {
            return false;
        }
        payload_total_bytes += payload_batch_bytes;
        payload_total_requests += payload_batch_requests;
        ++payload_batches;
        LOG_INFO("[physx] payload scatter batch=%zu requests=%zu bytes=%zu total_mb=%zu", payload_batches,
                 payload_batch_requests, payload_batch_bytes, payload_total_bytes / (1024ull * 1024ull));
        payload_batch_bytes = 0;
        payload_batch_requests = 0;
        return true;
    };
    const auto queue_payload = [&](const std::uint64_t address, void* destination, const std::size_t size) -> bool
    {
        if (size > std::numeric_limits<DWORD>::max())
        {
            return false;
        }
        if (payload_batch_requests &&
            (payload_batch_requests >= max_payload_batch_requests ||
             size > max_payload_batch_bytes - std::min(payload_batch_bytes, max_payload_batch_bytes)))
        {
            if (!flush_payload_batch())
            {
                return false;
            }
        }
        if (!mem.add_scatter_read(scatter, address, destination, size))
        {
            LOG_WARN("[physx:%llu] geometry payload read failed va=0x%llx size=%zu",
                     m_diagnostic_id, address, size);
            return false;
        }
        payload_batch_bytes += size;
        ++payload_batch_requests;
        return true;
    };
    std::size_t valid_mesh_headers = 0;
    std::size_t valid_convex_headers = 0;
    std::size_t valid_heightfield_headers = 0;
    std::size_t recovered_mesh_headers = 0;
    std::size_t recovered_convex_headers = 0;
    std::size_t recovered_heightfield_headers = 0;
    for (auto& record : triangle_meshes)
    {
        auto& mesh = record.mesh;
        if (!valid_address(mesh.vertices) || !valid_address(mesh.indices) || !mesh.vertex_count || !mesh.triangle_count ||
            mesh.vertex_count > max_mesh_elements || mesh.triangle_count > max_mesh_elements)
        {
            mesh = {};
            if (!mem.read(record.address, &mesh, sizeof(mesh)) || !valid_address(mesh.vertices) ||
                !valid_address(mesh.indices) || !mesh.vertex_count || !mesh.triangle_count ||
                mesh.vertex_count > max_mesh_elements || mesh.triangle_count > max_mesh_elements)
            {
                continue;
            }
            ++recovered_mesh_headers;
        }
        const std::size_t index_count = static_cast<std::size_t>(mesh.triangle_count) * 3u;
        record.vertices.resize(mesh.vertex_count);
        if (!queue_payload(mesh.vertices, record.vertices.data(), record.vertices.size() * sizeof(vec3)))
        {
            LOG_WARN("[physx:%llu] skipping mesh payload asset=0x%llx vertices=0x%llx vertex_count=%u",
                     m_diagnostic_id, record.address, mesh.vertices, mesh.vertex_count);
            record.valid = false;
            continue;
        }
        if ((mesh.flags & 2u) != 0)
        {
            record.indices16.resize(index_count);
            if (!queue_payload(mesh.indices, record.indices16.data(),
                               record.indices16.size() * sizeof(std::uint16_t)))
            {
                LOG_WARN("[physx:%llu] skipping mesh index payload asset=0x%llx indices=0x%llx count=%zu",
                         m_diagnostic_id, record.address, mesh.indices, index_count);
                record.valid = false;
                continue;
            }
        }
        else
        {
            record.indices32.resize(index_count);
            if (!queue_payload(mesh.indices, record.indices32.data(),
                               record.indices32.size() * sizeof(std::uint32_t)))
            {
                LOG_WARN("[physx:%llu] skipping mesh index payload asset=0x%llx indices=0x%llx count=%zu",
                         m_diagnostic_id, record.address, mesh.indices, index_count);
                record.valid = false;
                continue;
            }
        }
        record.valid = true;
        ++valid_mesh_headers;
    }
    for (auto& record : convexes)
    {
        auto* hull = &record.mesh.hull;
        if (!valid_address(hull->polygons) || hull->vertex_count < 3 || !hull->polygon_count)
        {
            record.mesh = {};
            if (!mem.read(record.address, &record.mesh, sizeof(record.mesh)))
            {
                continue;
            }
            hull = &record.mesh.hull;
            if (!valid_address(hull->polygons) || hull->vertex_count < 3 || !hull->polygon_count)
            {
                continue;
            }
            ++recovered_convex_headers;
        }
        record.polygons.resize(hull->polygon_count);
        record.vertices.resize(hull->vertex_count);
        const std::uint64_t vertices_address =
            hull->polygons + static_cast<std::uint64_t>(hull->polygon_count) * sizeof(physx_layout::hull_polygon);
        if (!queue_payload(hull->polygons, record.polygons.data(),
                           record.polygons.size() * sizeof(physx_layout::hull_polygon)) ||
            !queue_payload(vertices_address, record.vertices.data(), record.vertices.size() * sizeof(vec3)))
        {
            LOG_WARN("[physx:%llu] skipping convex payload asset=0x%llx polygons=0x%llx vertices=0x%llx",
                     m_diagnostic_id, record.address, hull->polygons, vertices_address);
            record.valid = false;
            continue;
        }
        const std::uint16_t edge_count = hull->edge_count & 0x7fffu;
        record.references_address = vertices_address + static_cast<std::uint64_t>(hull->vertex_count) * sizeof(vec3) +
                                    static_cast<std::uint64_t>(edge_count) * 2u +
                                    static_cast<std::uint64_t>(hull->vertex_count) * 3u;
        if ((hull->edge_count & 0x8000u) != 0)
        {
            record.references_address += static_cast<std::uint64_t>(edge_count) * 2u * sizeof(std::uint16_t);
        }
        record.valid = true;
        ++valid_convex_headers;
    }
    for (auto& record : heightfields)
    {
        auto* data = &record.heightfield.data;
        if (!valid_address(data->samples) || data->rows < 2 || data->columns < 2 || data->rows > 8192 ||
            data->columns > 8192 || record.heightfield.sample_stride < sizeof(physx_layout::heightfield_sample) ||
            record.heightfield.sample_stride > 256)
        {
            record.heightfield = {};
            if (!mem.read(record.address, &record.heightfield, sizeof(record.heightfield)))
            {
                continue;
            }
            data = &record.heightfield.data;
            if (!valid_address(data->samples) || data->rows < 2 || data->columns < 2 || data->rows > 8192 ||
                data->columns > 8192 || record.heightfield.sample_stride < sizeof(physx_layout::heightfield_sample) ||
                record.heightfield.sample_stride > 256)
            {
                continue;
            }
            ++recovered_heightfield_headers;
        }
        const std::size_t count = static_cast<std::size_t>(data->rows) * data->columns;
        if (count > max_heightfield_samples ||
            (record.heightfield.sample_count && record.heightfield.sample_count < count))
        {
            continue;
        }
        record.samples.resize(count);
        if (record.heightfield.sample_stride == sizeof(physx_layout::heightfield_sample))
        {
            if (!queue_payload(data->samples, record.samples.data(),
                               record.samples.size() * sizeof(physx_layout::heightfield_sample)))
            {
                LOG_WARN("[physx:%llu] skipping heightfield payload asset=0x%llx samples=0x%llx count=%zu",
                         m_diagnostic_id, record.address, data->samples, count);
                record.valid = false;
                continue;
            }
        }
        else
        {
            for (std::size_t index = 0; index < count; ++index)
            {
                if (!queue_payload(data->samples + index * record.heightfield.sample_stride, &record.samples[index],
                                   sizeof(record.samples[index])))
                {
                    LOG_WARN("[physx:%llu] skipping heightfield payload asset=0x%llx samples=0x%llx index=%zu",
                             m_diagnostic_id, record.address, data->samples, index);
                    record.valid = false;
                    break;
                }
            }
        }
        record.valid = true;
        ++valid_heightfield_headers;
    }
    if (!flush_payload_batch())
    {
        return false;
    }
    LOG_INFO("[physx] payload scatter complete batches=%zu requests=%zu total_mb=%zu mesh_headers=%zu convex_headers=%zu heightfield_headers=%zu",
             payload_batches, payload_total_requests, payload_total_bytes / (1024ull * 1024ull), valid_mesh_headers,
             valid_convex_headers, valid_heightfield_headers);
    LOG_INFO("[physx] recovered geometry headers mesh=%zu convex=%zu heightfield=%zu", recovered_mesh_headers,
             recovered_convex_headers, recovered_heightfield_headers);

    // Pass 3 is only needed for convex polygon reference lists, whose lengths came from Pass 2.
    bool references_queued = false;
    for (auto& record : convexes)
    {
        if (!record.valid)
        {
            continue;
        }
        std::size_t reference_count = 0;
        for (const auto& polygon : record.polygons)
        {
            reference_count = std::max(reference_count,
                                       static_cast<std::size_t>(polygon.vertex_ref) + polygon.vertex_count);
        }
        if (!reference_count || reference_count > max_actor_shapes)
        {
            record.valid = false;
            continue;
        }
        record.references.resize(reference_count);
        if (!mem.add_scatter_read(scatter, record.references_address, record.references.data(), record.references.size()))
        {
            return false;
        }
        references_queued = true;
    }
    if (references_queued && !mem.execute_read_scatter(scatter))
    {
        return false;
    }
    LOG_INFO("[physx] convex reference scatter complete");

    const auto checked_add_triangles = [](std::size_t& total, const std::size_t per_instance,
                                          const std::size_t instances) -> bool
    {
        if (per_instance && instances > (std::numeric_limits<std::size_t>::max() - total) / per_instance)
        {
            return false;
        }
        total += per_instance * instances;
        return true;
    };
    const auto valid_triangle_instance = [](const triangle_mesh_record::instance& instance)
    {
        return finite_positive(instance.geometry.scale.x) && finite_positive(instance.geometry.scale.y) &&
               finite_positive(instance.geometry.scale.z);
    };
    const auto valid_convex_instance = [](const convex_record::instance& instance)
    {
        return finite_positive(instance.geometry.scale.value.x) && finite_positive(instance.geometry.scale.value.y) &&
               finite_positive(instance.geometry.scale.value.z);
    };

    std::size_t expected_triangles = 0;
    std::size_t expanded_instances = 0;
    std::size_t invalid_triangle_instances = 0;
    std::size_t invalid_convex_instances = 0;
    std::size_t convex_records_without_references = 0;
    for (auto& record : triangle_meshes)
    {
        if (!record.valid)
        {
            continue;
        }
        const auto index_at = [&](const std::size_t index) -> std::uint32_t
        {
            return record.indices16.empty() ? record.indices32[index] : record.indices16[index];
        };
        const std::size_t index_count = record.indices16.empty() ? record.indices32.size() : record.indices16.size();
        record.valid_faces.reserve(index_count / 3u);
        std::size_t valid_faces = 0;
        for (std::size_t index = 0; index + 2 < index_count; index += 3)
        {
            const std::uint32_t i0 = index_at(index);
            const std::uint32_t i1 = index_at(index + 1u);
            const std::uint32_t i2 = index_at(index + 2u);
            if (i0 < record.vertices.size() && i1 < record.vertices.size() && i2 < record.vertices.size())
            {
                ++valid_faces;
                record.valid_faces.push_back({i0, i1, i2});
            }
        }
        const std::size_t valid_instances =
            static_cast<std::size_t>(std::count_if(record.instances.begin(), record.instances.end(), valid_triangle_instance));
        invalid_triangle_instances += record.instances.size() - valid_instances;
        if (!checked_add_triangles(expected_triangles, valid_faces, valid_instances))
        {
            LOG_ERROR("[physx] expanded triangle count overflow");
            return false;
        }
        expanded_instances += valid_instances;
    }
    for (auto& record : convexes)
    {
        if (!record.valid || record.references.empty())
        {
            convex_records_without_references += record.instances.size();
            continue;
        }
        std::size_t valid_faces = 0;
        std::size_t possible_faces = 0;
        for (const auto& polygon : record.polygons)
        {
            if (polygon.vertex_count >= 3)
            {
                possible_faces += polygon.vertex_count - 2u;
            }
        }
        record.valid_faces.reserve(possible_faces);
        for (const auto& polygon : record.polygons)
        {
            const std::size_t first = polygon.vertex_ref;
            if (polygon.vertex_count < 3 || first + polygon.vertex_count > record.references.size())
            {
                continue;
            }
            const std::uint8_t i0 = record.references[first];
            for (std::size_t vertex = 1; vertex + 1 < polygon.vertex_count; ++vertex)
            {
                if (i0 < record.vertices.size() && record.references[first + vertex] < record.vertices.size() &&
                    record.references[first + vertex + 1u] < record.vertices.size())
                {
                    ++valid_faces;
                    record.valid_faces.push_back({i0, record.references[first + vertex],
                                                 record.references[first + vertex + 1u]});
                }
            }
        }
        const std::size_t valid_instances =
            static_cast<std::size_t>(std::count_if(record.instances.begin(), record.instances.end(), valid_convex_instance));
        invalid_convex_instances += record.instances.size() - valid_instances;
        if (!checked_add_triangles(expected_triangles, valid_faces, valid_instances))
        {
            LOG_ERROR("[physx] expanded convex count overflow");
            return false;
        }
        expanded_instances += valid_instances;
    }
    for (const auto& record : heightfields)
    {
        if (!record.valid)
        {
            continue;
        }
        std::size_t valid_faces = 0;
        const auto& data = record.heightfield.data;
        for (std::uint32_t row = 0; row + 1u < data.rows; ++row)
        {
            for (std::uint32_t column = 0; column + 1u < data.columns; ++column)
            {
                const auto& sample = record.samples[static_cast<std::size_t>(row) * data.columns + column];
                valid_faces += (sample.material0 & 0x7fu) != 0x7fu;
                valid_faces += (sample.material1 & 0x7fu) != 0x7fu;
            }
        }
        if (!checked_add_triangles(expected_triangles, valid_faces, record.instances.size()))
        {
            LOG_ERROR("[physx] expanded heightfield count overflow");
            return false;
        }
        expanded_instances += record.instances.size();
    }
    const std::size_t sphere_slices = std::clamp<std::size_t>(m_config.sphere_slices, 3u, 128u);
    const std::size_t sphere_rings = std::clamp<std::size_t>(m_config.sphere_rings, 2u, 64u);
    const std::size_t capsule_slices = std::clamp<std::size_t>(m_config.capsule_slices, 3u, 128u);
    const std::size_t capsule_rings = std::clamp<std::size_t>(m_config.capsule_hemisphere_rings, 1u, 32u);
    if (!checked_add_triangles(expected_triangles, 2u, planes.size()) ||
        !checked_add_triangles(expected_triangles, 12u, boxes.size()) ||
        !checked_add_triangles(expected_triangles, sphere_slices * sphere_rings * 2u, spheres.size()) ||
        !checked_add_triangles(expected_triangles, capsule_slices * capsule_rings * 4u, capsules.size()))
    {
        LOG_ERROR("[physx] expanded primitive count overflow");
        return false;
    }
    expanded_instances += planes.size() + boxes.size() + spheres.size() + capsules.size();

    const std::size_t expected_megabytes =
        expected_triangles > std::numeric_limits<std::size_t>::max() / sizeof(triangle)
            ? std::numeric_limits<std::size_t>::max()
            : expected_triangles * sizeof(triangle) / (1024ull * 1024ull);
    LOG_INFO("[physx] expanding instances=%zu expected_triangles=%zu output_mb=%zu", expanded_instances,
              expected_triangles, expected_megabytes);
    LOG_INFO("[physx:%llu] expansion eligibility invalid_mesh_instances=%zu invalid_convex_instances=%zu convex_instances_without_references=%zu",
             m_diagnostic_id, invalid_triangle_instances, invalid_convex_instances,
             convex_records_without_references);

    std::vector<triangle> dumped;
    m_dynamic_triangle_ranges.clear();
    const auto begin_dynamic_range = [&](const std::uint64_t actor_address, const std::size_t first,
                                         const std::size_t count, const transform& pose)
    {
        if (actor_address && count && actor_address < std::numeric_limits<std::uint64_t>::max())
        {
            const auto actor = std::find_if(m_cached_actors.begin(), m_cached_actors.end(),
                                            [&](const cached_actor& value) { return value.address == actor_address; });
            if (actor == m_cached_actors.end())
            {
                return;
            }
            if (actor->type == rigid_dynamic_type)
            {
                for (std::size_t index = first; index < first + count && index < dumped.size(); ++index)
                {
                    dumped[index].dynamic = true;
                }
            }
            transform actor_pose{};
            const auto actor_shapes = std::find_if(m_cached_actor_shapes.begin(), m_cached_actor_shapes.end(),
                                                   [&](const cached_actor_shapes& value)
                                                   { return value.actor_address == actor_address; });
            if (actor_shapes != m_cached_actor_shapes.end()) actor_pose = actor_shapes->actor_pose;
            if (!actor_pose.sane())
            {
                return;
            }
            m_dynamic_triangle_ranges.push_back({actor_address, actor->type, first, count, pose, actor_pose});
        }
    };
    try
    {
        dumped.reserve(expected_triangles);
    }
    catch (const std::bad_alloc&)
    {
        LOG_ERROR("[physx] unable to allocate output for %zu triangles (%zu MB)", expected_triangles,
                  expected_megabytes);
        return false;
    }
    std::size_t mesh_triangles = 0;
    std::size_t convex_triangles = 0;
    std::size_t heightfield_triangles = 0;
    std::size_t primitive_triangles = 0;
    std::size_t completed_instances = 0;
    std::vector<vec3> transformed_vertices;
    for (const auto& record : triangle_meshes)
    {
        if (!record.valid)
        {
            continue;
        }
        for (const auto& instance : record.instances)
        {
            if (!valid_triangle_instance(instance))
            {
                continue;
            }
            const std::size_t first = dumped.size();
            auto geometry = instance.geometry;
            if (geometry.scale_rotation.sane())
            {
                geometry.scale_rotation.normalize();
            }
            else
            {
                geometry.scale_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
            }
            transformed_vertices.resize(record.vertices.size());
            const quaternion inverse_scale_rotation = geometry.scale_rotation.conjugate();
            for (std::size_t vertex = 0; vertex < record.vertices.size(); ++vertex)
            {
                const vec3 scaled = inverse_scale_rotation.rotate(
                    geometry.scale_rotation.rotate(record.vertices[vertex]) * geometry.scale);
                transformed_vertices[vertex] = instance.pose.point(scaled);
            }
            for (const auto& face : record.valid_faces)
            {
                dumped.push_back({transformed_vertices[face[0]], transformed_vertices[face[1]],
                                  transformed_vertices[face[2]],
                                  px_geometry_type::triangle_mesh, instance.actor_address});
                ++mesh_triangles;
            }
            if (++completed_instances % 4096u == 0)
            {
                LOG_INFO("[physx] expansion progress instances=%zu/%zu triangles=%zu", completed_instances,
                         expanded_instances, dumped.size());
            }
            begin_dynamic_range(instance.actor_address, first, dumped.size() - first, instance.pose);
        }
    }
    for (const auto& record : convexes)
    {
        if (!record.valid || record.references.empty())
        {
            continue;
        }
        for (const auto& instance : record.instances)
        {
            if (!valid_convex_instance(instance))
            {
                continue;
            }
            const std::size_t first = dumped.size();
            auto scale = instance.geometry.scale;
            if (scale.rotation.sane())
            {
                scale.rotation.normalize();
            }
            else
            {
                scale.rotation = {0.0f, 0.0f, 0.0f, 1.0f};
            }
            transformed_vertices.resize(record.vertices.size());
            const quaternion inverse_scale_rotation = scale.rotation.conjugate();
            for (std::size_t vertex = 0; vertex < record.vertices.size(); ++vertex)
            {
                const vec3 scaled = inverse_scale_rotation.rotate(scale.rotation.rotate(record.vertices[vertex]) *
                                                                  scale.value);
                transformed_vertices[vertex] = instance.pose.point(scaled);
            }
            for (const auto& face : record.valid_faces)
            {
                dumped.push_back({transformed_vertices[face[0]], transformed_vertices[face[1]],
                                  transformed_vertices[face[2]], px_geometry_type::convex_mesh, instance.actor_address});
                ++convex_triangles;
            }
                begin_dynamic_range(instance.actor_address, first, dumped.size() - first,
                                instance.pose);
            if (++completed_instances % 4096u == 0)
            {
                LOG_INFO("[physx] expansion progress instances=%zu/%zu triangles=%zu", completed_instances,
                         expanded_instances, dumped.size());
            }
        }
    }
    for (const auto& record : heightfields)
    {
        if (!record.valid)
        {
            continue;
        }
        const auto& data = record.heightfield.data;
        for (const auto& instance : record.instances)
        {
            const std::size_t first = dumped.size();
            const auto vertex = [&](const std::uint32_t row, const std::uint32_t column)
            {
                const auto& sample = record.samples[static_cast<std::size_t>(row) * data.columns + column];
                return vec3{static_cast<float>(row) * instance.geometry.row_scale,
                            static_cast<float>(sample.height) * instance.geometry.height_scale,
                            static_cast<float>(column) * instance.geometry.column_scale};
            };
            for (std::uint32_t row = 0; row + 1u < data.rows; ++row)
            {
                for (std::uint32_t column = 0; column + 1u < data.columns; ++column)
                {
                    const auto& sample = record.samples[static_cast<std::size_t>(row) * data.columns + column];
                    const vec3 v0 = vertex(row, column);
                    const vec3 v1 = vertex(row + 1u, column);
                    const vec3 v2 = vertex(row, column + 1u);
                    const vec3 v3 = vertex(row + 1u, column + 1u);
                    const bool tessellated = (sample.material0 & 0x80u) != 0;
                    if ((sample.material0 & 0x7fu) != 0x7fu)
                    {
                        if (tessellated) add_triangle(dumped, instance.pose, v1, v0, v3, px_geometry_type::heightfield,
                                                       instance.actor_address);
                        else add_triangle(dumped, instance.pose, v0, v2, v1, px_geometry_type::heightfield,
                                          instance.actor_address);
                        ++heightfield_triangles;
                    }
                    if ((sample.material1 & 0x7fu) != 0x7fu)
                    {
                        if (tessellated) add_triangle(dumped, instance.pose, v2, v3, v0, px_geometry_type::heightfield,
                                                       instance.actor_address);
                        else add_triangle(dumped, instance.pose, v3, v1, v2, px_geometry_type::heightfield,
                                          instance.actor_address);
                        ++heightfield_triangles;
                    }
                }
            }
            if (++completed_instances % 4096u == 0)
            {
                LOG_INFO("[physx] expansion progress instances=%zu/%zu triangles=%zu", completed_instances,
                         expanded_instances, dumped.size());
            }
                begin_dynamic_range(instance.actor_address, first, dumped.size() - first,
                                instance.pose);
        }
    }

    for (const auto* shape : boxes)
    {
        const std::size_t first = dumped.size();
        const vec3 h = geometry_as<physx_layout::box_geometry>(shape->data).half_extents;
        const vec3 vertices[8] = {{-h.x, -h.y, -h.z}, {h.x, -h.y, -h.z}, {h.x, h.y, -h.z}, {-h.x, h.y, -h.z},
                                  {-h.x, -h.y, h.z},  {h.x, -h.y, h.z},  {h.x, h.y, h.z},  {-h.x, h.y, h.z}};
        constexpr std::uint8_t indices[12][3] = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6},
                                                  {0, 4, 5}, {0, 5, 1}, {3, 2, 6}, {3, 6, 7},
                                                  {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2}};
        for (const auto& face : indices)
        {
            add_triangle(dumped, shape->world_pose, vertices[face[0]], vertices[face[1]], vertices[face[2]],
                         px_geometry_type::box, shape->actor_address);
            ++primitive_triangles;
        }
        ++completed_instances;
        begin_dynamic_range(shape->actor_address, first, dumped.size() - first,
                            shape->world_pose);
    }
    for (const auto* shape : planes)
    {
        const float extent = m_config.plane_extent;
        const std::size_t first = dumped.size();
        add_triangle(dumped, shape->world_pose, {0.0f, -extent, -extent}, {0.0f, extent, -extent},
                     {0.0f, extent, extent}, px_geometry_type::plane, shape->actor_address);
        add_triangle(dumped, shape->world_pose, {0.0f, -extent, -extent}, {0.0f, extent, extent},
                     {0.0f, -extent, extent}, px_geometry_type::plane, shape->actor_address);
        primitive_triangles += 2u;
        ++completed_instances;
        begin_dynamic_range(shape->actor_address, first, dumped.size() - first,
                            shape->world_pose);
    }
    for (const auto* shape : spheres)
    {
        const float radius = geometry_as<physx_layout::sphere_geometry>(shape->data).radius;
        const std::size_t first = dumped.size();
        for (std::size_t ring = 0; ring < sphere_rings; ++ring)
        {
            const float latitude0 = -pi * 0.5f + pi * static_cast<float>(ring) / static_cast<float>(sphere_rings);
            const float latitude1 = -pi * 0.5f + pi * static_cast<float>(ring + 1u) / static_cast<float>(sphere_rings);
            const float x0 = std::sin(latitude0) * radius, x1 = std::sin(latitude1) * radius;
            const float radial0 = std::cos(latitude0) * radius, radial1 = std::cos(latitude1) * radius;
            for (std::size_t slice = 0; slice < sphere_slices; ++slice)
            {
                const float angle0 = 2.0f * pi * static_cast<float>(slice) / static_cast<float>(sphere_slices);
                const float angle1 = 2.0f * pi * static_cast<float>(slice + 1u) / static_cast<float>(sphere_slices);
                const vec3 a{x0, radial0 * std::cos(angle0), radial0 * std::sin(angle0)};
                const vec3 b{x0, radial0 * std::cos(angle1), radial0 * std::sin(angle1)};
                const vec3 c{x1, radial1 * std::cos(angle1), radial1 * std::sin(angle1)};
                const vec3 d{x1, radial1 * std::cos(angle0), radial1 * std::sin(angle0)};
                add_triangle(dumped, shape->world_pose, a, b, c, px_geometry_type::sphere, shape->actor_address);
                add_triangle(dumped, shape->world_pose, a, c, d, px_geometry_type::sphere, shape->actor_address);
                primitive_triangles += 2u;
            }
        }
        ++completed_instances;
        begin_dynamic_range(shape->actor_address, first, dumped.size() - first,
                            shape->world_pose);
    }
    for (const auto* shape : capsules)
    {
        const auto geometry = geometry_as<physx_layout::capsule_geometry>(shape->data);
        const std::size_t first = dumped.size();
        const std::size_t profile_rings = capsule_rings * 2u;
        for (std::size_t ring = 0; ring < profile_rings; ++ring)
        {
            const float latitude0 = -pi * 0.5f + pi * static_cast<float>(ring) / static_cast<float>(profile_rings);
            const float latitude1 = -pi * 0.5f + pi * static_cast<float>(ring + 1u) / static_cast<float>(profile_rings);
            const float x0 = std::sin(latitude0) * geometry.radius + (latitude0 < 0.0f ? -geometry.half_height : geometry.half_height);
            const float x1 = std::sin(latitude1) * geometry.radius + (latitude1 < 0.0f ? -geometry.half_height : geometry.half_height);
            const float radial0 = std::cos(latitude0) * geometry.radius;
            const float radial1 = std::cos(latitude1) * geometry.radius;
            for (std::size_t slice = 0; slice < capsule_slices; ++slice)
            {
                const float angle0 = 2.0f * pi * static_cast<float>(slice) / static_cast<float>(capsule_slices);
                const float angle1 = 2.0f * pi * static_cast<float>(slice + 1u) / static_cast<float>(capsule_slices);
                const vec3 a{x0, radial0 * std::cos(angle0), radial0 * std::sin(angle0)};
                const vec3 b{x0, radial0 * std::cos(angle1), radial0 * std::sin(angle1)};
                const vec3 c{x1, radial1 * std::cos(angle1), radial1 * std::sin(angle1)};
                const vec3 d{x1, radial1 * std::cos(angle0), radial1 * std::sin(angle0)};
                add_triangle(dumped, shape->world_pose, a, b, c, px_geometry_type::capsule, shape->actor_address);
                add_triangle(dumped, shape->world_pose, a, c, d, px_geometry_type::capsule, shape->actor_address);
                primitive_triangles += 2u;
            }
        }
        ++completed_instances;
        begin_dynamic_range(shape->actor_address, first, dumped.size() - first,
                            shape->world_pose);
    }

    LOG_INFO("[physx] expansion complete instances=%zu triangles=%zu", completed_instances, dumped.size());
    LOG_INFO("[physx:%llu] tracked geometry ranges=%zu dynamic_triangles=%zu", m_diagnostic_id,
             m_dynamic_triangle_ranges.size(),
             static_cast<std::size_t>(std::count_if(dumped.begin(), dumped.end(),
                                                    [](const triangle& value) { return value.dynamic; })));

    auto snapshot = std::make_shared<render_snapshot>();
    snapshot->topology_generation = ++m_topology_generation;
    snapshot->triangles = std::move(dumped);
    snapshot->actor_count = m_cached_actors.size();
    for (const auto& actor_shapes : m_cached_actor_shapes)
    {
        snapshot->shape_count += actor_shapes.shapes.size();
    }
    snapshot->spatial_cells.reserve(snapshot->triangles.size() / 32u + 1u);
    for (std::size_t index = 0; index < snapshot->triangles.size(); ++index)
    {
        const vec3 center = triangle_center(snapshot->triangles[index]);
        if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
            index > std::numeric_limits<std::uint32_t>::max())
        {
            continue;
        }
        snapshot->spatial_cells[spatial_key(spatial_coordinate(center.x), spatial_coordinate(center.y),
                                            spatial_coordinate(center.z))]
            .push_back(static_cast<std::uint32_t>(index));
    }
    m_render_snapshot = std::move(snapshot);
    LOG_INFO("[physx] geometry pipeline mesh candidates=%zu valid_headers=%zu triangles=%zu convex candidates=%zu valid_headers=%zu triangles=%zu heightfield valid_headers=%zu triangles=%zu",
             mesh_pointer_candidates, valid_mesh_headers, mesh_triangles, convex_pointer_candidates,
             valid_convex_headers, convex_triangles, valid_heightfield_headers, heightfield_triangles);
    LOG_INFO("[physx] primitive triangles=%zu planes=%zu boxes=%zu capsules=%zu spheres=%zu", primitive_triangles,
             planes.size(), boxes.size(), capsules.size(), spheres.size());
    LOG_INFO("[physx] dumped triangles=%zu mesh_shapes=%zu convex_shapes=%zu heightfields=%zu planes=%zu boxes=%zu capsules=%zu spheres=%zu",
             m_render_snapshot->triangles.size(), shape_counts[static_cast<std::size_t>(px_geometry_type::triangle_mesh)],
             shape_counts[static_cast<std::size_t>(px_geometry_type::convex_mesh)],
             shape_counts[static_cast<std::size_t>(px_geometry_type::heightfield)],
             shape_counts[static_cast<std::size_t>(px_geometry_type::plane)],
             shape_counts[static_cast<std::size_t>(px_geometry_type::box)],
              shape_counts[static_cast<std::size_t>(px_geometry_type::capsule)],
              shape_counts[static_cast<std::size_t>(px_geometry_type::sphere)]);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started);
    LOG_INFO("[physx:%llu] geometry dump complete triangles=%zu cells=%zu mesh_triangles=%zu convex_triangles=%zu heightfield_triangles=%zu primitive_triangles=%zu elapsed_ms=%lld",
             m_diagnostic_id, m_render_snapshot->triangles.size(), m_render_snapshot->spatial_cells.size(),
             mesh_triangles, convex_triangles, heightfield_triangles, primitive_triangles,
             static_cast<long long>(elapsed.count()));
    return true;
}
