#pragma once

#include <cstdint>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>

#include "physx_types.h"

class physx
{
public:
    struct render_snapshot
    {
        std::vector<triangle> triangles{};
        std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> spatial_cells{};
        std::size_t actor_count{};
        std::size_t shape_count{};
        std::uint64_t topology_generation{};
        std::vector<std::uint64_t> updated_dynamic_actors{};
        bool raycast(const vec3& origin, const vec3& direction, float max_distance, vec3& hit) const;
    };

    struct config
    {
        bool triangle_meshes{true};
        bool convex_meshes{true};
        bool heightfields{true};
        bool planes{false};
        bool boxes{false};
        bool capsules{false};
        bool spheres{false};
        std::uint32_t sphere_slices{16};
        std::uint32_t sphere_rings{12};
        std::uint32_t capsule_slices{16};
        std::uint32_t capsule_hemisphere_rings{6};
        float plane_extent{1000.0f};
    };

    physx();
    explicit physx(config configuration);
    ~physx();

    std::shared_ptr<const render_snapshot> get_render_snapshot() const
    {
        return m_render_snapshot;
    }
    bool refresh();
    void reset_object_data();
    bool regrab_object_data();
    void request_regrab();
    bool tick(const vec3& query_origin, bool query_origin_valid);
    bool refresh_dynamic(const vec3& query_origin, bool query_origin_valid);
    bool raycast(const vec3& origin, const vec3& direction, float max_distance, vec3& hit) const;

private:
    struct cached_actor
    {
        std::uint64_t address{};
        std::uint16_t type{};
        std::uint64_t shape_data{};
        std::uint16_t shape_count{};
    };

    struct cached_actor_shapes
    {
        struct cached_shape
        {
            std::uint64_t address{};
            std::uint64_t actor_address{};
            physx_layout::shape data{};
            transform world_pose{};
            px_geometry_type type{px_geometry_type::invalid};
        };

        std::uint64_t actor_address{};
        transform actor_pose{};
        std::uint64_t shape_data{};
        std::uint16_t shape_count{};
        std::vector<std::uint64_t> addresses{};
        std::vector<cached_shape> shapes{};
    };

    static constexpr std::uint64_t sdk_offset = 0x1AD18B0;

    bool cache_actors(void* scatter_handle);
    bool cache_actor_shapes(void* scatter_handle);

    bool dump_shapes(void* scatter_handle);

    struct dynamic_triangle_range
    {
        std::uint64_t actor_address{};
        std::uint16_t actor_type{};
        std::size_t first{};
        std::size_t count{};
        transform pose{};
        transform actor_pose{};
    };

    std::vector<cached_actor> m_cached_actors{};
    std::vector<cached_actor_shapes> m_cached_actor_shapes{};
    std::shared_ptr<const render_snapshot> m_render_snapshot{std::make_shared<render_snapshot>()};
    std::vector<dynamic_triangle_range> m_dynamic_triangle_ranges{};
    config m_config{};
    void* m_scatter_handle{};
    std::atomic<std::uint64_t> m_refresh_sequence{};
    std::uint64_t m_diagnostic_id{};
    std::uint64_t m_topology_generation{};
    std::unordered_map<std::uint64_t, bool> m_dynamic_actor_active{};
    std::atomic<bool> m_regrab_requested{};
};
