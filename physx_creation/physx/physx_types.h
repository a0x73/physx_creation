#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

enum class px_geometry_type : std::int32_t
{
    sphere,
    plane,
    capsule,
    box,
    convex_mesh,
    triangle_mesh,
    heightfield,
    count,
    invalid = -1
};

struct vec3
{
    float x{};
    float y{};
    float z{};

    vec3 operator+(const vec3& value) const
    {
        return {x + value.x, y + value.y, z + value.z};
    }

    vec3 operator*(const float value) const
    {
        return {x * value, y * value, z * value};
    }

    vec3 operator*(const vec3& value) const
    {
        return {x * value.x, y * value.y, z * value.z};
    }
};

struct alignas(16) quaternion
{
    float x{};
    float y{};
    float z{};
    float w{};

    bool sane() const
    {
        const float length = x * x + y * y + z * z + w * w;
        return std::isfinite(length) && length > 0.5f && length < 1.5f;
    }

    void normalize()
    {
        const float inverse = 1.0f / std::sqrt(x * x + y * y + z * z + w * w);
        x *= inverse;
        y *= inverse;
        z *= inverse;
        w *= inverse;
    }

    quaternion conjugate() const
    {
        return {-x, -y, -z, w};
    }

    quaternion operator*(const quaternion& value) const
    {
        return {w * value.x + x * value.w + y * value.z - z * value.y,
                w * value.y - x * value.z + y * value.w + z * value.x,
                w * value.z + x * value.y - y * value.x + z * value.w,
                w * value.w - x * value.x - y * value.y - z * value.z};
    }

    vec3 rotate(const vec3& value) const
    {
        const quaternion point{value.x, value.y, value.z, 0.0f};
        const quaternion result = *this * point * conjugate();
        return {result.x, result.y, result.z};
    }
};

struct transform
{
    alignas(16) quaternion rotation{};
    vec3 position{};

    bool sane() const
    {
        return rotation.sane() && std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
    }

    vec3 point(const vec3& value) const
    {
        return rotation.rotate(value) + position;
    }

    transform operator*(const transform& value) const
    {
        return {rotation * value.rotation, rotation.rotate(value.position) + position};
    }

    transform inverse() const
    {
        const quaternion inverse_rotation = rotation.conjugate();
        return {inverse_rotation, inverse_rotation.rotate(position * -1.0f)};
    }
};

struct triangle
{
    vec3 v0{};
    vec3 v1{};
    vec3 v2{};
    px_geometry_type geometry_type{px_geometry_type::invalid};
    std::uint64_t actor_address{};
    bool dynamic{};
};

inline vec3 triangle_center(const triangle& value)
{
    return {(value.v0.x + value.v1.x + value.v2.x) / 3.0f,
            (value.v0.y + value.v1.y + value.v2.y) / 3.0f,
            (value.v0.z + value.v1.z + value.v2.z) / 3.0f};
}

namespace physx_layout
{
struct base
{
    std::byte padding[0x8]{};
    std::uint16_t type{};
    std::uint16_t flags{};
    std::uint64_t user_data{};
};

struct shape_table
{
    std::uint64_t data{};
    std::uint16_t count{};
    bool owns_memory{};
    bool buffer_used{};
    std::byte padding[0x4]{};
};

struct rigid_actor : base
{
    std::uint64_t name{};
    std::uint64_t connectors{};
    shape_table shapes{};
    std::byte shape_manager_tail[0x20]{};
    std::uint32_t index{};
    std::byte tail[0x4]{};
};

struct rigid_core
{
    alignas(16) transform body_to_world{};
    std::byte tail[0x10]{};
};

struct body_core
{
    std::byte padding[0x10]{};
    alignas(16) rigid_core rigid{};
    alignas(16) transform body_to_actor{};
    std::byte tail[0x70]{};
    std::uint64_t simulation_state{};
};

struct dynamic_actor : rigid_actor
{
    std::uint64_t scene{};
    std::uint32_t control_state{};
    std::uint32_t control_padding{};
    std::uint64_t stream{};
    body_core body{};
    alignas(16) transform buffered_body_to_world{};
    std::byte tail[0x28]{};
};

struct static_actor : rigid_actor
{
    std::uint64_t scene{};
    std::uint32_t control_state{};
    std::uint32_t control_padding{};
    std::uint64_t stream{};
    std::byte actor_core[0x10]{};
    rigid_core rigid{};
};

struct geometry_union
{
    std::byte bytes[80]{};

    px_geometry_type type() const
    {
        px_geometry_type result{};
        std::memcpy(&result, bytes, sizeof(result));
        return result >= px_geometry_type::sphere && result <= px_geometry_type::heightfield ? result  : px_geometry_type::invalid;
    }
};

struct shape_core
{
    alignas(16) transform local_pose{};
    float contact_offset{};
    std::uint8_t shape_flags{};
    std::uint8_t owns_material{};
    std::uint16_t material_index{};
    geometry_union geometry{};
};

struct shape
{
    base base_data{};
    std::uint64_t ref_vtable{};
    std::int32_t ref_count{};
    std::byte ref_padding[0x4]{};
    std::uint64_t actor{};
    std::uint64_t scene{};
    std::uint32_t control_state{};
    std::uint32_t control_padding{};
    std::uint64_t stream{};
    std::byte query_filter[0x10]{};
    std::byte simulation_filter[0x10]{};
    alignas(16) shape_core core{};
    float rest_offset{};
    std::byte tail[0x4]{};
    std::uint64_t name{};
    std::int32_t exclusive_actor_count{};
    std::byte final_padding[0x4]{};
};

struct mesh_scale
{
    vec3 value{};
    std::byte padding[0x4]{};
    alignas(16) quaternion rotation{};

    vec3 apply(const vec3& vertex) const
    {
        quaternion scale_rotation = rotation;
        if (!scale_rotation.sane())
        {
            scale_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
        }
        else
        {
            scale_rotation.normalize();
        }
        return scale_rotation.conjugate().rotate(scale_rotation.rotate(vertex) * value);
    }
};

struct sphere_geometry
{
    px_geometry_type type{};
    float radius{};
};

struct plane_geometry
{
    px_geometry_type type{};
};

struct capsule_geometry
{
    px_geometry_type type{};
    float radius{};
    float half_height{};
};

struct box_geometry
{
    px_geometry_type type{};
    vec3 half_extents{};
};

struct triangle_geometry
{
    px_geometry_type type{};
    vec3 scale{};
    alignas(16) quaternion scale_rotation{};
    std::uint8_t flags{};
    std::byte flag_padding[0x7]{};
    std::uint64_t mesh{};

    vec3 apply(const vec3& vertex) const
    {
        return mesh_scale{scale, {}, scale_rotation}.apply(vertex);
    }
};

struct convex_geometry
{
    px_geometry_type type{};
    std::byte padding[0xC]{};
    mesh_scale scale{};
    std::uint64_t mesh{};
    std::uint8_t flags{};
    std::byte tail[0x7]{};
};

struct heightfield_geometry
{
    px_geometry_type type{};
    std::byte padding[0x4]{};
    std::uint64_t heightfield{};
    float height_scale{};
    float row_scale{};
    float column_scale{};
    std::uint8_t flags{};
    std::byte tail[0x3]{};
};

struct bounds { vec3 center{}; vec3 extents{}; };

struct triangle_mesh
{
    std::byte padding[0x20]{};
    std::uint32_t vertex_count{};
    std::uint32_t triangle_count{};
    std::uint64_t vertices{};
    std::uint64_t indices{};
    bounds aabb{};
    std::uint64_t extra_triangle_data{};
    float epsilon{};
    std::uint8_t flags{};
    std::byte tail[0x43]{};
};

struct hull_polygon
{
    vec3 normal{};
    float distance{};
    std::uint16_t vertex_ref{};
    std::uint8_t vertex_count{};
    std::uint8_t minimum_index{};
};

struct hull_data
{
    bounds aabb{};
    vec3 center_of_mass{};
    std::uint16_t edge_count{};
    std::uint8_t vertex_count{};
    std::uint8_t polygon_count{};
    std::uint64_t polygons{};
    std::uint64_t big_convex_data{};
    std::byte internal[0x10]{};
};

struct convex_mesh
{
    std::byte padding[0x20]{};
    hull_data hull{};
};

struct heightfield_sample
{
    std::int16_t height{};
    std::uint8_t material0{};
    std::uint8_t material1{};
};

struct heightfield_data
{
    bounds aabb{};
    std::uint32_t rows{};
    std::uint32_t columns{};
    float row_limit{};
    float column_limit{};
    float column_count{};
    std::uint64_t samples{};
    float thickness{};
    float convex_edge_threshold{};
    std::uint16_t flags{};
    std::uint8_t format{};
    std::uint8_t padding{};
};

struct heightfield
{
    std::byte padding[0x20]{};
    heightfield_data data{};
    std::uint32_t sample_stride{};
    std::uint32_t sample_count{};
    std::byte tail[0x18]{};
};

static_assert(sizeof(rigid_actor) == 0x60);
static_assert(offsetof(rigid_actor, shapes) == 0x28);
static_assert(offsetof(shape, actor) == 0x28);
static_assert(offsetof(shape, core) == 0x70);
static_assert(offsetof(shape_core, geometry) == 0x28);
static_assert(sizeof(hull_polygon) == 0x14);
static_assert(offsetof(sphere_geometry, radius) == 0x4);
static_assert(offsetof(capsule_geometry, radius) == 0x4);
static_assert(offsetof(capsule_geometry, half_height) == 0x8);
static_assert(offsetof(box_geometry, half_extents) == 0x4);
static_assert(offsetof(triangle_geometry, scale) == 0x4);
static_assert(offsetof(triangle_geometry, scale_rotation) == 0x10);
static_assert(offsetof(triangle_geometry, flags) == 0x20);
static_assert(offsetof(triangle_geometry, mesh) == 0x28);
static_assert(offsetof(convex_geometry, mesh) == 0x30);
static_assert(offsetof(triangle_mesh, vertices) == 0x28);
static_assert(offsetof(heightfield, sample_stride) == 0x68);
}
