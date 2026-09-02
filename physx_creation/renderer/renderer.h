#pragma once

#include <Windows.h>
#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <chrono>

#include "../camera/camera.h"
#include "../physx/physx.h"

#if defined(_WIN64)
#include <embree4/rtcore.h>
#endif

struct ID3D11Device;
struct ID3D11DeviceContext;
struct ID3D11RenderTargetView;
struct IDXGISwapChain;

class renderer
{
public:
    explicit renderer(HWND window);
    ~renderer();
    void render_frame(const camera_state& camera, const std::shared_ptr<const physx::render_snapshot>& physics);
    void set_topmost(bool enabled);
    void set_transparent(bool enabled);

private:
    void create_target();
    void rebuild_raycast_scene(const physx::render_snapshot& physics);
    void update_raycast(const camera_state& camera, float width, float height);
    bool update_raycast_geometry(const physx::render_snapshot& physics);
    HWND m_window{};
    ID3D11Device* m_device{};
    ID3D11DeviceContext* m_context{};
    IDXGISwapChain* m_swap_chain{};
    ID3D11RenderTargetView* m_target{};
    float m_range{25.0f};
    int m_triangle_budget{200000};
    float m_line_thickness{1.0f};
    bool m_enabled{true};
    std::array<bool, 7> m_types{true, false, true, true, true, true, true};
    bool m_raycast_enabled{true};
    bool m_vsync{true};
    bool m_topmost{};
    bool m_transparent{};
    float m_raycast_distance{100.0f};
    bool m_last_raycast_hit{};
    float m_last_raycast_hit_distance{};
    vec3 m_last_raycast_endpoint{};
    vec3 m_last_raycast_normal{};
    std::shared_ptr<const physx::render_snapshot> m_raycast_snapshot{};
    std::uint64_t m_raycast_topology_generation{};
    std::chrono::steady_clock::time_point m_last_raycast_geometry_update{};
#if defined(_WIN64)
    struct raycast_geometry
    {
        RTCGeometry geometry{};
        std::uint32_t geometry_id{RTC_INVALID_GEOMETRY_ID};
        std::vector<std::uint32_t> source_indices{};
    };
    RTCDevice m_embree_device{};
    RTCScene m_embree_scene{};
    std::unordered_map<std::uint64_t, raycast_geometry> m_raycast_geometries{};
#endif
};
