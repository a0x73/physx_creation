#include "renderer.h"

#include "../dependencies/imgui/imgui.h"
#include "../dependencies/imgui/imgui_impl_dx11.h"
#include "../dependencies/imgui/imgui_impl_win32.h"
#include "../dependencies/fonts/visitor.h"

#include <algorithm>
#include <cmath>
#include <d3d11.h>
#include <stdexcept>
#include <cstdio>
#include <unordered_set>
#include <tbb/parallel_for.h>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "tbb.lib")

namespace
{
constexpr float cell_size = 10.0f;
constexpr std::uint64_t coordinate_mask = (1ull << 21u) - 1u;
constexpr const char* type_names[] = {"Sphere", "Plane", "Capsule", "Box", "Convex", "Triangle mesh", "Heightfield"};
constexpr ImU32 type_colours[] = {IM_COL32(255, 120, 120, 220), IM_COL32(180, 180, 180, 220),
                                  IM_COL32(255, 180, 80, 220), IM_COL32(80, 225, 135, 220),
                                  IM_COL32(75, 170, 255, 220), IM_COL32(195, 100, 255, 210),
                                  IM_COL32(255, 225, 75, 220)};
std::uint64_t cell_key(const int x, const int y, const int z)
{
    return (static_cast<std::uint64_t>(x) & coordinate_mask) |
           ((static_cast<std::uint64_t>(y) & coordinate_mask) << 21u) |
           ((static_cast<std::uint64_t>(z) & coordinate_mask) << 42u);
}
int coordinate(const float value)
{
    return static_cast<int>(std::floor(value / cell_size));
}
template <typename type>
void release(type*& value)
{
    if (value)
    {
        value->Release();
        value = nullptr;
    }
}
}

renderer::renderer(HWND window) : m_window(window)
{
    DXGI_SWAP_CHAIN_DESC description{};
    description.BufferCount = 2;
    description.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    description.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    description.OutputWindow = window;
    description.SampleDesc.Count = 1;
    description.Windowed = TRUE;
    if (FAILED(D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0,
                                              D3D11_SDK_VERSION, &description, &m_swap_chain, &m_device, nullptr,
                                              &m_context)))
        throw std::runtime_error("Unable to create D3D11 renderer.");
    create_target();
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.Fonts->AddFontFromMemoryCompressedTTF(visitor_tt2brk, visitor_tt2brk_size, 9.0f);
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 8.0f;
    ImGui::GetStyle().FrameRounding = 5.0f;
    ImGui_ImplWin32_Init(window);
    ImGui_ImplDX11_Init(m_device, m_context);
#if defined(_WIN64)
    m_embree_device = rtcNewDevice(nullptr);
    if (!m_embree_device) throw std::runtime_error("Unable to create Embree device.");
#endif
}

renderer::~renderer()
{
#if defined(_WIN64)
    if (m_embree_scene) rtcReleaseScene(m_embree_scene);
    if (m_embree_device) rtcReleaseDevice(m_embree_device);
#endif
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    release(m_target); release(m_swap_chain); release(m_context); release(m_device);
}

void renderer::create_target()
{
    ID3D11Texture2D* buffer{};
    if (FAILED(m_swap_chain->GetBuffer(0, IID_PPV_ARGS(&buffer)))) throw std::runtime_error("No back buffer.");
    const HRESULT result = m_device->CreateRenderTargetView(buffer, nullptr, &m_target);
    release(buffer);
    if (FAILED(result)) throw std::runtime_error("No render target.");
}

void renderer::render_frame(const camera_state& camera, const std::shared_ptr<const physx::render_snapshot>& physics)
{
    RECT client{}; GetClientRect(m_window, &client);
    const float width = static_cast<float>(client.right - client.left);
    const float height = static_cast<float>(client.bottom - client.top);
    ImGui_ImplDX11_NewFrame(); ImGui_ImplWin32_NewFrame(); ImGui::NewFrame();

    ImGui::SetNextWindowPos({18.0f, 18.0f}, ImGuiCond_Once);
    ImGui::SetNextWindowSize({280.0f, 0.0f}, ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(m_transparent ? 0.0f : 0.92f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(9, 13, 22, 235));
    ImGui::PushStyleColor(ImGuiCol_Border, IM_COL32(54, 82, 120, 220));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {12.0f, 10.0f});
    ImGui::Begin("PHYSX // DEBUG", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::TextColored({0.35f, 0.85f, 1.0f, 1.0f}, "RUNTIME TELEMETRY");
    ImGui::SameLine(ImGui::GetWindowWidth() - 62.0f);
    ImGui::Text("%.0f FPS", ImGui::GetIO().Framerate);
    ImGui::Separator();
    ImGui::Text("camera");
    ImGui::SameLine(112.0f);
    ImGui::TextColored(camera.projection_valid ? ImVec4{0.3f, 1.0f, 0.55f, 1.0f}
                                                : ImVec4{1.0f, 0.55f, 0.25f, 1.0f},
                     camera.projection_valid ? "ONLINE" : "SEARCHING");
    ImGui::Text("scene");
    ImGui::SameLine(112.0f);
    ImGui::Text("%zu triangles", physics ? physics->triangles.size() : 0u);
    ImGui::Text("render");
    ImGui::SameLine(112.0f);
    ImGui::Text("raycast overlay");
    ImGui::Checkbox("vsync", &m_vsync);
    if (ImGui::Checkbox("topmost", &m_topmost))
    {
        set_topmost(m_topmost);
    }

    if (ImGui::Checkbox("transparent background", &m_transparent))
    {
        set_transparent(m_transparent);
    }

    ImGui::Checkbox("enable rendering", &m_enabled);
    ImGui::SliderFloat("render radius", &m_range, 2.0f, 150.0f, "%.0f m");
    ImGui::SliderInt("max triangles", &m_triangle_budget, 1000, 1000000, "%d");
    ImGui::SliderFloat("line width", &m_line_thickness, 0.5f, 3.0f, "%.1f");
    ImGui::Separator();
    ImGui::TextColored({0.55f, 0.6f, 0.72f, 1.0f}, "RAYCAST RESULT");
    if (m_raycast_enabled)
    {
        ImGui::TextColored(m_last_raycast_hit ? ImVec4{0.3f, 1.0f, 0.55f, 1.0f}
                                               : ImVec4{1.0f, 0.65f, 0.25f, 1.0f},
                          m_last_raycast_hit ? "VISIBLE // HIT" : "CLEAR // MISS");
        ImGui::Text("distance  %.2f m", m_last_raycast_hit_distance);
        ImGui::Text("endpoint  %.1f  %.1f  %.1f", m_last_raycast_endpoint.x,
                     m_last_raycast_endpoint.y, m_last_raycast_endpoint.z);
    }
    if (camera.projection_valid && camera.position_valid && physics)
    {
        if (physics != m_raycast_snapshot)
        {
            const bool topology_changed = !m_embree_scene ||
                                           physics->topology_generation != m_raycast_topology_generation;
            const bool update_due = m_last_raycast_geometry_update.time_since_epoch().count() == 0 ||
                                    std::chrono::steady_clock::now() - m_last_raycast_geometry_update >=
                                        std::chrono::milliseconds(16);
            bool geometry_applied = false;
            if (topology_changed)
            {
                rebuild_raycast_scene(*physics);
                m_last_raycast_geometry_update = std::chrono::steady_clock::now();
                geometry_applied = true;
            }
            else if (update_due)
            {
                if (!update_raycast_geometry(*physics))
                    rebuild_raycast_scene(*physics);
                m_last_raycast_geometry_update = std::chrono::steady_clock::now();
                geometry_applied = true;
            }
            if (geometry_applied)
            {
                m_raycast_snapshot = physics;
                m_raycast_topology_generation = physics->topology_generation;
            }
        }
        update_raycast(camera, width, height);
    }
    if (m_enabled && camera.projection_valid && camera.position_valid && physics)
    {
        std::size_t processed = 0;
        std::size_t drawn = 0;
        const float range_squared = m_range * m_range;
        const int radius = static_cast<int>(std::ceil(m_range / cell_size));
        const int cx = coordinate(camera.position.x);
        const int cy = coordinate(camera.position.y);
        const int cz = coordinate(camera.position.z);
        ImDrawList* background = ImGui::GetBackgroundDrawList();
        for (int z = cz - radius; z <= cz + radius && processed < static_cast<std::size_t>(m_triangle_budget); ++z)
        for (int y = cy - radius; y <= cy + radius && processed < static_cast<std::size_t>(m_triangle_budget); ++y)
        for (int x = cx - radius; x <= cx + radius && processed < static_cast<std::size_t>(m_triangle_budget); ++x)
        {
            const auto cell = physics->spatial_cells.find(cell_key(x, y, z));
            if (cell == physics->spatial_cells.end())
            {
                continue;
            }
            for (const std::uint32_t index : cell->second)
            {
                if (index >= physics->triangles.size() || processed++ >= static_cast<std::size_t>(m_triangle_budget))
                {
                    break;
                }
                const triangle& value = physics->triangles[index];
                const std::size_t type = static_cast<std::size_t>(value.geometry_type);
                if (type >= m_types.size() || !m_types[type])
                {
                    continue;
                }
                const vec3 center = triangle_center(value);
                const float dx = center.x - camera.position.x;
                const float dy = center.y - camera.position.y;
                const float dz = center.z - camera.position.z;
                if (dx * dx + dy * dy + dz * dz > range_squared)
                {
                    continue;
                }
                float x0{}, y0{}, x1{}, y1{}, x2{}, y2{};
                const bool p0 = camera.world_to_screen(value.v0, width, height, x0, y0);
                const bool p1 = camera.world_to_screen(value.v1, width, height, x1, y1);
                const bool p2 = camera.world_to_screen(value.v2, width, height, x2, y2);
                if (p0 && p1)
                {
                    background->AddLine({x0, y0}, {x1, y1}, type_colours[type], m_line_thickness);
                }
                if (p1 && p2)
                {
                    background->AddLine({x1, y1}, {x2, y2}, type_colours[type], m_line_thickness);
                }
                if (p2 && p0)
                {
                    background->AddLine({x2, y2}, {x0, y0}, type_colours[type], m_line_thickness);
                }
                drawn += p0 || p1 || p2;
            }
        }
        ImGui::Text("Frame: %zu processed | %zu drawn", processed, drawn);
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);

    ImGui::Render();
    const float clear[4]{0.0f, 0.0f, 0.0f, 1.0f};
    m_context->OMSetRenderTargets(1, &m_target, nullptr);
    m_context->ClearRenderTargetView(m_target, clear);
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
    m_swap_chain->Present(m_vsync ? 1u : 0u, 0);
}

void renderer::set_topmost(bool enabled)
{
    if (!m_window)
    {
        return;
    }

    SetWindowPos(m_window, enabled ? HWND_TOPMOST : HWND_NOTOPMOST, 0, 0, 0, 0,
                 SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE | SWP_SHOWWINDOW);
}

void renderer::set_transparent(bool enabled)
{
    if (!m_window)
    {
        return;
    }

    const LONG_PTR style = GetWindowLongPtrW(m_window, GWL_EXSTYLE);
    const LONG_PTR updated_style = enabled ? style | WS_EX_LAYERED : style & ~WS_EX_LAYERED;
    SetWindowLongPtrW(m_window, GWL_EXSTYLE, updated_style);

    if (enabled)
    {
        SetLayeredWindowAttributes(m_window, RGB(0, 0, 0), 0, LWA_COLORKEY);
    }
    else
    {
        SetLayeredWindowAttributes(m_window, 0, 255, LWA_ALPHA);
    }

    SetWindowPos(m_window, nullptr, 0, 0, 0, 0,
                 SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE | SWP_FRAMECHANGED);
}

void renderer::rebuild_raycast_scene(const physx::render_snapshot& physics)
{
#if defined(_WIN64)
    if (!m_embree_device)
    {
        return;
    }
    if (m_embree_scene) rtcReleaseScene(m_embree_scene);
    m_raycast_geometries.clear();
    m_embree_scene = rtcNewScene(m_embree_device);
    rtcSetSceneBuildQuality(m_embree_scene, RTC_BUILD_QUALITY_MEDIUM);
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> groups;
    for (std::uint32_t index = 0; index < physics.triangles.size(); ++index)
    {
        const auto type = static_cast<std::size_t>(physics.triangles[index].geometry_type);
        if (type < 7u)
            groups[(physics.triangles[index].actor_address << 3u) | type].push_back(index);
    }
    for (const auto& [key, source_indices] : groups)
    {
        const std::size_t type = static_cast<std::size_t>(key & 7u);
        RTCGeometry geometry = rtcNewGeometry(m_embree_device, RTC_GEOMETRY_TYPE_TRIANGLE);
        auto* vertices = static_cast<float*>(rtcSetNewGeometryBuffer(
            geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float) * 3u, source_indices.size() * 3u));
        auto* indices = static_cast<std::uint32_t*>(rtcSetNewGeometryBuffer(
            geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(std::uint32_t) * 3u, source_indices.size()));
        if (vertices && indices)
        {
            for (std::size_t i = 0; i < source_indices.size(); ++i)
            {
                const auto& value = physics.triangles[source_indices[i]];
                const vec3 points[3]{value.v0, value.v1, value.v2};
                for (std::size_t vertex = 0; vertex < 3; ++vertex)
                {
                    const std::size_t offset = (i * 3u + vertex) * 3u;
                    vertices[offset] = points[vertex].x;
                    vertices[offset + 1u] = points[vertex].y;
                    vertices[offset + 2u] = points[vertex].z;
                }
                indices[i * 3u] = static_cast<std::uint32_t>(i * 3u);
                indices[i * 3u + 1u] = static_cast<std::uint32_t>(i * 3u + 1u);
                indices[i * 3u + 2u] = static_cast<std::uint32_t>(i * 3u + 2u);
            }
            rtcSetGeometryMask(geometry, 1u << static_cast<unsigned int>(type));
            rtcCommitGeometry(geometry);
            raycast_geometry value{};
            value.geometry = geometry;
            value.geometry_id = rtcAttachGeometry(m_embree_scene, geometry);
            value.source_indices = source_indices;
            m_raycast_geometries.emplace(key, std::move(value));
        }
        rtcReleaseGeometry(geometry);
    }
    rtcCommitScene(m_embree_scene);
#else
    (void)physics;
#endif
}

bool renderer::update_raycast_geometry(const physx::render_snapshot& physics)
{
#if defined(_WIN64)
    if (!m_embree_scene || m_raycast_geometries.empty())
    {
        return false;
    }

    if (physics.updated_dynamic_actors.empty())
    {
        return true;
    }
    std::unordered_set<std::uint64_t> updated_actors(physics.updated_dynamic_actors.begin(),
                                                      physics.updated_dynamic_actors.end());
    std::vector<raycast_geometry*> geometry;
    for (auto& [key, value] : m_raycast_geometries)
    {
        // Actor address zero is the static/untagged group. Its vertices never
        // change during a transform-only snapshot update.
        const std::uint64_t actor_address = key >> 3u;
        if (actor_address == 0u || !updated_actors.contains(actor_address))
        {
            continue;
        }
        geometry.push_back(&value);
    }
    tbb::parallel_for(std::size_t{0}, geometry.size(), [&](const std::size_t i)
    {
        auto& value = *geometry[i];
        auto* vertices = static_cast<float*>(rtcGetGeometryBufferData(value.geometry, RTC_BUFFER_TYPE_VERTEX, 0));
        if (!vertices)
        {
            return;
        }
        for (std::size_t triangle = 0; triangle < value.source_indices.size(); ++triangle)
        {
            const auto& source = physics.triangles[value.source_indices[triangle]];
            const vec3 points[3]{source.v0, source.v1, source.v2};
            for (std::size_t vertex = 0; vertex < 3; ++vertex)
            {
                const std::size_t offset = (triangle * 3u + vertex) * 3u;
                vertices[offset] = points[vertex].x;
                vertices[offset + 1u] = points[vertex].y;
                vertices[offset + 2u] = points[vertex].z;
            }
        }
        rtcUpdateGeometryBuffer(value.geometry, RTC_BUFFER_TYPE_VERTEX, 0);
        rtcCommitGeometry(value.geometry);
    });
    rtcCommitScene(m_embree_scene);
    return true;
#else
    (void)physics;
    return false;
#endif
}

void renderer::update_raycast(const camera_state& camera, const float width, const float height)
{
#if defined(_WIN64)
    m_last_raycast_hit = false;
    m_last_raycast_hit_distance = m_raycast_distance;
    m_last_raycast_endpoint = camera.position + camera.forward * m_raycast_distance;
    m_last_raycast_normal = {};
    if (!m_embree_scene)
    {
        return;
    }
    RTCRayHit ray_hit{};
    ray_hit.ray.org_x = camera.position.x; ray_hit.ray.org_y = camera.position.y; ray_hit.ray.org_z = camera.position.z;
    ray_hit.ray.dir_x = camera.forward.x; ray_hit.ray.dir_y = camera.forward.y; ray_hit.ray.dir_z = camera.forward.z;
    ray_hit.ray.tnear = 0.01f; ray_hit.ray.tfar = m_raycast_distance; ray_hit.ray.mask = 0xffffffffu;
    ray_hit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    RTCIntersectArguments arguments{}; rtcInitIntersectArguments(&arguments);
    rtcIntersect1(m_embree_scene, &ray_hit, &arguments);
    const bool hit = ray_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
    m_last_raycast_hit = hit;
    m_last_raycast_hit_distance = ray_hit.ray.tfar;
    m_last_raycast_endpoint = camera.position + camera.forward * ray_hit.ray.tfar;
    if (hit)
    {
        m_last_raycast_normal = {ray_hit.hit.Ng_x, ray_hit.hit.Ng_y, ray_hit.hit.Ng_z};
    }
    const ImU32 colour = hit ? IM_COL32(255, 75, 75, 255) : IM_COL32(255, 215, 70, 255);
    auto* draw = ImGui::GetBackgroundDrawList();
    const ImVec2 center(width * 0.5f, height * 0.5f);
    // A center-screen ray always lands at the screen center. Drawing the
    // marker directly avoids losing a valid hit when projection rejects the
    // endpoint because of floating-point or matrix-convention differences.
    draw->AddLine({center.x - 9.0f, center.y}, {center.x + 9.0f, center.y}, colour, 2.5f);
    draw->AddLine({center.x, center.y - 9.0f}, {center.x, center.y + 9.0f}, colour, 2.5f);

    float endpoint_x{}, endpoint_y{};
    if (camera.world_to_screen(m_last_raycast_endpoint, width, height, endpoint_x, endpoint_y))
    {
        const ImVec2 endpoint{endpoint_x, endpoint_y};
        draw->AddCircleFilled(endpoint, hit ? 4.0f : 2.5f, colour, 12);
        draw->AddCircle(endpoint, hit ? 9.0f : 5.0f, colour, 16, 2.0f);
        draw->AddLine({endpoint_x - 10.0f, endpoint_y}, {endpoint_x + 10.0f, endpoint_y}, colour, 2.0f);
        draw->AddLine({endpoint_x, endpoint_y - 10.0f}, {endpoint_x, endpoint_y + 10.0f}, colour, 2.0f);
        if (hit)
        {
            const vec3 normal_end = m_last_raycast_endpoint + m_last_raycast_normal * 0.5f;
            float normal_x{}, normal_y{};
            if (camera.world_to_screen(normal_end, width, height, normal_x, normal_y))
                draw->AddLine(endpoint, {normal_x, normal_y}, IM_COL32(100, 200, 255, 255), 2.0f);
        }
    }
#else
    (void)camera; (void)width; (void)height;
#endif
}
