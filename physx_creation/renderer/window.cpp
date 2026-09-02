#include "window.h"

#include "../dependencies/imgui/imgui_impl_win32.h"

#include <stdexcept>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

render_window::render_window() : m_instance(GetModuleHandleW(nullptr))
{
    WNDCLASSEXW type{};
    type.cbSize = sizeof(type);
    type.style = CS_HREDRAW | CS_VREDRAW;
    type.lpfnWndProc = procedure;
    type.hInstance = m_instance;
    type.hCursor = LoadCursor(nullptr, IDC_ARROW);
    type.lpszClassName = L"PhysXVisualizerWindow";
    if (!RegisterClassExW(&type))
    {
        throw std::runtime_error("Unable to register renderer window.");
    }

    const int width = GetSystemMetrics(SM_CXSCREEN);
    const int height = GetSystemMetrics(SM_CYSCREEN);
    m_handle = CreateWindowExW(0, type.lpszClassName, L"PhysX Scene Visualizer", WS_POPUP | WS_VISIBLE,
                               0, 0, width, height, nullptr, nullptr, m_instance, nullptr);
    if (!m_handle)
    {
        throw std::runtime_error("Unable to create renderer window.");
    }
    ShowWindow(m_handle, SW_SHOW);
    UpdateWindow(m_handle);
}

render_window::~render_window()
{
    if (m_handle) DestroyWindow(m_handle);
    if (m_instance) UnregisterClassW(L"PhysXVisualizerWindow", m_instance);
}

bool render_window::process_messages() const
{
    MSG message{};
    while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
    {
        if (message.message == WM_QUIT)
        {
            return false;
        }
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return true;
}

LRESULT CALLBACK render_window::procedure(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
{
    if (ImGui_ImplWin32_WndProcHandler(window, message, wparam, lparam))
    {
        return 1;
    }

    if (message == WM_DESTROY)
    {
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(window, message, wparam, lparam);
}
