#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

class render_window
{
public:
    render_window();
    ~render_window();
    bool process_messages() const;
    HWND handle() const
    {
        return m_handle;
    }

private:
    static LRESULT CALLBACK procedure(HWND window, UINT message, WPARAM wparam, LPARAM lparam);
    HINSTANCE m_instance{};
    HWND m_handle{};
};
