#include "guard.h"
#include <cstring>
#include "../string.h"

namespace
{

    using NtQueryInformationProcess_t = long (WINAPI*)(HANDLE, ULONG, PVOID, ULONG, PULONG);
    using NtSetInformationThread_t    = long (WINAPI*)(HANDLE, ULONG, PVOID, ULONG);

    // Resolved once at thread startup. Resolved via GetProcAddress so the
    // import table has no plaintext "NtQueryInformationProcess" string.
    struct api
    {
        NtQueryInformationProcess_t NtQueryInformationProcess;
        NtSetInformationThread_t    NtSetInformationThread;
        FARPROC                      IsDebuggerPresent;
        FARPROC                      CheckRemoteDebuggerPresent;
    };

    // Stash decoded function-name bytes on the stack, resolve, then wipe.
    template <unsigned N>
    FARPROC resolve(HMODULE m, const char (&enc)[N], char key)
    {
        char buf[N];
        for (unsigned i = 0; i < N; ++i) buf[i] = static_cast<char>(enc[i] ^ key);
        FARPROC p = GetProcAddress(m, buf);
        SecureZeroMemory(buf, sizeof buf);
        return p;
    }

    static void resolve_all(api& a)
    {
        HMODULE nt = GetModuleHandleA(_("ntdll.dll"));
        HMODULE k  = GetModuleHandleA(_("kernel32.dll"));
        a.NtQueryInformationProcess = nullptr;
        a.NtSetInformationThread    = nullptr;
        a.IsDebuggerPresent         = nullptr;
        a.CheckRemoteDebuggerPresent = nullptr;
        if (nt)
        {
            // "NtQueryInformationProcess" ^ 0x71
            const char qip[] = { 0x27,0x14,0x14,0x07,0x3A,0x12,0x30,0x2A,0x12,0x21,0x3B,0x3A,0x21,0x32,0x12,0x3B,0x21,0x10,0x21,0x32,0x3F,0x21,0x32,0x32,0x3F,0x00 };
            a.NtQueryInformationProcess = reinterpret_cast<NtQueryInformationProcess_t>(resolve(nt, qip, 0x71));
            // "NtSetInformationThread" ^ 0x17
            const char sit[] = { 0x41,0x73,0x6B,0x76,0x5A,0x5A,0x64,0x76,0x52,0x5A,0x6D,0x5A,0x73,0x5A,0x52,0x52,0x64,0x73,0x53,0x76,0x60,0x00 };
            a.NtSetInformationThread = reinterpret_cast<NtSetInformationThread_t>(resolve(nt, sit, 0x17));
        }
        if (k)
        {
            // "IsDebuggerPresent" ^ 0x4A
            const char idp[] = { 0x23,0x35,0x26,0x32,0x33,0x2A,0x2A,0x32,0x39,0x29,0x24,0x12,0x39,0x32,0x35,0x33,0x5A,0x00 };
            a.IsDebuggerPresent = resolve(k, idp, 0x4A);
            // "CheckRemoteDebuggerPresent" ^ 0x2D
            const char crdp[] = { 0x46,0x2D,0x25,0x25,0x3B,0x4E,0x49,0x3D,0x49,0x35,0x49,0x42,0x35,0x25,0x25,0x4C,0x3B,0x25,0x42,0x42,0x49,0x3D,0x29,0x40,0x3B,0x5C,0x00 };
            a.CheckRemoteDebuggerPresent = resolve(k, crdp, 0x2D);
        }
    }

    static std::uint8_t* peb()
    {
        return reinterpret_cast<std::uint8_t*>(__readgsqword(0x60));
    }

    static int pick_response()
    {
        std::uint8_t* p = peb();
        if (!p)
        {
            return 2;
        }
        std::uintptr_t h = *reinterpret_cast<std::uintptr_t*>(p + 0x18);   // ProcessHeap
        std::uintptr_t f = *reinterpret_cast<std::uint32_t*>(p + 0xBC);    // NtGlobalFlag
        std::uintptr_t b = p[0x02];                                        // BeingDebugged
        std::uintptr_t mix = h ^ (f << 1) ^ (b << 3) ^ __rdtsc();
        return static_cast<int>(mix & 3) % 3;  // 0..2
    }
}

guard::guard(std::atomic<bool>& application_running) : application_running(application_running), thread(nullptr), thread_id(0)
{
    active.store(true, std::memory_order_release);
    thread = CreateThread(nullptr, 0, &guard::worker, this, 0, &thread_id);
}

guard::~guard()
{
    active.store(false, std::memory_order_release);
}

DWORD WINAPI guard::worker(void* self)
{
    auto* g = static_cast<guard*>(self);

    // Hide this thread from any attached debugger.
    api a{};
    resolve_all(a);
    if (a.NtSetInformationThread)
        a.NtSetInformationThread(GetCurrentThread(), 0x11 /* ThreadHideFromDebugger */, nullptr, 0);

    // Lower priority so we never perturb the real workload.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);

    // Locate the image's .text once -- used for both hashing and corruption.
    HMODULE mod = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&g->worker), &mod);
    std::uint8_t* image = reinterpret_cast<std::uint8_t*>(mod);
    std::uint8_t* text  = nullptr;
    std::uint32_t  textSize = 0;
    if (image && image[0] == 'M' && image[1] == 'Z')
    {
        std::uint32_t pe = *reinterpret_cast<std::uint32_t*>(image + 0x3C);
        if (image[pe] == 'P' && image[pe+1] == 'E' && image[pe+2] == 0 && image[pe+3] == 0)
        {
            std::uint16_t n   = *reinterpret_cast<std::uint16_t*>(image + pe + 6);
            std::uint16_t osz = *reinterpret_cast<std::uint16_t*>(image + pe + 20);
            std::uint32_t tbl = pe + 24 + osz;
            for (std::uint16_t i = 0; i < n; ++i)
            {
                auto* sh = reinterpret_cast<IMAGE_SECTION_HEADER*>(image + tbl + 40 * i);
                if (sh->Name[0] == '.' && sh->Name[1] == 't' && sh->Name[2] == 'e' &&
                    sh->Name[3] == 'x' && sh->Name[4] == 't')
                {
                    text     = image + sh->VirtualAddress;
                    textSize = sh->Misc.VirtualSize > sh->SizeOfRawData
                               ? sh->Misc.VirtualSize : sh->SizeOfRawData;
                    break;
                }
            }
        }
    }

    // Baseline CRC32 of .text so we can detect in-place patching later.
    auto crc32 = [](const std::uint8_t* p, std::uint32_t n) -> std::uint32_t
    {
        std::uint32_t t[256];
        for (std::uint32_t i = 0; i < 256; ++i)
        {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k)
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            t[i] = c;
        }
        std::uint32_t crc = 0xFFFFFFFFu;
        for (std::uint32_t i = 0; i < n; ++i)
            crc = t[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
        return ~crc;
    };
    std::uint32_t baseline = text ? crc32(text, textSize) : 0;

    Sleep(5000);
    if (text) baseline = crc32(text, textSize);

    // --- detection helpers ---
    auto is_debugger_present = [&]() -> bool
    {
        if (a.IsDebuggerPresent && reinterpret_cast<BOOL (WINAPI*)()>(a.IsDebuggerPresent)())
            return true;
        if (a.CheckRemoteDebuggerPresent)
        {
            BOOL present = FALSE;
            if (reinterpret_cast<BOOL (WINAPI*)(HANDLE, PBOOL)>(a.CheckRemoteDebuggerPresent)(GetCurrentProcess(), &present) && present)
                return true;
        }
        std::uint8_t* p = peb();
        if (p && p[0x02])
        {
            return true;
        }

        if (a.NtQueryInformationProcess)
        {
            ULONG ret = 0;
            std::uintptr_t port = 0;
            if (a.NtQueryInformationProcess(GetCurrentProcess(), 7 /* ProcessDebugPort */, &port, sizeof(port), &ret) >= 0 && port)
                return true;
            std::uint32_t flags = 1;
            if (a.NtQueryInformationProcess(GetCurrentProcess(), 0x1F /* ProcessDebugFlags */, &flags, sizeof(flags), &ret) >= 0 && flags == 0)
                return true;
            HANDLE h = nullptr;
            if (a.NtQueryInformationProcess(GetCurrentProcess(), 0x1E /* ProcessDebugObjectHandle */, &h, sizeof(h), &ret) >= 0 && h)
            { CloseHandle(h); return true; }
        }
        return false;
    };

    auto text_tampered = [&]() -> bool
    {
        if (!text)
        {
            return false;
        }
        return crc32(text, textSize) != baseline;
    };

    auto text_page_writable = [&]() -> bool
    {
        if (!text)
        {
            return false;
        }
        MEMORY_BASIC_INFORMATION mbi{};
        if (!VirtualQuery(text, &mbi, sizeof(mbi)))
        {
            return false;
        }
        constexpr DWORD W =
            PAGE_READWRITE | PAGE_WRITECOPY |
            PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
        return (mbi.Protect & W) != 0;
    };

    // Respond. Choice is fixed at the moment of detection -- different each
    // start because pick_response() pulls PEB entropy.
    auto respond = [&]() -> void
    {
        int mode = pick_response();
        switch (mode)
        {
        case 0:
            // Corrupt memory: scramble .text + first heap region. Aim is to
            // leave the process in a useless state without telling the
            // debugger anything is wrong first.
            if (text && textSize)
            {
                DWORD oldp = 0;
                if (VirtualProtect(text, textSize, PAGE_EXECUTE_READWRITE, &oldp))
                {
                    std::memset(text, 0xCC /* int3 */, textSize);
                    FlushInstructionCache(GetCurrentProcess(), text, textSize);
                }
            }
            {
                std::uint8_t* p = peb();
                if (p)
                {
                    void* heap = *reinterpret_cast<void**>(p + 0x18);
                    if (heap)
                        std::memset(heap, 0x55, 0x1000);
                }
            }
            TerminateProcess(GetCurrentProcess(), 0);
            break;
        case 1:
            // Set application_running=false; main loop exits on its own and
            // looks like a clean shutdown. No debugger ever gets a clue.
            g->application_running.store(false, std::memory_order_release);
            break;
        case 2:
        default:
            // Kill. Generic-looking status code that no one will wonder about.
            TerminateProcess(GetCurrentProcess(), 0xC0000135u /* STATUS_DLL_NOT_FOUND */);
            break;
        }
    };

    std::uint32_t text_tamper_streak = 0;
    std::uint32_t text_page_streak   = 0;
    while (g->active.load(std::memory_order_acquire))
    {
        bool tripped = false;

        if (is_debugger_present()) tripped = true;

        if (text_tampered())
        {
            if (++text_tamper_streak >= 2) tripped = true;
        }
        else text_tamper_streak = 0;

        if (text_page_writable())
        {
            if (++text_page_streak >= 2) tripped = true;
        }
        else text_page_streak = 0;

        if (tripped)
        {
            respond();
            return 0;
        }

        Sleep(2000 + (GetTickCount() & 0x7FF));  // ~2.0 - 2.5 s, jittered cheaply
    }
    return 0;
}
