#define NOMINMAX
#include "memory.h"
#include <algorithm>
#include <limits>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <unordered_set>
#include "../dependencies/logging/logging.h"

static std::vector<std::string> s_blacklist =
{
    "kernel32.dll", "kernelbase.dll", "wow64.dll", "wow64win.dll",
    "wow64cpu.dll", "ntoskrnl.exe", "win32kbase.sys", "BEClient_x64.dll", "DUser.dll"
};

static std::wstring widen_process_name(const std::string& name)
{
    if (name.empty())
    {
        return {};
    }

    const int length = MultiByteToWideChar(CP_UTF8, 0, name.data(), static_cast<int>(name.size()), nullptr, 0);
    if (length <= 0)
    {
        return {};
    }

    std::wstring result(static_cast<std::size_t>(length), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, name.data(), static_cast<int>(name.size()), result.data(), length);
    return result;
}

static std::string narrow_process_name(const wchar_t* name)
{
    if (!name || !*name)
    {
        return {};
    }

    const int length = WideCharToMultiByte(CP_UTF8, 0, name, -1, nullptr, 0, nullptr, nullptr);
    if (length <= 1)
    {
        return {};
    }

    std::string result(static_cast<std::size_t>(length), '\0');
    WideCharToMultiByte(CP_UTF8, 0, name, -1, result.data(), length, nullptr, nullptr);
    result.resize(static_cast<std::size_t>(length - 1));
    return result;
}

static std::size_t find_pattern_offset(const std::uint8_t* data, std::size_t size, const std::vector<memory::pattern_byte>& pattern)
{
    if (!data || pattern.empty() || pattern.size() > size)
    {
        return SIZE_MAX;
    }

    const std::size_t limit = size - pattern.size();
    for (std::size_t i = 0; i <= limit; ++i)
    {
        bool found = true;
        for (std::size_t j = 0; j < pattern.size(); ++j)
        {
            if (!pattern[j].wildcard && data[i + j] != pattern[j].value)
            {
                found = false;
                break;
            }
        }
        if (found)
        {
            return i;
        }
    }
    return SIZE_MAX;
}

memory::memory()
{
#if MEMORY_USE_WINDOWS_RPM
    LOG_INFO("[memory] Windows ReadProcessMemory backend enabled");
#endif
#if !MEMORY_USE_WINDOWS_RPM
    modules.vmm = LoadLibraryA("vmm.dll");
    modules.ftd3xx = LoadLibraryA("FTD3XX.dll");
    modules.leechcore = LoadLibraryA("leechcore.dll");
#endif
}

memory::~memory()
{
    initialised = false;
#if MEMORY_USE_WINDOWS_RPM
    if (native_process_handle)
    {
        CloseHandle(native_process_handle);
        native_process_handle = nullptr;
    }
#else
    if (fpga_handle)
    {
        VMMDLL_Close(fpga_handle);
        fpga_handle = nullptr;
    }
#endif
}

bool memory::load(std::string process_name, bool use_mem_map)
{
    if (initialised)
    {
        return true;
    }

    const bool requested_mem_map = use_mem_map;
    LOG_INFO("[memory] initializing acquisition process=%s physical_map_requested=%s", process_name.c_str(), requested_mem_map ? "enabled" : "disabled");

    std::string mem_map_path;

    while (true)
    {
        std::vector<std::string> owned_arguments{ "", "-device", "fpga://algo=0", "-waitinitialize" };
        std::vector<std::string> pagefile_arguments;
        pagefile_arguments.reserve(2);

        char windows_directory[MAX_PATH]{};
        const DWORD windows_directory_length = GetWindowsDirectoryA(windows_directory, MAX_PATH);
        if (windows_directory_length && windows_directory_length < MAX_PATH)
        {
            const std::filesystem::path windows_path(windows_directory);
            const std::array<std::filesystem::path, 2> pagefiles =
            {
                windows_path.root_path() / "pagefile.sys",
                windows_path.root_path() / "swapfile.sys"
            };
            for (std::size_t index = 0; index < pagefiles.size(); ++index)
            {
                std::error_code error;
                if (!std::filesystem::is_regular_file(pagefiles[index], error))
                    continue;
                pagefile_arguments.push_back(pagefiles[index].string());
                owned_arguments.push_back("-pagefile" + std::to_string(index));
                owned_arguments.push_back(pagefile_arguments.back());
            }
        }

        if (use_mem_map)
        {
            mem_map_path = std::filesystem::temp_directory_path().string() + "\\mmap.txt";
            if (!dump_memory_map())
            {
                LOG_WARN("[memory] physical map generation failed; retrying acquisition without -memmap");
                use_mem_map = false;
            }

            if (use_mem_map)
            {
                owned_arguments.push_back("-memmap");
                owned_arguments.push_back(mem_map_path);
            }
        }

        std::vector<LPCSTR> args;
        args.reserve(owned_arguments.size());
        for (const std::string& argument : owned_arguments)
            args.push_back(argument.c_str());

        LOG_INFO("[memory] acquisition attempt physical_map_active=%s path=%s arguments=%zu",
                 use_mem_map ? "yes" : "no", use_mem_map ? mem_map_path.c_str() : "<none>", args.size());
        LOG_INFO("[memory] pagefile backing files=%zu", pagefile_arguments.size());

        PLC_CONFIG_ERRORINFO initialization_error = nullptr;
        fpga_handle = VMMDLL_InitializeEx(static_cast<DWORD>(args.size()), args.data(), &initialization_error);
        if (fpga_handle)
        {
            if (initialization_error)
            {
                VMMDLL_MemFree(initialization_error);
            }
            break;
        }

        if (initialization_error)
        {
            const std::wstring error_text(initialization_error->wszUserText,
                                          initialization_error->wszUserText + initialization_error->cwszUserText);
            std::string error_utf8;
            if (!error_text.empty())
            {
                const int length = WideCharToMultiByte(CP_UTF8, 0, error_text.c_str(),
                                                       static_cast<int>(error_text.size()), nullptr, 0, nullptr, nullptr);
                if (length > 0)
                {
                    error_utf8.resize(static_cast<std::size_t>(length));
                    WideCharToMultiByte(CP_UTF8, 0, error_text.c_str(), static_cast<int>(error_text.size()),
                                        error_utf8.data(), length, nullptr, nullptr);
                }
            }
            LOG_ERROR("[memory] FPGA initialization failed detail=%s", error_utf8.empty() ? "<none>" : error_utf8.c_str());
            VMMDLL_MemFree(initialization_error);
        }
        else
        {
            LOG_ERROR("[memory] FPGA initialization failed without diagnostic information");
        }

        if (!use_mem_map)
        {
            return false;
        }

        use_mem_map = false;
    }

    if (!set_fpga())
    {
        VMMDLL_Close(fpga_handle);
        fpga_handle = nullptr;
        return false;
    }

    SIZE_T pid_count = 0;
    if (!VMMDLL_PidList(fpga_handle, nullptr, &pid_count) || !pid_count)
        return false;

    std::vector<DWORD> pids(pid_count);
    if (!VMMDLL_PidList(fpga_handle, pids.data(), &pid_count))
        return false;

    DWORD selected_pid = 0;
    VMMDLL_PROCESS_INFORMATION selected_info{};
    std::uint64_t selected_base = 0;
    std::uint64_t selected_size = 0;

    for (SIZE_T i = 0; i < pid_count; ++i)
    {
        VMMDLL_PROCESS_INFORMATION info{};
        info.magic = VMMDLL_PROCESS_INFORMATION_MAGIC;
        info.wVersion = VMMDLL_PROCESS_INFORMATION_VERSION;
        SIZE_T info_size = sizeof(info);

        if (!VMMDLL_ProcessGetInformation(fpga_handle, pids[i], &info, &info_size))
            continue;

        if (_stricmp(info.szName, process_name.c_str()) != 0 &&
            _stricmp(info.szNameLong, process_name.c_str()) != 0)
            continue;

        if (info.dwState != 0 || !info.paDTB)
            continue;

        PVMMDLL_MAP_MODULEENTRY module = nullptr;
        if (!VMMDLL_Map_GetModuleFromNameU(fpga_handle, pids[i], const_cast<LPSTR>(process_name.c_str()),
            &module, VMMDLL_MODULE_FLAG_NORMAL) || !module)
            continue;

        const std::uint64_t module_base = module->vaBase;
        const std::uint64_t module_size = module->cbImageSize;
        VMMDLL_MemFree(module);
        if (!module_base || !module_size)
            continue;

        IMAGE_DOS_HEADER dos{};
        DWORD bytes_read = 0;
        if (!VMMDLL_MemReadEx(fpga_handle, pids[i], module_base, reinterpret_cast<PBYTE>(&dos), sizeof(dos),
            &bytes_read, VMMDLL_FLAG_NOCACHE) || bytes_read != sizeof(dos) ||
            dos.e_magic != IMAGE_DOS_SIGNATURE)
            continue;

        if (!selected_pid || pids[i] > selected_pid)
        {
            selected_pid = pids[i];
            selected_info = info;
            selected_base = module_base;
            selected_size = module_size;
        }
    }

    process_information.process_id = static_cast<std::int32_t>(selected_pid);
    if (!process_information.process_id)
    {
        return false;
    }
    process_information.name = process_name;
    process_information.base_address = selected_base;
    process_information.base_size = selected_size;
    LOG_INFO("[memory] read backend=VMM/FPGA");

    LPSTR image_path = VMMDLL_ProcessGetInformationString(fpga_handle, selected_pid, VMMDLL_PROCESS_INFORMATION_OPT_STRING_PATH_USER_IMAGE);
    LOG_DEBUG("pid:%lu state:%lu dtb:0x%llx session:%lu address:0x%llx size:0x%llx image:%s", selected_pid, selected_info.dwState, selected_info.paDTB, selected_info.win.dwSessionId, process_information.base_address, process_information.base_size, image_path ? image_path : "<unknown>");
    LOG_INFO("[memory] selected process pid=%lu dtb=0x%llx session=%lu module_base=0x%llx module_size=0x%llx pid_candidates=%llu",
             selected_pid, selected_info.paDTB, selected_info.win.dwSessionId, process_information.base_address,
             process_information.base_size, static_cast<unsigned long long>(pid_count));
    if (image_path)
    {
        VMMDLL_MemFree(image_path);
    }

    initialised = true;
    init_keyboard();
    return true;
}

bool memory::load_memory_into_vector(const std::string& module_name)
{
    if (!initialised)
    {
        return false;
    }

    std::uint64_t base = process_information.base_address;
    std::uint64_t size = process_information.base_size;
    if (!module_name.empty() && !get_module_information(module_name, base, size))
        return false;

    LOG_DEBUG("module=%s base=0x%llx size=0x%llx", module_name.c_str(), base, size);

    if (!base || !size || size > (std::numeric_limits<std::size_t>::max)())
        return false;

    constexpr std::size_t page_size = 0x1000;
    std::vector<std::uint8_t> buffer(static_cast<std::size_t>(size));
    std::vector<std::uint8_t> valid((buffer.size() + page_size - 1) / page_size);
    std::size_t readable_bytes = 0;

    for (std::size_t offset = 0; offset < buffer.size(); offset += page_size)
    {
        const std::size_t request_size = (std::min)(page_size, buffer.size() - offset);
        DWORD bytes_read = 0;
        bool read_ok = VMMDLL_MemReadEx(fpga_handle, process_information.process_id, base + offset,
            buffer.data() + offset, static_cast<DWORD>(request_size), &bytes_read,
            VMMDLL_FLAG_NOCACHE) && bytes_read == request_size;
        if (!read_ok)
        {
            bytes_read = 0;
            read_ok = VMMDLL_MemReadEx(fpga_handle, process_information.process_id, base + offset,
                buffer.data() + offset, static_cast<DWORD>(request_size), &bytes_read, 0) &&
                bytes_read == request_size;
        }
        if (read_ok)
        {
            valid[offset / page_size] = 1;
            readable_bytes += request_size;
        }
    }

    if (!readable_bytes)
    {
        return false;
    }

    process_bytes = std::move(buffer);
    process_bytes_valid = std::move(valid);
    process_bytes_base = base;
    LOG_DEBUG("loaded 0x%llx/0x%llx readable bytes with base 0x%llx", readable_bytes, size, process_bytes_base);
    return true;
}

std::vector<std::string> memory::get_module_list(std::string process_name)
{
    std::vector<std::string> list;
    PVMMDLL_MAP_MODULE info = nullptr;
    if (!VMMDLL_Map_GetModuleU(fpga_handle, process_information.process_id, &info, VMMDLL_MODULE_FLAG_NORMAL))
        return list;

    for (size_t i = 0; i < info->cMap; i++)
        list.push_back(info->pMap[i].uszText);

    return list;
}

std::int32_t memory::get_pid_from_name(std::string process_name)
{
    DWORD pid = 0;
    if (fpga_handle)
    {
        VMMDLL_PidGetFromName(fpga_handle, const_cast<LPSTR>(process_name.c_str()), &pid);
    }
    return static_cast<std::int32_t>(pid);
}

bool memory::get_module_information(const std::string& module, std::uint64_t& base, std::uint64_t& size) const
{
    base = 0;
    size = 0;
    if (!fpga_handle || !process_information.process_id || module.empty())
    {
        return false;
    }

    PVMMDLL_MAP_MODULEENTRY e = nullptr;
    if (!VMMDLL_Map_GetModuleFromNameU(fpga_handle, process_information.process_id, const_cast<LPSTR>(module.c_str()), &e, VMMDLL_MODULE_FLAG_NORMAL))
        return false;

    if (!e)
    {
        return false;
    }
    base = e->vaBase;
    size = e->cbImageSize;
    VMMDLL_MemFree(e);
    return base && size;
}

std::uint64_t memory::get_base_address(const std::string& module) const
{
    std::uint64_t base = 0;
    std::uint64_t size = 0;
    return get_module_information(module, base, size) ? base : 0;
}

std::uint64_t memory::get_base_size(const std::string& module) const
{
    std::uint64_t base = 0;
    std::uint64_t size = 0;
    return get_module_information(module, base, size) ? size : 0;
}

bool memory::read(std::uint64_t address, void* buffer, std::size_t size) const
{
    return read(address, buffer, size, process_information.process_id);
}

bool memory::read(std::uint64_t address, void* buffer, std::size_t size, std::int32_t pid) const
{
    if (!address || !buffer || !size || !pid)
    {
        return false;
    }

    if (!fpga_handle)
    {
        return false;
    }

    DWORD read_size = 0;
    if (VMMDLL_MemReadEx(fpga_handle, pid, address, static_cast<PBYTE>(buffer), static_cast<DWORD>(size), &read_size,
                         VMMDLL_FLAG_NOCACHE) && read_size == size)
        return true;

    read_size = 0;
    if (VMMDLL_MemReadEx(fpga_handle, pid, address, static_cast<PBYTE>(buffer), static_cast<DWORD>(size), &read_size,
                         0) && read_size == size)
        return true;

    read_size = 0;
    if (VMMDLL_MemReadEx(fpga_handle, pid, address, static_cast<PBYTE>(buffer), static_cast<DWORD>(size), &read_size,
                         VMMDLL_FLAG_FORCECACHE_READ) && read_size == size)
        return true;

    auto* output = static_cast<std::uint8_t*>(buffer);
    std::size_t offset = 0;
    while (offset < size)
    {
        const std::size_t chunk = std::min<std::size_t>(size - offset, 0x1000u - ((address + offset) & 0xfffu));
        DWORD chunk_read = 0;
        if (!VMMDLL_MemReadEx(fpga_handle, pid, address + offset, output + offset,
                              static_cast<DWORD>(chunk), &chunk_read, 0) || chunk_read != chunk)
            return false;
        offset += chunk;
    }
    return true;
}

bool memory::read(std::uint64_t offset, void* buffer, std::size_t size, const std::string& module_name) const
{
    const std::uint64_t module_base = get_base_address(module_name);
    if (!module_base || offset > (std::numeric_limits<std::uint64_t>::max)() - module_base)
        return false;

    return read(module_base + offset, buffer, size);
}

std::string memory::read_widechar(std::uint64_t address, std::size_t max_length, std::int32_t pid) const
{
    if (!address || !max_length)
    {
        return {};
    }

    std::vector<char> buffer(max_length);
    const std::int32_t use_pid = pid ? pid : process_information.process_id;
    if (!read(address, buffer.data(), buffer.size(), use_pid))
    {
        return {};
    }

    const auto terminator = std::find(buffer.begin(), buffer.end(), '\0');
    return std::string(buffer.begin(), terminator);
}

bool memory::write(std::uint64_t address, void* buffer, std::size_t size) const
{
    return write(address, buffer, size, process_information.process_id);
}

bool memory::write(std::uint64_t address, void* buffer, std::size_t size, std::int32_t pid) const
{
#if MEMORY_USE_WINDOWS_RPM
    if (!native_process_handle || pid != process_information.process_id || !address || !buffer || !size)
    {
        return false;
    }

    SIZE_T bytes_written = 0;
    return WriteProcessMemory(native_process_handle, reinterpret_cast<LPVOID>(address), buffer, size, &bytes_written) &&
           bytes_written == size;
#else
    if (!fpga_handle || !address || !buffer || !size || !pid)
    {
        return false;
    }
    return VMMDLL_MemWrite(fpga_handle, pid, address, static_cast<PBYTE>(buffer), static_cast<DWORD>(size));
#endif
}

VMMDLL_SCATTER_HANDLE memory::create_scatter_handle(std::int32_t pid) const
{
    std::int32_t use_pid = pid ? pid : process_information.process_id;
    if (!fpga_handle || !use_pid)
    {
        return nullptr;
    }
    return VMMDLL_Scatter_Initialize(fpga_handle, use_pid, VMMDLL_FLAG_SCATTER_FORCE_PAGEREAD);
}

void memory::close_scatter_handle(VMMDLL_SCATTER_HANDLE handle)
{
    if (handle)
    {
        VMMDLL_Scatter_CloseHandle(handle);
    }
}

bool memory::refresh_process_memory()
{
    if (!fpga_handle || !process_information.process_id)
    {
        return false;
    }

    const ULONG64 process_refresh = VMMDLL_OPT_REFRESH_SPECIFIC_PROCESS |
                                    static_cast<std::uint32_t>(process_information.process_id);
    const bool process_ok = VMMDLL_ConfigSet(fpga_handle, process_refresh, 1) != FALSE;
    const bool tlb_ok = VMMDLL_ConfigSet(fpga_handle, VMMDLL_OPT_REFRESH_FREQ_TLB, 1) != FALSE;
    const bool memory_ok = VMMDLL_ConfigSet(fpga_handle, VMMDLL_OPT_REFRESH_FREQ_MEM, 1) != FALSE;
    return process_ok && tlb_ok && memory_ok;
}

bool memory::add_scatter_read(VMMDLL_SCATTER_HANDLE handle, std::uint64_t address, void* buffer, std::size_t size)
{
    if (!handle || !address || !buffer || !size)
    {
        return false;
    }

    return VMMDLL_Scatter_PrepareEx(handle, address, static_cast<DWORD>(size), static_cast<PBYTE>(buffer), nullptr);
}

bool memory::add_scatter_read(VMMDLL_SCATTER_HANDLE handle, std::uint64_t address, void* buffer, std::size_t size,
                              DWORD* bytes_read)
{
    if (!handle || !address || !buffer || !size || size > std::numeric_limits<DWORD>::max())
    {
        return false;
    }

    return VMMDLL_Scatter_PrepareEx(handle, address, static_cast<DWORD>(size), static_cast<PBYTE>(buffer), bytes_read);
}

bool memory::add_scatter_write(VMMDLL_SCATTER_HANDLE handle, std::uint64_t address, void* buffer, std::size_t size)
{
    if (!handle || !address || !buffer || !size)
    {
        return false;
    }

    return VMMDLL_Scatter_PrepareWrite(handle, address, static_cast<PBYTE>(buffer), static_cast<DWORD>(size));
}

bool memory::execute_read_scatter(VMMDLL_SCATTER_HANDLE handle, std::int32_t pid)
{
    std::int32_t use_pid = pid ? pid : process_information.process_id;
    if (!handle || !use_pid)
    {
        return false;
    }

    const bool result = VMMDLL_Scatter_ExecuteRead(handle);
    VMMDLL_Scatter_Clear(handle, use_pid, 0);
    return result;
}

bool memory::execute_write_scatter(VMMDLL_SCATTER_HANDLE handle, std::int32_t pid)
{
    std::int32_t use_pid = pid ? pid : process_information.process_id;

    if (!handle || !use_pid)
    {
        return false;
    }

    const bool result = VMMDLL_Scatter_Execute(handle);
    VMMDLL_Scatter_Clear(handle, use_pid, VMMDLL_FLAG_NOCACHE);
    return result;
}

std::int32_t memory::hex_value(unsigned char c) const
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }

    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }

    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }

    return -1;
}

bool memory::parse_signature(const char* signature, std::vector<pattern_byte>& out) const
{
    out.clear();
    if (!signature || !*signature)
    {
        return false;
    }

    const char* p = signature;
    while (*p)
    {
        if (*p == ' ')
        {
            ++p;
            continue;
        }

        if (*p == '?')
        {
            out.push_back({ 0, true });
            ++p;
            if (*p == '?')
            {
                ++p;
            }
            continue;
        }

        if (!p[0] || !p[1])
        {
            return false;
        }
        std::int32_t hi = hex_value(static_cast<unsigned char>(p[0]));
        std::int32_t lo = hex_value(static_cast<unsigned char>(p[1]));
        if (hi < 0 || lo < 0)
        {
            return false;
        }
        out.push_back({ static_cast<std::uint8_t>((hi << 4) | lo), false });
        p += 2;
    }
    return !out.empty();
}

std::uint64_t memory::find_aob_fast(const std::string& pattern, char wc)
{
    if (!fpga_handle || !process_information.process_id)
    {
        return 0;
    }

    std::vector<pattern_byte> sig;
    std::string normalized_pattern = pattern;
    if (wc && wc != '?')
        std::replace(normalized_pattern.begin(), normalized_pattern.end(), wc, '?');
    if (!parse_signature(normalized_pattern.c_str(), sig) || sig.empty())
    {
        return 0;
    }

    const auto matches_at = [&](const std::uint8_t* data, std::size_t pos)
    {
        for (std::size_t j = 0; j < sig.size(); ++j)
        {
            if (!sig[j].wildcard && data[pos + j] != sig[j].value)
            {
                return false;
            }
        }
        return true;
    };

    const auto scan_buffer = [&](const std::uint8_t* data, std::size_t size, std::uint64_t base) -> std::uint64_t {
        if (!data || sig.size() > size)
        {
            return 0;
        }
        std::size_t anchor = SIZE_MAX;
        for (std::size_t i = 0; i < sig.size(); ++i)
        {
            if (!sig[i].wildcard && sig[i].value != 0x00)
            {
                anchor = i;
                break;
            }
        }
        for (std::size_t i = 0; anchor == SIZE_MAX && i < sig.size(); ++i)
        {
            if (!sig[i].wildcard)
            {
                anchor = i;
                break;
            }
        }
        if (anchor == SIZE_MAX)
        {
            return 0;
        }
        const std::uint8_t anchor_byte = sig[anchor].value;
        for (std::size_t pos = 0; pos <= size - sig.size();)
        {
            const void* hit = std::memchr(data + pos + anchor, anchor_byte, size - pos - anchor);
            if (!hit)
            {
                break;
            }
            pos = static_cast<const std::uint8_t*>(hit) - data - anchor;
            if (matches_at(data, pos))
            {
                return base + pos;
            }
            ++pos;
        }
        return 0;
    };

    if (!process_bytes.empty())
    {
        if (const std::uint64_t result = scan_buffer(process_bytes.data(), process_bytes.size(), process_bytes_base))
            return result;
    }

    PVMMDLL_MAP_VAD vad_map = nullptr;
    if (!VMMDLL_Map_GetVadU(fpga_handle, process_information.process_id, FALSE, &vad_map))
    {
        return 0;
    }
    if (!vad_map || vad_map->dwVersion != VMMDLL_MAP_VAD_VERSION || !vad_map->cMap)
    {
        if (vad_map)
        {
            VMMDLL_MemFree(vad_map);
        }
        return 0;
    }

    const auto readable = [](DWORD p)
    {
        return !(p & PAGE_NOACCESS) && !(p & PAGE_GUARD);
    };

    const size_t CHUNK_SIZE = 0x400000;
    const size_t BATCH_SIZE = 4;
    const size_t overlap = sig.size() - 1;

    std::vector<std::uint8_t> buffer(CHUNK_SIZE + overlap);

    for (DWORD i = 0; i < vad_map->cMap; ++i)
    {
        const auto& vad = vad_map->pMap[i];
        if (!vad.vaStart || vad.vaEnd < vad.vaStart || !readable(vad.Protection))
        {
            continue;
        }
        if (vad.vaEnd - vad.vaStart < sig.size())
        {
            continue;
        }

        std::uint64_t current = vad.vaStart;
        std::uint64_t prev_chunk_end = 0;
        std::vector<std::uint8_t> prev_overlap(overlap);

        while (current <= vad.vaEnd)
        {
            VMMDLL_SCATTER_HANDLE hS = VMMDLL_Scatter_Initialize(fpga_handle, process_information.process_id, 0);
            if (!hS)
            {
                break;
            }

            struct chunk_info
            {
                std::vector<std::uint8_t> data;
                std::uint64_t base;
                size_t size;
                DWORD cbRead;
            };
            std::vector<chunk_info> chunks;
            chunks.reserve(BATCH_SIZE);

            size_t batch_count = 0;
            while (batch_count < BATCH_SIZE && current <= vad.vaEnd)
            {
                size_t to_read = static_cast<size_t>(std::min<std::uint64_t>(CHUNK_SIZE, vad.vaEnd - current + 1));
                if (to_read < sig.size())
                {
                    break;
                }

                chunk_info ci;
                ci.size = to_read;
                ci.base = current;
                ci.data.resize(to_read + overlap);
                DWORD cbRead = 0;
                if (!VMMDLL_Scatter_PrepareEx(hS, current, static_cast<DWORD>(to_read),
                                               ci.data.data(), &cbRead))
                {
                    break;
                }
                ci.cbRead = cbRead;
                chunks.push_back(std::move(ci));
                current += to_read;
                batch_count++;
            }

            if (chunks.empty())
            {
                VMMDLL_Scatter_CloseHandle(hS);
                break;
            }

            BOOL ok = VMMDLL_Scatter_ExecuteRead(hS);
            if (!ok)
            {
                ok = VMMDLL_Scatter_ExecuteRead(hS);
            }
            if (!ok)
            {
                VMMDLL_Scatter_CloseHandle(hS);
                current += 0x1000;
                continue;
            }

            for (size_t idx = 0; idx < chunks.size(); ++idx)
            {
                auto& ci = chunks[idx];
                if (idx == 0 && prev_chunk_end != 0)
                {
                    std::memcpy(ci.data.data(), prev_overlap.data(), overlap);
                }
                else if (idx > 0)
                {
                    const auto& prev_chunk = chunks[idx - 1];
                    std::memcpy(ci.data.data(),
                                prev_chunk.data.data() + prev_chunk.size,
                                overlap);
                }
                std::uint8_t* scan_start = ci.data.data() + overlap;
                size_t scan_size = ci.size + overlap;

                std::uint64_t scan_base = ci.base;
                if (idx == 0 && prev_chunk_end != 0)
                {
                    scan_base = ci.base - overlap;
                }
                else if (idx > 0)
                {
                    scan_base = ci.base - overlap;
                }

                if (const std::uint64_t result = scan_buffer(scan_start, scan_size, scan_base))
                {
                    VMMDLL_Scatter_CloseHandle(hS);
                    VMMDLL_MemFree(vad_map);
                    return result;
                }

                if (idx == chunks.size() - 1)
                {
                    std::memcpy(prev_overlap.data(),
                                ci.data.data() + ci.size,
                                overlap);
                    prev_chunk_end = ci.base + ci.size;
                }
            }

            VMMDLL_Scatter_Clear(hS, process_information.process_id, 0);
            VMMDLL_Scatter_CloseHandle(hS);
        }

    }

    VMMDLL_MemFree(vad_map);
    return 0;
}

bool memory::dump_pe_to_file(const std::string& module_name, const std::string& output_path, std::int32_t pid) const
{
    const std::int32_t use_pid = pid ? pid : process_information.process_id;
    if (!fpga_handle || !use_pid)
    {
        return false;
    }

    PVMMDLL_MAP_MODULEENTRY mod_entry = nullptr;
    if (!VMMDLL_Map_GetModuleFromNameU(fpga_handle, use_pid, const_cast<LPSTR>(module_name.c_str()), &mod_entry, VMMDLL_MODULE_FLAG_NORMAL))
        return false;

    const std::uint64_t base = mod_entry->vaBase;
    VMMDLL_MemFree(mod_entry);
    if (!base)
    {
        return false;
    }

    IMAGE_DOS_HEADER dos{};
    if (!read(base, &dos, sizeof(dos), use_pid) || dos.e_magic != IMAGE_DOS_SIGNATURE)
    {
        return false;
    }

    IMAGE_NT_HEADERS64 nt{};
    if (!read(base + dos.e_lfanew, &nt, sizeof(nt), use_pid) || nt.Signature != IMAGE_NT_SIGNATURE)
    {
        return false;
    }

    if (nt.OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC)
    {
        return false;
    }

    const std::uint16_t num_sections = nt.FileHeader.NumberOfSections;
    const std::uint32_t size_of_hdrs = nt.OptionalHeader.SizeOfHeaders;

    const std::uint32_t sec_tbl_off = dos.e_lfanew
        + sizeof(DWORD)
        + sizeof(IMAGE_FILE_HEADER)
        + nt.FileHeader.SizeOfOptionalHeader;

    std::vector<IMAGE_SECTION_HEADER> sections(num_sections);
    if (!read(base + sec_tbl_off, sections.data(), num_sections * sizeof(IMAGE_SECTION_HEADER), use_pid))
        return false;

    std::uint32_t file_size = size_of_hdrs;
    for (const auto& sec : sections)
    {
        if (!sec.PointerToRawData || !sec.SizeOfRawData)
        {
            continue;
        }
        std::uint32_t end = sec.PointerToRawData + sec.SizeOfRawData;
        if (end > file_size) file_size = end;
    }

    std::vector<std::uint8_t> out(file_size, 0);

    read(base, out.data(), size_of_hdrs, use_pid);

    for (const auto& sec : sections)
    {
        if (!sec.VirtualAddress || !sec.SizeOfRawData || !sec.PointerToRawData)
        {
            continue;
        }

        if (sec.PointerToRawData + sec.SizeOfRawData > file_size)
        {
            continue;
        }

        const std::uint32_t read_size = std::min<std::uint32_t>(sec.Misc.VirtualSize, sec.SizeOfRawData);
        read(base + sec.VirtualAddress, out.data() + sec.PointerToRawData, read_size, use_pid);
    }

    std::ofstream f(output_path, std::ios::binary | std::ios::trunc);
    if (!f)
    {
        return false;
    }
    f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    return f.good();
}

bool memory::is_cave_writable_vad(uint64_t address, size_t size, int pid)
{
    if (!address || address < 0x10000ULL || address > 0x00007FFFFFFEFFFFULL)
        return false;

    PVMMDLL_MAP_VAD pVadMap = nullptr;
    if (!VMMDLL_Map_GetVadU(fpga_handle, pid, TRUE, &pVadMap) || !pVadMap)
        return false;

    bool writable = false;
    for (DWORD i = 0; i < pVadMap->cMap; ++i)
    {
        const auto& entry = pVadMap->pMap[i];

        if (address < entry.vaStart || (address + size) > entry.vaEnd + 1)
            continue;

        const DWORD prot = entry.Protection;
        writable = (prot & PAGE_READWRITE) || (prot & PAGE_WRITECOPY) ||
            (prot & PAGE_EXECUTE_READWRITE) || (prot & PAGE_EXECUTE_WRITECOPY) ||
            (prot & PAGE_EXECUTE_READ) || (prot & PAGE_EXECUTE);
        break;
    }

    VMMDLL_MemFree(pVadMap);
    return writable;
}

bool memory::module_has_writable_rx_vad(uint64_t base, int pid)
{
    PVMMDLL_MAP_VAD pVadMap = nullptr;
    if (!VMMDLL_Map_GetVadU(fpga_handle, pid, TRUE, &pVadMap) || !pVadMap)
        return false;

    bool found = false;
    for (DWORD i = 0; i < pVadMap->cMap; ++i)
    {
        const auto& entry = pVadMap->pMap[i];
        if (entry.vaStart != base)
        {
            continue;
        }
        found = (entry.Protection & VMMDLL_MEMMAP_FLAG_PAGE_W) &&
            (entry.Protection & VMMDLL_MEMMAP_FLAG_PAGE_NX);
        break;
    }

    VMMDLL_MemFree(pVadMap);
    return found;
}
std::uint64_t memory::find_codecave(size_t function_size, const std::string& process_name, const std::string& module, bool peek)
{
    int pid = mem.get_pid_from_name(process_name);
    VMMDLL_PROCESS_INFORMATION process_info = { 0 };
    process_info.magic = VMMDLL_PROCESS_INFORMATION_MAGIC;
    process_info.wVersion = VMMDLL_PROCESS_INFORMATION_VERSION;
    SIZE_T process_info_size = sizeof(VMMDLL_PROCESS_INFORMATION);
    if (!VMMDLL_ProcessGetInformation(fpga_handle, pid, &process_info, &process_info_size))
    {
        return 0;
    }

    DWORD cSections = 0;
    if (!VMMDLL_ProcessGetSectionsU(fpga_handle, pid, const_cast<LPSTR>(module.c_str()), NULL, 0, &cSections) || !cSections)
    {
        return 0;
    }

    const PIMAGE_SECTION_HEADER pSections = static_cast<PIMAGE_SECTION_HEADER>(LocalAlloc(LMEM_ZEROINIT, cSections * sizeof(IMAGE_SECTION_HEADER)));
    if (!pSections || !VMMDLL_ProcessGetSectionsU(fpga_handle, pid, const_cast<LPSTR>(module.c_str()), pSections, cSections, &cSections) || !cSections)
    {
        return 0;
    }

    uint64_t mod_base = VMMDLL_ProcessGetModuleBaseU(fpga_handle, pid, const_cast<LPSTR>(module.c_str()));
    auto buffer = std::unique_ptr<uint8_t[]>(new uint8_t[function_size]);

    for (DWORD i = 0; i < cSections; i++)
    {
        if (!(pSections[i].Characteristics & IMAGE_SCN_MEM_EXECUTE))
            continue;

        uint32_t vsize = pSections[i].Misc.VirtualSize;
        uint32_t aligned = (vsize + 0xFFFu) & ~0xFFFu;

        if (aligned - vsize < function_size)
            continue;

        uint64_t candidate = mod_base + pSections[i].VirtualAddress + vsize;

        if (allocated_caves.count(candidate))
            continue;

        bool phys_backed = true;
        for (uint64_t page = candidate & ~0xFFFULL; page < candidate + function_size; page += 0x1000)
        {
            uint64_t pa = 0;
            if (!VMMDLL_MemVirt2Phys(fpga_handle, pid, page, &pa) || !pa)
            {
                phys_backed = false;
                break;
            }
        }
        if (!phys_backed)
            continue;

        if (!mem.read(candidate, buffer.get(), function_size, pid))
            continue;

        bool all_zero = true;
        for (size_t j = 0; j < function_size; j++)
            if (buffer[j] != 0x00)
            {
                all_zero = false;
                break;
            }

        if (all_zero)
        {
            if (!peek)
                allocated_caves.insert(candidate);
            LocalFree(pSections);
            return candidate;
        }
    }

    LocalFree(pSections);
    return 0;
}

std::vector<std::pair<std::uint64_t, std::string>> memory::find_all_codecave(size_t function_size, const std::string& process_name)
{
    std::vector<std::pair<std::uint64_t, std::string>> caves;
    int pid = get_pid_from_name(process_name);

    for (const auto& module : get_module_list(process_name))
    {
        if (module == process_name)
        {
            continue;
        }

        if (std::find(s_blacklist.begin(), s_blacklist.end(), module) != s_blacklist.end())
        {
            continue;
        }

        std::uint64_t base = VMMDLL_ProcessGetModuleBaseU(fpga_handle, pid, const_cast<LPSTR>(module.c_str()));
        if (base < 0x00007F0000000000ULL)
        {
            continue;
        }

        std::uint64_t cave = find_codecave(function_size, process_name, module, true);
        if (cave)
            caves.emplace_back(cave, module);
    }
    return caves;
}

std::pair<std::uint64_t, std::string> memory::alloc_codecave(size_t size, const std::string& process)
{
    auto caves = find_all_codecave(size, process);
    if (caves.empty())
    {
        return { 0, {} };
    }
    allocated_caves.insert(caves[0].first);
    return caves[0];
}

void memory::claim_codecave(std::uint64_t cave)
{
    allocated_caves.insert(cave);
}

void memory::release_codecave(std::uint64_t cave)
{
    allocated_caves.erase(cave);
}

bool memory::dump_memory_map()
{
    LC_CONFIG config{};
    config.dwVersion = LC_CONFIG_VERSION;
    strcpy_s(config.szDevice, "fpga://algo=0");
    HANDLE h = LcCreate(&config);
    if (!h)
    {
        LOG_WARN("[memory] LeechCore physical map probe initialization failed");
        return false;
    }

    PBYTE map_data = nullptr;
    DWORD map_bytes = 0;
    if (!LcCommand(h, LC_CMD_MEMMAP_GET_STRUCT, 0, nullptr, &map_data, &map_bytes) || !map_data ||
        map_bytes < sizeof(LC_MEMMAP_ENTRY) || map_bytes % sizeof(LC_MEMMAP_ENTRY) != 0)
    {
        if (map_data) LcMemFree(map_data);
        LcClose(h);
        LOG_WARN("[memory] LeechCore physical memory ranges unavailable bytes=%lu", map_bytes);
        return false;
    }

    const auto* ranges = reinterpret_cast<const LC_MEMMAP_ENTRY*>(map_data);
    const DWORD range_count = map_bytes / sizeof(LC_MEMMAP_ENTRY);

    auto path = std::filesystem::temp_directory_path() / "mmap.txt";
    std::ofstream f(path);
    if (!f.is_open())
    {
        LcMemFree(map_data);
        LcClose(h);
        LOG_WARN("[memory] unable to create physical map path=%s", path.string().c_str());
        return false;
    }

    f << std::hex;
    std::uint64_t total_bytes = 0;
    std::uint64_t first_address = (std::numeric_limits<std::uint64_t>::max)();
    std::uint64_t last_address = 0;
    for (DWORD i = 0; i < range_count; i++)
    {
        if (!ranges[i].cb || ranges[i].pa > (std::numeric_limits<std::uint64_t>::max)() - ranges[i].cb)
        {
            continue;
        }
        const std::uint64_t last = ranges[i].pa + ranges[i].cb - 1;
        f << ranges[i].pa << ' ' << last << '\n';
        total_bytes += ranges[i].cb;
        first_address = (std::min)(first_address, ranges[i].pa);
        last_address = (std::max)(last_address, last);
    }

    LOG_INFO("[memory] physical map ranges=%lu total_bytes=%llu first=0x%llx last=0x%llx path=%s", range_count,
             static_cast<unsigned long long>(total_bytes), first_address, last_address, path.string().c_str());

    f.flush();
    const bool write_ok = f.good();
    f.close();
    LcMemFree(map_data);
    LcClose(h);
    if (!write_ok)
    {
        LOG_WARN("[memory] physical map write failed path=%s", path.string().c_str());
    }
    return write_ok;
}

bool memory::set_fpga()
{
    std::uint64_t id = 0, major = 0, minor = 0;
    if (!VMMDLL_ConfigGet(fpga_handle, LC_OPT_FPGA_FPGA_ID, &id))           return false;
    if (!VMMDLL_ConfigGet(fpga_handle, LC_OPT_FPGA_VERSION_MAJOR, &major))  return false;
    if (!VMMDLL_ConfigGet(fpga_handle, LC_OPT_FPGA_VERSION_MINOR, &minor))  return false;

    if (major >= 4 && (major >= 5 || minor >= 7))
    {
        LC_CONFIG cfg{}; cfg.dwVersion = LC_CONFIG_VERSION;
        strcpy_s(cfg.szDevice, "existing");
        HANDLE h = LcCreate(&cfg);
        if (!h)
        {
            return false;
        }
        unsigned char abort2[4] = { 0x10, 0x00, 0x10, 0x00 };
        LcCommand(h, LC_CMD_FPGA_CFGREGPCIE_MARKWR | 0x002, 4, reinterpret_cast<PBYTE>(&abort2), nullptr, nullptr);
        LcClose(h);
    }
    return true;
}

std::string memory::query_registry(const char* path, e_registry_type type)
{
    if (!fpga_handle)
    {
        return {};
    }
    BYTE  buf[0x128] = {};
    DWORD reg_type = static_cast<DWORD>(type);
    DWORD size = sizeof(buf);
    if (!VMMDLL_WinReg_QueryValueExU(fpga_handle, const_cast<LPSTR>(path), &reg_type, buf, &size))
    {
        return {};
    }

    if (type == e_registry_type::dword)
    {
        return std::to_string(*reinterpret_cast<DWORD*>(buf));
    }
    const auto* value = reinterpret_cast<const wchar_t*>(buf);
    return narrow_process_name(value);
}

bool memory::init_keyboard()
{
    std::string win = query_registry("HKLM\\SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\CurrentBuild", e_registry_type::sz);
    std::string ubr = query_registry("HKLM\\SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\UBR", e_registry_type::dword);

    if (win.empty() || ubr.empty())
    {
        return false;
    }

    m_winlogon_pid = get_pid_from_name("winlogon.exe");

    int winver = std::stoi(win);

    if (winver > 22000)
    {
        SIZE_T count = 0;
        if (!VMMDLL_PidList(fpga_handle, nullptr, &count) || !count)
        {
            return false;
        }

        std::vector<DWORD> pids(count);
        if (!VMMDLL_PidList(fpga_handle, pids.data(), &count))
        {
            return false;
        }

        for (SIZE_T p = 0; p < count; p++)
        {
            VMMDLL_PROCESS_INFORMATION pi{}; SIZE_T pi_sz = sizeof(pi);
            pi.magic = VMMDLL_PROCESS_INFORMATION_MAGIC;
            pi.wVersion = VMMDLL_PROCESS_INFORMATION_VERSION;
            if (!VMMDLL_ProcessGetInformation(fpga_handle, pids[p], &pi, &pi_sz))
            {
                continue;
            }

            if (strcmp(pi.szName, "csrss.exe") != 0)
            {
                continue;
            }

            PVMMDLL_MAP_MODULEENTRY win32k = nullptr;
            bool got_win32k = VMMDLL_Map_GetModuleFromNameW(fpga_handle, pids[p], const_cast<LPWSTR>(L"win32ksgd.sys"), &win32k, VMMDLL_MODULE_FLAG_NORMAL);
            if (got_win32k)
            {
            }
            else
            {
                got_win32k = VMMDLL_Map_GetModuleFromNameW(fpga_handle, pids[p], const_cast<LPWSTR>(L"win32k.sys"), &win32k, VMMDLL_MODULE_FLAG_NORMAL);
                if (got_win32k)
                {
                }
                else
                {
                }
            }

            std::uint64_t g_session = find_signature("48 8B 05 ?? ?? ?? ?? 48 8B 04 C8", win32k->vaBase, win32k->vaBase + win32k->cbImageSize, pids[p]);
            if (!g_session)
            {
                g_session = find_signature("48 8B 05 ?? ?? ?? ?? FF C9", win32k->vaBase, win32k->vaBase + win32k->cbImageSize, pids[p]);

            }
            if (!g_session)
            {
                continue;
            }

            int relative = read<int>(g_session + 3, pids[p]);
            std::uint64_t slots = g_session + 7 + relative;

            std::uint64_t user_session = 0;
            for (int i = 0; i < 4; i++)
            {
                std::uint64_t a = read<std::uint64_t>(slots, pids[p]);
                std::uint64_t b = read<std::uint64_t>(a + 8 * i, pids[p]);
                std::uint64_t c = read<std::uint64_t>(b, pids[p]);
                user_session = c;
                if (user_session > 0x7FFFFFFFFFFF)
                {
                    break;
                }
            }

            PVMMDLL_MAP_MODULEENTRY base = nullptr;
            if (!VMMDLL_Map_GetModuleFromNameW(fpga_handle, pids[p], const_cast<LPWSTR>(L"win32kbase.sys"), &base, VMMDLL_MODULE_FLAG_NORMAL))
            {

            }

            std::uint64_t ptr = find_signature("48 8D 90 ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 57 C0", base->vaBase, base->vaBase + base->cbImageSize, pids[p]);

            if (!ptr)
            {
                continue;
            }

            std::uint32_t session_offset = read<std::uint32_t>(ptr + 3, pids[p]);
            m_gaf_async_key_state = user_session + session_offset;

            if (m_gaf_async_key_state > 0x7FFFFFFFFFFF)
            {
                return true;
            }
        }

        return m_gaf_async_key_state > 0x7FFFFFFFFFFF;
    }
    else
    {
        int winlogon_km_pid = get_pid_from_name("winlogon.exe") | VMMDLL_PID_PROCESS_WITH_KERNELMEMORY;

        PVMMDLL_MAP_EAT eat = nullptr;
        if (!VMMDLL_Map_GetEATU(fpga_handle, winlogon_km_pid, const_cast<LPSTR>("win32kbase.sys"), &eat))
        {
        }


        if (eat->dwVersion != VMMDLL_MAP_EAT_VERSION)
        {
            VMMDLL_MemFree(eat);
            return false;
        }

        bool found = false;
        for (DWORD i = 0; i < eat->cMap; i++)
        {
            if (strcmp(eat->pMap[i].uszFunction, "gafAsyncKeyState") == 0)
            {
                m_gaf_async_key_state = eat->pMap[i].vaFunction;
                found = true;
                break;
            }
        }

        VMMDLL_MemFree(eat);
        return m_gaf_async_key_state > 0x7FFFFFFFFFFF;
    }
}

void memory::update_keys()
{
    std::uint8_t prev[64] = {};
    memcpy(prev, m_key_state, 64);

    VMMDLL_MemReadEx(fpga_handle, m_winlogon_pid | VMMDLL_PID_PROCESS_WITH_KERNELMEMORY,
        m_gaf_async_key_state, reinterpret_cast<PBYTE>(m_key_state), 64, nullptr,
        VMMDLL_FLAG_NOCACHE);

    for (int vk = 0; vk < 256; ++vk)
        if ((m_key_state[vk * 2 / 8] & 1 << vk % 4 * 2) && !(prev[vk * 2 / 8] & 1 << vk % 4 * 2))
            m_prev_key_state[vk / 8] |= 1 << vk % 8;
}

std::uint64_t memory::find_signature_impl(const std::vector<pattern_byte>& pattern) const
{
    constexpr std::size_t page_size = 0x1000;
    if (process_bytes.empty() || process_bytes_valid.empty() || !process_bytes_base || pattern.empty() || pattern.size() > process_bytes.size())
        return 0;

    const std::size_t limit = process_bytes.size() - pattern.size();
    for (std::size_t i = 0; i <= limit; ++i)
    {
        const std::size_t first_page = i / page_size;
        const std::size_t last_page = (i + pattern.size() - 1) / page_size;
        bool pages_valid = true;
        for (std::size_t page = first_page; page <= last_page; ++page)
        {
            if (!process_bytes_valid[page])
            {
                pages_valid = false;
                break;
            }
        }
        if (!pages_valid)
            continue;

        if (find_pattern_offset(process_bytes.data() + i, pattern.size(), pattern) == 0)
            return process_bytes_base + i;
    }
    return 0;
}

std::uint64_t memory::find_signature_live_impl(const std::vector<pattern_byte>& pattern, std::uint64_t start, std::uint64_t end, std::int32_t pid, std::size_t chunk_size) const
{
    const std::int32_t use_pid = pid ? pid : process_information.process_id;
    if (!fpga_handle || !use_pid || start >= end || pattern.empty() || pattern.size() > end - start)
        return 0;

    chunk_size = (std::max)(chunk_size, pattern.size());
    chunk_size = (std::min)(chunk_size, static_cast<std::size_t>((std::numeric_limits<DWORD>::max)()) - pattern.size());
    const std::size_t overlap = pattern.size() - 1;
    std::vector<std::uint8_t> buffer(chunk_size + overlap);

    const auto scan_buffer = [&](const std::uint8_t* data, std::size_t size, std::uint64_t address) -> std::uint64_t
    {
        const std::size_t offset = find_pattern_offset(data, size, pattern);
        return offset == SIZE_MAX ? 0 : address + offset;
    };

    for (std::uint64_t current = start; current < end;)
    {
        const std::uint64_t remaining = end - current;
        const std::size_t read_size = static_cast<std::size_t>((std::min<std::uint64_t>)(buffer.size(), remaining));
        DWORD bytes_read = 0;

        if (VMMDLL_MemReadEx(fpga_handle, use_pid, current, buffer.data(), static_cast<DWORD>(read_size), &bytes_read,
                             VMMDLL_FLAG_NOCACHE) && bytes_read >= pattern.size())
        {
            if (const std::uint64_t result = scan_buffer(buffer.data(), bytes_read, current))
                return result;
        }
        else
        {
            constexpr std::size_t page_size = 0x1000;
            std::vector<std::uint8_t> page_buffer(page_size + overlap);
            for (std::size_t page_offset = 0; page_offset < read_size; page_offset += page_size)
            {
                const std::size_t page_read_size = (std::min)(page_buffer.size(), read_size - page_offset);
                DWORD page_bytes_read = 0;
                bool page_ok = VMMDLL_MemReadEx(fpga_handle, use_pid, current + page_offset,
                    page_buffer.data(), static_cast<DWORD>(page_read_size), &page_bytes_read,
                    VMMDLL_FLAG_NOCACHE) && page_bytes_read >= pattern.size();
                if (!page_ok)
                {
                    page_bytes_read = 0;
                    page_ok = VMMDLL_MemReadEx(fpga_handle, use_pid, current + page_offset,
                        page_buffer.data(), static_cast<DWORD>(page_read_size), &page_bytes_read, 0) &&
                        page_bytes_read >= pattern.size();
                }
                if (!page_ok && page_read_size > page_size)
                {
                    const std::size_t exact_page_size = (std::min)(page_size, read_size - page_offset);
                    page_bytes_read = 0;
                    page_ok = VMMDLL_MemReadEx(fpga_handle, use_pid, current + page_offset,
                        page_buffer.data(), static_cast<DWORD>(exact_page_size), &page_bytes_read,
                        VMMDLL_FLAG_NOCACHE) && page_bytes_read >= pattern.size();
                    if (!page_ok)
                    {
                        page_bytes_read = 0;
                        page_ok = VMMDLL_MemReadEx(fpga_handle, use_pid, current + page_offset,
                            page_buffer.data(), static_cast<DWORD>(exact_page_size), &page_bytes_read, 0) &&
                            page_bytes_read >= pattern.size();
                    }
                }
                if (page_ok)
                {
                    if (const std::uint64_t result = scan_buffer(page_buffer.data(), page_bytes_read, current + page_offset))
                        return result;
                }
            }
        }

        const std::uint64_t advance = (std::min<std::uint64_t>)(chunk_size, remaining);
        if (!advance)
        {
            break;
        }
        current += advance;
    }
    return 0;
}

std::uint64_t memory::find_signature(const char* signature)
{
    if (!signature || !*signature)
    {
        return 0;
    }
    std::vector<pattern_byte> pattern;
    return parse_signature(signature, pattern) ? find_signature_impl(pattern) : 0;
}

std::uint64_t memory::find_signature(const std::vector<std::uint8_t>& signature)
{
    std::vector<pattern_byte> pattern;
    pattern.reserve(signature.size());
    for (std::uint8_t byte : signature)
    {
        pattern.push_back({ byte, false });
    }
    return find_signature_impl(pattern);
}

std::uint64_t memory::find_signature(const std::vector<pattern_byte>& pattern)
{
    return find_signature_impl(pattern);
}

std::uint64_t memory::find_signature(const char* signature, std::uint64_t start, std::uint64_t end, std::int32_t pid)
{
    return find_signature(signature, start, end, pid, 0x10000);
}

std::uint64_t memory::find_signature(const char* signature, std::uint64_t start, std::uint64_t end, std::int32_t pid, std::size_t chunk_size)
{
    if (!signature || !*signature)
    {
        return 0;
    }

    std::vector<pattern_byte> pattern;
    if (!parse_signature(signature, pattern))
    {
        return 0;
    }

    return find_signature_live_impl(pattern, start, end, pid, chunk_size);
}

std::uint64_t memory::find_signature(const std::vector<std::uint8_t>& signature, std::uint64_t start, std::uint64_t end, std::int32_t pid)
{
    std::vector<pattern_byte> pattern;
    pattern.reserve(signature.size());
    for (std::uint8_t b : signature)
        pattern.push_back({ b, false });

    return find_signature_live_impl(pattern, start, end, pid, 0x10000);
}

std::uint64_t memory::find_signature(const std::vector<pattern_byte>& pattern, std::uint64_t start, std::uint64_t end, std::int32_t pid)
{
    return find_signature_live_impl(pattern, start, end, pid, 0x10000);
}

void memory::tick_keys()
{
    if (m_gaf_async_key_state > 0x7FFFFFFFFFFF)
        update_keys();
}
