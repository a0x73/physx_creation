#pragma once
#define NOMINMAX

#ifndef MEMORY_USE_WINDOWS_RPM
#define MEMORY_USE_WINDOWS_RPM 0
#endif

#include <Windows.h>

#pragma warning(push)
#pragma warning(disable : 4200)
#include "libs/vmmdll.h"
#pragma warning(pop)

#include <cstdint>
#include <cstddef>
#include <array>
#include <string>
#include <vector>
#include <unordered_set>
#include "../security/signature.h"

class memory
{
public:
    struct current_process_information
    {
        std::int32_t process_id = 0;
        std::uint64_t base_address = 0;
        std::uint64_t base_size = 0;
        std::string name = {};
    };

    struct pattern_byte
    {
        std::uint8_t value = 0;
        bool wildcard = false;
    };

    static constexpr std::size_t max_managed_string_length = 128;

    struct managed_string_scatter_read
    {
        std::uint64_t address = 0;
        std::int32_t max_length = static_cast<std::int32_t>(max_managed_string_length);
        std::int32_t length = 0;
        std::array<wchar_t, max_managed_string_length> wide{};
        std::string value{};
    };

#if MEMORY_USE_WINDOWS_RPM
    struct native_scatter_read
    {
        std::uint64_t address{};
        void* buffer{};
        std::size_t size{};
        DWORD* bytes_read{};
    };

    struct native_scatter_context
    {
        std::int32_t process_id{};
        std::vector<native_scatter_read> reads{};
    };
#endif

    memory();
    ~memory();

    bool load(std::string process_name, bool use_mem_map);
    bool load_memory_into_vector(const std::string& module_name = {});
    void clear_memory_vector()
    {
        process_bytes.clear();
        process_bytes_valid.clear();
        process_bytes_base = 0;
    }

    std::uint64_t get_base() const
    {
        return process_information.base_address;
    }

    std::uint64_t get_base(std::string module_name)
    {
        std::uint64_t base = 0;
#if MEMORY_USE_WINDOWS_RPM
        std::uint64_t size = 0;
        if (!get_module_information(module_name, base, size))
        {
            return 0;
        }

        return base;
#else
        if (!fpga_handle || !process_information.process_id || module_name.empty())
        {
            return false;
        }

        PVMMDLL_MAP_MODULEENTRY e = nullptr;
        if (!VMMDLL_Map_GetModuleFromNameU(fpga_handle, process_information.process_id, const_cast<LPSTR>(module_name.c_str()), &e, VMMDLL_MODULE_FLAG_NORMAL))
            return false;

        if (!e)
        {
            return false;
        }
        base = e->vaBase;
        VMMDLL_MemFree(e);
        return base;
#endif
    }
    std::uint64_t get_size() const
    {
        return process_information.base_size;
    }

    std::int32_t get_pid() const
    {
        return process_information.process_id;
    }

    bool is_ready() const
    {
        return initialised;
    }

    std::vector<std::string> get_module_list(std::string process_name);

    bool read(std::uint64_t address, void* buffer, std::size_t size) const;
    bool read(std::uint64_t address, void* buffer, std::size_t size, std::int32_t pid) const;
    bool read(std::uint64_t offset, void* buffer, std::size_t size, const std::string& module_name) const;
    std::string read_widechar(std::uint64_t address, std::size_t max_length, std::int32_t pid = 0) const;

    template <typename T>
    T read(std::uint64_t address, std::int32_t pid = 0) const
    {
        T buffer{};
        read(address, &buffer, sizeof(T), pid ? pid : process_information.process_id);
        return buffer;
    }

    template <typename T>
    T read(std::uint64_t offset, const std::string& module_name) const
    {
        T buffer{};
        read(offset, &buffer, sizeof(T), module_name);
        return buffer;
    }

    template <typename type>
    std::vector<type> read_array(uintptr_t address, size_t size, size_t custom_type_size = NULL)
    {

        std::vector<type> temp{};

        constexpr std::size_t max_read_elements = 4000000;
        constexpr std::size_t max_read_bytes = 256ull * 1024 * 1024;

        const std::size_t byte_count = custom_type_size ? custom_type_size : sizeof(type) * size;

        if (!size || size > max_read_elements || byte_count > max_read_bytes)
        {
            return temp;
        }

        temp.resize(custom_type_size ? custom_type_size : size);

        if (read(address, temp.data(), byte_count))
        {
            return temp;
        }

        return temp;

    }

    std::string read_str(std::uint64_t address, std::size_t max_length = max_managed_string_length) const
    {
        std::string out(max_length, '\0');

        if (!address || !max_length)
        {
            return {};
        }

        if (!read(address, out.data(), max_length))
        {
            return {};
        }

        const auto terminator = out.find('\0');
        if (terminator != std::string::npos)
            out.resize(terminator);

        return out;
    }

    template <typename T>
    T read_chain(std::uint64_t address, const std::vector<std::uint64_t>& offsets) const
    {
        std::uint64_t current = address;

        for (std::size_t i = 0; i + 1 < offsets.size(); ++i)
        {
            current = read<std::uint64_t>(current + offsets[i]);

            if (!current)
            {
                return {};
            }
        }

        return read<T>(current + offsets.back());
    }

    bool write(std::uint64_t address, void* buffer, std::size_t size) const;
    bool write(std::uint64_t address, void* buffer, std::size_t size, std::int32_t pid) const;

    template <typename T>
    bool write(std::uint64_t address, const T& value) const
    {
        return write(address, const_cast<T*>(&value), sizeof(T));
    }

    VMMDLL_SCATTER_HANDLE create_scatter_handle(std::int32_t pid = 0) const;
    void close_scatter_handle(VMMDLL_SCATTER_HANDLE handle);
    bool refresh_process_memory();
    bool add_scatter_read(VMMDLL_SCATTER_HANDLE handle, std::uint64_t address, void* buffer, std::size_t size);
    bool add_scatter_read(VMMDLL_SCATTER_HANDLE handle, std::uint64_t address, void* buffer, std::size_t size,
                          DWORD* bytes_read);
    bool add_scatter_write(VMMDLL_SCATTER_HANDLE handle, std::uint64_t address, void* buffer, std::size_t size);
    bool execute_read_scatter(VMMDLL_SCATTER_HANDLE handle, std::int32_t pid = 0);
    bool execute_write_scatter(VMMDLL_SCATTER_HANDLE handle, std::int32_t pid = 0);

    bool parse_signature(const char* signature, std::vector<pattern_byte>& out) const;
    std::uint64_t find_signature(const char* signature);
    std::uint64_t find_signature(const std::vector<std::uint8_t>& signature);
    std::uint64_t find_signature(const std::vector<pattern_byte>& pattern);
    std::uint64_t find_signature(const char* signature, std::uint64_t start, std::uint64_t end, std::int32_t pid = 0);
    std::uint64_t find_signature(const char* signature, std::uint64_t start, std::uint64_t end, std::int32_t pid, std::size_t chunk_size);
    std::uint64_t find_signature(const std::vector<std::uint8_t>& signature, std::uint64_t start, std::uint64_t end, std::int32_t pid = 0);
    std::uint64_t find_signature(const std::vector<pattern_byte>& pattern, std::uint64_t start, std::uint64_t end, std::int32_t pid = 0);
    std::uint64_t find_aob_fast(const std::string& pattern, char wc);
    template <std::size_t N>
    std::uint64_t find_signature(const senc::sig_t<N>& sig)
    {
        std::vector<pattern_byte> pattern;
        pattern.reserve(sig.size());

        for (std::size_t i = 0; i < sig.size(); ++i)
            pattern.push_back({ sig.at(i), sig.mask[i] != 0 });

        return find_signature(pattern);
    }

    template <std::size_t N>
    std::uint64_t find_signature(const senc::sig_t<N>& sig, std::uint64_t start, std::uint64_t end, std::int32_t pid = 0)
    {
        std::vector<pattern_byte> pattern;
        pattern.reserve(sig.size());

        for (std::size_t i = 0; i < sig.size(); ++i)
            pattern.push_back({ sig.at(i), sig.mask[i] != 0 });

        return find_signature(pattern, start, end, pid);
    }

    bool dump_pe_to_file(const std::string& module_name, const std::string& output_path, std::int32_t pid = 0) const;

    std::uint64_t find_codecave(size_t size, const std::string& process, const std::string& module, bool peek = false);
    bool is_cave_writable_vad(uint64_t address, size_t size, int pid);
    bool module_has_writable_rx_vad(uint64_t base, int pid);
    std::vector<std::pair<std::uint64_t, std::string>> find_all_codecave(size_t size, const std::string& process);
    std::pair<std::uint64_t, std::string> alloc_codecave(size_t size, const std::string& process);
    void claim_codecave(std::uint64_t cave);
    void release_codecave(std::uint64_t cave);

    bool init_keyboard();
    void tick_keys();

    bool is_key_down(std::uint32_t vk) const
    {
        return m_gaf_async_key_state > 0x7FFFFFFFFFFF &&
               (m_key_state[vk * 2 / 8] & (1 << vk % 4 * 2)) != 0;
    }

    std::int32_t get_pid_from_name(std::string process_name);

private:
    struct library_modules
    {
        HMODULE vmm = nullptr;
        HMODULE ftd3xx = nullptr;
        HMODULE leechcore = nullptr;
    } modules;

    enum class e_registry_type
    {
        sz = REG_SZ,
        dword = REG_DWORD,
    };

    current_process_information process_information{};
    VMM_HANDLE fpga_handle = nullptr;
#if MEMORY_USE_WINDOWS_RPM
    HANDLE native_process_handle{};
#endif
    bool initialised = false;

    std::uint64_t m_gaf_async_key_state = 0;
    std::uint8_t m_key_state[64] = {};
    std::uint8_t m_prev_key_state[32] = {};
    std::int32_t m_winlogon_pid = 0;

    bool dump_memory_map();
    bool set_fpga();
    bool get_module_information(const std::string& module, std::uint64_t& base, std::uint64_t& size) const;
    std::uint64_t get_base_address(const std::string& module) const;
    std::uint64_t get_base_size(const std::string& module) const;
    std::string query_registry(const char* path, e_registry_type type);
    std::int32_t hex_value(unsigned char c) const;
    void update_keys();

    std::unordered_set<std::uint64_t> allocated_caves;

    std::vector<std::uint8_t> process_bytes = std::vector<std::uint8_t>(0);
    std::vector<std::uint8_t> process_bytes_valid = std::vector<std::uint8_t>(0);
    std::uint64_t process_bytes_base = 0;

    std::uint64_t find_signature_impl(const std::vector<pattern_byte>& pattern) const;
    std::uint64_t find_signature_live_impl(const std::vector<pattern_byte>& pattern, std::uint64_t start, std::uint64_t end, std::int32_t pid, std::size_t chunk_size) const;
};

inline memory mem;
