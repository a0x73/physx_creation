#pragma once
#include <cstdint>
#include <cstddef>
#include <array>

namespace senc
{
    template <std::size_t N>
    struct sig_t
    {
        static constexpr size_t str_size_v = N * 3;

        std::array<uint8_t, N> data{};
        std::array<uint8_t, N> mask{};
        size_t count = 0;
        uint8_t key = 0;

        static constexpr size_t size_v = N;

        constexpr const uint8_t* data_ptr() const
        {
            return data.data();
        }

        constexpr const uint8_t* mask_ptr() const
        {
            return mask.data();
        }

        constexpr size_t size() const
        {
            return count;
        }

        constexpr uint8_t at(size_t i) const
        {
            return mask[i] ? uint8_t(0) : uint8_t(data[i] ^ key);
        }

        static constexpr char hex_ch(uint8_t v)
        {
            return v < 10 ? char('0' + v) : char('A' + (v - 10));
        }

        // decrypt into caller's buffer (stack). plaintext never stored statically.
        constexpr void to_buffer(char* out, size_t out_size) const
        {
            size_t pos = 0;
            for (size_t i = 0; i < count && pos + 3 < out_size; ++i)
            {
                if (i)
                {
                    out[pos++] = ' ';
                }

                if (mask[i])
                {
                    out[pos++] = '?';
                    out[pos++] = '?';
                }
                else
                {
                    uint8_t b = uint8_t(data[i] ^ key);
                    out[pos++] = hex_ch(uint8_t(b >> 4));
                    out[pos++] = hex_ch(uint8_t(b & 0x0F));
                }
            }
            out[pos] = '\0';
        }

        // RAII helper: stack buffer that wipes itself on destruction
        struct scoped_str
        {
            char buf[str_size_v];
            scoped_str(const sig_t& s)
            {
                s.to_buffer(buf, str_size_v);
            }

            ~scoped_str()
            {
                volatile char* p = buf;
                for (size_t i = 0; i < str_size_v; ++i)
                {
                    p[i] = 0;
                }
            }
            scoped_str(const scoped_str&) = delete;
            scoped_str& operator=(const scoped_str&) = delete;
            const char* c_str() const
            {
                return buf;
            }
        };

        scoped_str to_scoped() const
        {
            return scoped_str(*this);
        }
    };

    constexpr uint8_t hex_val(char c)
    {
        return (c >= '0' && c <= '9') ? uint8_t(c - '0') :
            (c >= 'A' && c <= 'F') ? uint8_t(c - 'A' + 10) :
            (c >= 'a' && c <= 'f') ? uint8_t(c - 'a' + 10) :
            0;
    }

    template <std::size_t N>
    consteval uint8_t derive_key(const char(&str)[N])
    {
        uint8_t key = 0xA5;
        for (size_t i = 0; i < N - 1; ++i)
            key = uint8_t((key ^ uint8_t(str[i])) * uint8_t(0x5D)) + uint8_t(0x29);
        return key ? key : uint8_t(0x5A);
    }

    template<size_t N>
    consteval sig_t<N> make_sig(const char(&str)[N], uint8_t key)
    {
        sig_t<N> out{};
        out.key = key;

        size_t i = 0;
        size_t idx = 0;

        while (i < N - 1)
        {
            while (i < N - 1 && str[i] == ' ')
                ++i;

            if (i >= N - 1)
                break;

            if (str[i] == '?' && str[i + 1] == '?')
            {
                out.mask[idx] = 1;
                out.data[idx] = 0;
                i += 2;
            }
            else
            {
                uint8_t byte = (hex_val(str[i]) << 4) | hex_val(str[i + 1]);
                out.data[idx] = byte ^ key;
                out.mask[idx] = 0;
                i += 2;
            }

            ++idx;
        }

        out.count = idx;
        return out;
    }
}

#define SIG(key, str) \
    senc::make_sig(str, senc::derive_key(str))
