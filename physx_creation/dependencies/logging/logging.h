#pragma once

#include <filesystem>

namespace logging
{
enum class level
{
    debug,
    info,
    warning,
    error
};

class logger
{
  public:
    static logger &instance();

    logger(const logger &) = delete;
    logger &operator=(const logger &) = delete;
    logger(logger &&) = delete;
    logger &operator=(logger &&) = delete;

    void set_file_path(const std::filesystem::path &path);
    void write(level log_level, const char *file, int line, const char *format, ...);

  private:
    logger();
    ~logger();

    struct impl;
    impl *impl_;
};
} // namespace logging

#if defined(_DEBUG) || !defined(NDEBUG)

#define LOG_INFO(...) ::logging::logger::instance().write(::logging::level::info, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...) ::logging::logger::instance().write(::logging::level::warning, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) ::logging::logger::instance().write(::logging::level::error, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...) ::logging::logger::instance().write(::logging::level::debug, __FILE__, __LINE__, __VA_ARGS__)

#else

#define LOG_INFO(...) ::logging::logger::instance().write(::logging::level::info, nullptr, 0, __VA_ARGS__)
#define LOG_WARN(...) ::logging::logger::instance().write(::logging::level::warning, nullptr, 0, __VA_ARGS__)
#define LOG_ERROR(...) ::logging::logger::instance().write(::logging::level::error, nullptr, 0, __VA_ARGS__)
#define LOG_DEBUG(...) ((void)0)

#endif