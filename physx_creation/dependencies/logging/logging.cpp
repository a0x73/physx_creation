#include "logging.h"

#include <Windows.h>

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include <string>

namespace logging
{
namespace
{
constexpr auto default_log_file = "codeine.log";

const char *level_name(level log_level)
{
    switch (log_level)
    {
    case level::debug:
        return "DEBUG";
    case level::info:
        return "INFO";
    case level::warning:
        return "WARN";
    case level::error:
        return "ERROR";
    }

    return "LOG";
}

std::string current_time_text()
{
    std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_s(&local_time, &now);

    char buffer[32]{};
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &local_time);
    return buffer;
}

const char *filename_from_path(const char *path)
{
    if (!path)
    {
        return "unknown";
    }

    const char *filename = path;
    for (const char *cursor = path; *cursor; ++cursor)
    {
        if (*cursor == '\\' || *cursor == '/')
        {
            filename = cursor + 1;
        }
    }

    return filename;
}
} // namespace

struct logger::impl
{
    std::mutex mutex;
    std::filesystem::path file_path = default_log_file;
    std::ofstream file;

    void open_file_if_needed()
    {
        if (file.is_open())
        {
            return;
        }

        file.open(file_path, std::ios::out | std::ios::app);
    }
};

logger &logger::instance()
{
    static logger instance;
    return instance;
}

logger::logger() : impl_(new impl{})
{
    std::filesystem::remove(default_log_file);
}

logger::~logger()
{
    delete impl_;
}

void logger::set_file_path(const std::filesystem::path &path)
{
    std::lock_guard lock(impl_->mutex);

    if (impl_->file_path == path)
    {
        return;
    }

    if (impl_->file.is_open())
    {
        impl_->file.close();
    }

    impl_->file_path = path;
    std::filesystem::remove(impl_->file_path);
}

void logger::write(level log_level, const char *file, int line, const char *format, ...)
{
    char message[1024]{};

    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    char final_message[1400]{};
    snprintf(final_message, sizeof(final_message), "/ [%s] / [%s] - %s (%s:%d)\n", current_time_text().c_str(),
             level_name(log_level), message, file ? filename_from_path(file) : "", line);

    std::lock_guard lock(impl_->mutex);

    impl_->open_file_if_needed();
    if (impl_->file.is_open())
    {
        impl_->file << final_message;
        impl_->file.flush();
    }

#if defined(_DEBUG) || !defined(NDEBUG)
    FILE *output = log_level == level::error || log_level == level::warning ? stderr : stdout;
    fputs(final_message, output);
    fflush(output);

    OutputDebugStringA(final_message);
#endif
}
} // namespace logging
