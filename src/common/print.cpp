#include "common/print.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <exception>
#include "common/spdlog/spdlog.h"
#include "common/spdlog/sinks/stdout_color_sinks.h"
#include "common/spdlog/sinks/basic_file_sink.h"
#include "common/spdlog/sinks/daily_file_sink.h"
static std::once_flag log_init_flag;
static spdlog::sink_ptr console_sink;
static spdlog::sink_ptr file_sink;



Logger LOGGER;


void __init_flag(const char *name)
{
    printf("init log :%s\n", name);
    LOGGER.set_name(name);
    try
    {
        char file_path[100];
        sprintf(file_path, "logs/%s.txt", name);
        console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e]-[%n]-[%l] %v%$");
        console_sink->set_level(spdlog::level::trace);
        //file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/multisink.txt", true);
        file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(file_path, 0, 0, false, 3);
        file_sink->set_level(spdlog::level::trace);
        std::vector<spdlog::sink_ptr> sinks_list;
        sinks_list.emplace_back(console_sink);
        sinks_list.emplace_back(file_sink);
        LOGGER.p_logger = std::make_shared<spdlog::logger>(name);
        LOGGER.p_logger->sinks() = sinks_list;
        spdlog::flush_every(std::chrono::seconds(1));
        LOGGER.p_logger->flush_on(spdlog::level::trace);
        LOGGER.p_logger->set_level(spdlog::level::trace);
        LOGGER.p_logger->info("Version: {}", GIT_SHA1);
    }
    catch(std::exception& e )
    {
        printf("%s\n", e.what());
    }
    catch(...)
    {
        printf("un expexted occured\n");
    }
}

void init_log(const char *name)
{
    std::call_once(log_init_flag, __init_flag, name);
}
