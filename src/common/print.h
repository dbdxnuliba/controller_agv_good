#pragma once

#include <stdio.h>
#include <chrono>
#include <string.h>
#include <string>
#include "common/back_trace.h"
#include "common/spdlog/spdlog.h"

#define FILE_NAME(x) strrchr(x,'/')?strrchr(x,'/')+1:x

class Logger
{
public:
    std::shared_ptr<spdlog::logger> p_logger;

    ~Logger()
    {
        printf("\n%s exit!\n", name);
        //print_trace();
    }

    void set_name(const char* i_name)
    {
        name = i_name;
    }

    const char* name;
};

void init_log(const char *name);
extern Logger LOGGER;

#define USE_SPD_LOG

#ifndef USE_SPD_LOG

#define ESC_START     "\033["
#define ESC_END       "\033[0m"
#define COLOR_FATAL   "5;31;40m"
#define COLOR_ALERT   "0;31;40m"
#define COLOR_CRIT    "0;31;40m"
#define COLOR_ERROR   "0;31;40m"
#define COLOR_WARN    "0;33;40m"
#define COLOR_NOTICE  "0;34;40m"
#define COLOR_INFO    "0;32;40m"
#define COLOR_DEBUG   "0;36;40m"
#define COLOR_TRACE   "0;37;40m"

#define PRINT_INFO(format, args...) (printf( ESC_START COLOR_INFO "[INFO]-[%ld]-[%s]-[%s]-[%d]: " format"\n" ESC_END, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(), FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
 
#define PRINT_DEBUG(format, args...) (printf( ESC_START COLOR_DEBUG "[DEBUG]-[%ld]-[%s]-[%s]-[%d]: " format"\n" ESC_END, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(), FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
 
#define PRINT_WARN(format, args...) (printf( ESC_START COLOR_WARN "[WARN]-[%ld]-[%s]-[%s]-[%d]: " format"\n" ESC_END, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(), FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
 
#define PRINT_ERROR(format, args...) (printf( ESC_START COLOR_ERROR "[ERROR]-[%ld]-[%s]-[%s]-[%d]: " format"\n" ESC_END, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(), FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))

#else
#define PRINT_INFO(format, args...) (LOGGER.p_logger->info("[{}]-[{}]-[{}]: " format"", FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
#define PRINT_DEBUG(format, args...) (LOGGER.p_logger->debug("[{}]-[{}]-[{}]: " format"", FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
#define PRINT_WARN(format, args...) (LOGGER.p_logger->warn("[{}]-[{}]-[{}]: " format"", FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
#define PRINT_ERROR(format, args...) (LOGGER.p_logger->error("[{}]-[{}]-[{}]: " format"", FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
//#define PRINT_DEBUG(format, args...) (printf( ESC_START COLOR_DEBUG "[DEBUG]-[%ld]-[%s]-[%s]-[%d]: " format"\n" ESC_END, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(), FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))

//#define PRINT_WARN(format, args...) (printf( ESC_START COLOR_WARN "[WARN]-[%ld]-[%s]-[%s]-[%d]: " format"\n" ESC_END, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(), FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))

//#define PRINT_ERROR(format, args...) (printf( ESC_START COLOR_ERROR "[ERROR]-[%ld]-[%s]-[%s]-[%d]: " format"\n" ESC_END, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(), FILE_NAME(__FILE__), __FUNCTION__ , __LINE__, ##args))
#endif
