#ifndef LOG_H_
#define LOG_H_

#include <stdio.h>
#include <string>
#include <time.h>
#include <mutex>

#ifdef WIN32
#include <Windows.h>
class TimeCalc
{
public:
    TimeCalc() {
        QueryPerformanceFrequency(&freq_);
        QueryPerformanceCounter(&start_);
    }
    ~TimeCalc() {
    }

public:
    void Print(const char* fmt, ...) {
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        double fTime = (now.QuadPart - start_.QuadPart) / double(freq_.QuadPart);
        char fmtmsg[512] = { 0 };
        char msg[1024] = { 0 };
        va_list args;
        va_start(args, fmt);
        _vsnprintf_s(fmtmsg, sizeof(fmtmsg), fmt, args);
        va_end(args);
        sprintf_s(msg, 1024, "[%.6f] %s", fTime, fmtmsg);
        OutputDebugStringA(msg);
    }

public:
    LARGE_INTEGER freq_;
    LARGE_INTEGER start_;
};
#else
class TimeCalc
{
public:
    TimeCalc() {}
    ~TimeCalc(){}
public:
    void Print(const char* fmt, ...){}
};
#endif

class CLog
{
    friend CLog* TheLog();
private:
    CLog();
public:
    ~CLog();

    enum LogLevel {
        INFO,
        WARN,
        ERR,
        SYS,
    };

    void Write(LogLevel level, const char* fmt, ...);

private:
    std::string GetFilePath(bool autoFilename);

private:
    FILE *m_fp;
    std::mutex m_mutex;
};

CLog* TheLog();
TimeCalc* TheTimeCal();

#define ENABLE_LOG_ERR  0
#define ENABLE_LOG_INFO 0

#ifdef WIN32
#define LOG_ALWAYS(fmt,...) TheLog()->Write(CLog::SYS, "%s<%d>: " fmt , __FUNCTION__, __LINE__, ##__VA_ARGS__) 

#if ENABLE_LOG_INFO
#define LOG_INFO(fmt,...) TheLog()->Write(CLog::INFO, "%s<%d>: " fmt , __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else //ENABLE_LOG_INFO
#define LOG_INFO(fmt,...)
#endif //ENABLE_LOG_INFO

#define LOG_WARN(fmt,...) TheLog()->Write(CLog::WARN, "%s<%d>: " fmt , __FUNCTION__, __LINE__, ##__VA_ARGS__)

#if ENABLE_LOG_ERR
#define LOG_ERR(fmt,...) TheLog()->Write(CLog::ERR, "%s<%d>: " fmt , __FUNCTION__, __LINE__, ##__VA_ARGS__) 
#else  //ENABLE_LOG_ERR
#define LOG_ERR(fmt,...)
#endif //ENABLE_LOG_ERR

#else
//linux etc

#if ENABLE_LOG_INFO
#define LOG_INFO(fmt, ...)  printf("[INF] ");printf(fmt, ##__VA_ARGS__)
#else
#define LOG_INFO(fmt,...)
#endif //ENABLE_LOG_INFO



#if ENABLE_LOG_ERR
#define LOG_ERR(fmt, ...)   printf("[ERR] ");printf(fmt, ##__VA_ARGS__)
#else  
#define LOG_ERR(fmt,...)
#endif //ENABLE_LOG_ERR

#define LOG_ALWAYS(fmt,...) printf("[SYS] ");printf(fmt, ##__VA_ARGS__)

#endif //_MSC_VER

#endif //LOG_H_


