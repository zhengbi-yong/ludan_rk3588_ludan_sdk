#include "log.h"
#include <string>
#include "common.h"

CLog* TheLog()
{
    static CLog _inst;
    return &_inst;
}

TimeCalc* TheTimeCal()
{
    static TimeCalc _inst;
    return &_inst;
}

CLog::CLog()
{
    std::string filepath = GetFilePath(false);
    //使用 "r+"方式打开日志，日志文件不存在则打开失败，此时不保存到文件
    //需要查看日志的时候，在dll同级目录下建立文件，就会写入日志
    //使用 "a+"方式打开日志，日志文件不存在则创建文件，文件存在则在尾部添加新数据
    m_fp = fopen(filepath.c_str(), "r+");

    if (!m_fp) {
        Write(SYS, "Log file[%s] open failed!", filepath.c_str());
    }
    else {
        Write(SYS, "Write to file[%s]", filepath.c_str());
        Write(SYS, "================= Logging =================");
    }
}

CLog::~CLog()
{
    if (m_fp) {
        fflush(m_fp);
        fclose(m_fp);
    }
}

std::string CLog::GetFilePath(bool autoFilename)
{
#ifdef WIN32
    if (!autoFilename) {
        return GetAppPathA() + "\\CANFDNET.log";
    }

    char buftime[100] = { 0 };
    SYSTEMTIME st;
    GetLocalTime(&st);
    sprintf(buftime, "%04d%02d%02d_%02d%02d%02d",
        st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    return GetAppPathA() + "\\CANFDNET_" + std::string(buftime) + ".log";
#else
	return "";
#endif
}

void CLog::Write(LogLevel level, const char* fmt, ...)
{
#ifdef WIN32
    std::unique_lock<std::mutex> lk(m_mutex);

    char buftime[100] = { 0 };
    char buf[2048] = { 0 };
    SYSTEMTIME st;
    GetLocalTime(&st);
    sprintf(buftime, "[%04d-%02d-%02d %02d:%02d:%02d.%03d] ",
        st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

    va_list args;
    va_start(args, fmt);
    _vsnprintf_s(buf, sizeof(buf) - 4, fmt, args);
    buf[sizeof(buf) - 1] = 0;         // 如果信息太长最后显示...
    buf[sizeof(buf) - 2] = '.';
    buf[sizeof(buf) - 3] = '.';
    buf[sizeof(buf) - 4] = '.';
    va_end(args);

    const char* levelStr = "";
    switch (level)
    {
    case INFO: levelStr = "[INF] "; break;
    case WARN: levelStr = "[WRN] "; break;
    case ERR: levelStr = "[ERR] "; break;
    case SYS: levelStr = "[SYS] "; break;
    }

    //打印到调试
    if (ERR == level || SYS == level) {
        OutputDebugStringA(std::string(std::string(buftime) + std::string(levelStr) + std::string(buf)).c_str());
    }

    //打印到文件
    if (!m_fp) {
        return;
    }

    // 限制文件大小
    const long limit = 50 * 1000 * 1000;
    fseek(m_fp, 0, SEEK_END);
    long len = ftell(m_fp);
    if (len > limit) {
        fclose(m_fp);
        m_fp = fopen(GetFilePath(true).c_str(), "w");
        if (!m_fp) {
            OutputDebugStringA("[ERR] Clear log file failed! \n");
        }
    }

    fprintf(m_fp, buftime);
    fprintf(m_fp, levelStr);
    fprintf(m_fp, buf);

    if (strlen(buf) > 0 && buf[strlen(buf) - 1] != '\n') {
        fprintf(m_fp, "\n");
    }

    fflush(m_fp);
#endif
}

