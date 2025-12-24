#include "common.h"
#include "log.h"
#include <cctype>

#ifdef WIN32
#include <windows.h>
#endif

void cc_time_begin_period(int ms)
{
#ifdef WIN32
	timeBeginPeriod(ms);
#endif
}

void cc_time_end_period(int ms)
{
#ifdef WIN32	
	timeEndPeriod(ms);
#endif
}

bool thread_set_priority(std::thread& thread, int prio)
{
#ifdef WIN32
    BOOL bRet = SetThreadPriority(thread.native_handle(), prio);
    LOG_INFO("set thread(id:%d) prio to : %d ret:%d\n", GetThreadId(thread.native_handle()), prio, bRet);
    return bRet > 0;
#else
    return false;
#endif
}

std::string GetHexString(const unsigned char *data, size_t len)
{
    std::string tmp;
    char by[10];
    for (size_t i = 0; i < len; i++) {
        snprintf(by, sizeof(by), "%02X ", data[i]);
        tmp += by;
    }
    return tmp;
}

#ifdef WIN32
static HMODULE GetSelfModule() {
    MEMORY_BASIC_INFORMATION mbi;
    if (::VirtualQuery(GetSelfModule, &mbi, sizeof(mbi)) != 0) {
        return (HMODULE)mbi.AllocationBase;
    }
    return NULL;
}

std::string GetAppPathA() {
    char sDrive[_MAX_DRIVE]; 
    char sDir[_MAX_DIR];
    char sFilename[_MAX_FNAME], Filename[_MAX_FNAME];
    char sExt[_MAX_EXT];

    ::GetModuleFileNameA(GetSelfModule(), Filename, _MAX_FNAME);
    _splitpath_s(Filename, sDrive, _MAX_DRIVE, sDir, _MAX_DIR, sFilename, _MAX_FNAME, sExt, _MAX_EXT);

    return std::string(sDrive) + std::string(sDir); 
}
#else
std::string GetAppPathA() {
    return std::string();
}
#endif

std::string UpperString(std::string &str)
{
    std::string result = str;
    for (char &c : result) {
        c = std::toupper(c);
    }
    return result;
}

std::vector<std::string> cc_split(const std::string &str, const std::string &delim)
{
	if (str.empty()) {
		return std::vector<std::string>();
	}

	std::regex re(delim);
	if (delim == ".") {
		re = "\\.";
	}

	return std::vector<std::string>(
		std::sregex_token_iterator(str.begin(), str.end(), re, -1),
		std::sregex_token_iterator()
		);
}
