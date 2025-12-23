#ifndef LINUX_COMPAT_H
#define LINUX_COMPAT_H

#ifdef LINUX_BUILD

#include <pthread.h>
#include <unistd.h>
#include <cstring>
#include <ctime>

// Windows types to Linux types
typedef unsigned char BYTE;
typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef int BOOL;
typedef unsigned int UINT;
#define TRUE 1
#define FALSE 0

// Critical Section replacement
typedef pthread_mutex_t CRITICAL_SECTION;

inline void InitializeCriticalSection(CRITICAL_SECTION* cs) {
    pthread_mutex_init(cs, NULL);
}

inline void DeleteCriticalSection(CRITICAL_SECTION* cs) {
    pthread_mutex_destroy(cs);
}

inline void EnterCriticalSection(CRITICAL_SECTION* cs) {
    pthread_mutex_lock(cs);
}

inline void LeaveCriticalSection(CRITICAL_SECTION* cs) {
    pthread_mutex_unlock(cs);
}

// Sleep function
#define Sleep(ms) usleep((ms) * 1000)

// Byte manipulation
#define LOBYTE(w) ((unsigned char)((unsigned long)(w) & 0xff))
#define HIBYTE(w) ((unsigned char)(((unsigned long)(w) >> 8) & 0xff))

#endif // LINUX_BUILD

#endif // LINUX_COMPAT_H
