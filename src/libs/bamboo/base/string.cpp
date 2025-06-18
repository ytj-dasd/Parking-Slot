#include "string.h"
#ifdef OS_WINDOWS
#include "windows.h"
#endif

namespace welkin::bamboo {
std::string StringToUTF8(const std::string& str) {
#ifdef OS_WINDOWS
    int nwLen = ::MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, 0);
    wchar_t * pwBuf = new wchar_t[nwLen + 1]; //一定要加1，不然会出现尾巴
    ZeroMemory(pwBuf, nwLen * 2 + 2);
    ::MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.length(), pwBuf, nwLen);
    int nLen = ::WideCharToMultiByte(CP_UTF8, 0, pwBuf, -1, NULL, NULL, NULL, NULL);
    char * pBuf = new char[nLen + 1];
    ZeroMemory(pBuf, nLen + 1);
    ::WideCharToMultiByte(CP_UTF8, 0, pwBuf, nwLen, pBuf, nLen, NULL, NULL);
    std::string retStr(pBuf);
    delete []pwBuf;
    delete []pBuf;
    pwBuf = NULL;
    pBuf = NULL;
    return retStr;
#else
    return str;
#endif
}
}