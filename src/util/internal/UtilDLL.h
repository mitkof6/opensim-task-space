#ifdef WIN32
#   ifdef Util_EXPORTS
#       define Util_API __declspec(dllexport)
#   else
#       define Util_API  __declspec(dllimport)
#   endif
#else
#   define Util_API
#endif // WIN32