#ifdef WIN32
#   ifdef OpenSimUtil_EXPORTS
#       define OpenSimUtil_API __declspec(dllexport)
#   else
#       define OpenSimUtil_API  __declspec(dllimport)
#   endif
#else
#   define OpenSimUtil_API
#endif // WIN32