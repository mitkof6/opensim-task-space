#ifdef WIN32
#   ifdef NumberSystems_EXPORTS
#       define NumberSystems_API __declspec(dllexport)
#   else
#       define NumberSystems_API  __declspec(dllimport)
#   endif
#else
#   define NumberSystems_API
#endif // WIN32