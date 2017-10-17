#ifdef WIN32
#   ifdef TaskSpaceMC_EXPORTS
#       define TaskSpaceMC_API __declspec(dllexport)
#   else
#       define TaskSpaceMC_API  __declspec(dllimport)
#   endif
#else
#   define TaskSpaceMC_API
#endif // WIN32