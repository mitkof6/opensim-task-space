#ifdef WIN32
#   ifdef TaskSpaceIK_EXPORTS
#       define TaskSpaceIK_API __declspec(dllexport)
#   else
#       define TaskSpaceIK_API  __declspec(dllimport)
#   endif
#else
#   define TaskSpaceIK_API
#endif // WIN32