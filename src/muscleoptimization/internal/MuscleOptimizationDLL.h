#ifdef WIN32
#   ifdef MuscleOptimization_EXPORTS
#       define MuscleOptimization_API __declspec(dllexport)
#   else
#       define MuscleOptimization_API  __declspec(dllimport)
#   endif
#else
#   define MuscleOptimization_API
#endif // WIN32