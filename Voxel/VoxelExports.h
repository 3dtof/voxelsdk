#if defined _WIN32 || defined __CYGWIN__
#ifdef BUILDING_DLL
#ifdef __GNUC__
#define LIB_EXPORT __attribute__ ((dllexport))
#else
#define LIB_EXPORT __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
#endif
#else
#ifdef __GNUC__
#define LIB_EXPORT __attribute__ ((dllimport))
#else
#define LIB_EXPORT __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
#endif
#endif
#define LIB_PRIVATE
#else
#if __GNUC__ >= 4
#define LIB_EXPORT __attribute__ ((visibility ("default")))
#define LIB_PRIVATE  __attribute__ ((visibility ("hidden")))
#else
#define LIB_EXPORT
#define LIB_PRIVATE
#endif
#endif