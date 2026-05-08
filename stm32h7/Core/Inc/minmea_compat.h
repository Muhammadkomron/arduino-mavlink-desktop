/* Minimal compatibility header for minmea
 * Placed under Core/Inc as minmea_compat.h
 * Note: the build environment should define MINMEA_INCLUDE_COMPAT
 */

#ifndef MINMEA_COMPAT_H
#define MINMEA_COMPAT_H

#include <time.h>

#if defined(_MSC_VER)

#if !defined(HAVE_STRUCT_TIMESPEC)
struct timespec {
    time_t tv_sec;
    long tv_nsec;
};
#endif

#define inline __inline
#define timegm _mkgmtime

#else

/* For embedded/toolchains without timegm, fall back to mktime.
   For many embedded systems this is acceptable; adjust in build if needed. */
#ifndef timegm
#define timegm mktime
#endif

#endif

#endif /* MINMEA_COMPAT_H */
