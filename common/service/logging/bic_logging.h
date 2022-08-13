#ifndef BIC_LOGGING_H
#define BIC_LOGGING_H

#include <logging/log.h>

// clang-format off

#define FOREACH_LOG_MODULE(MODULE) \
	MODULE(ADM1278_LOG)

// clang-format on

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

extern const size_t LOG_MODULE_COUNT;
extern const char *LOG_MODULE_STRING[];

extern const size_t SEVERITY_COUNT;
extern const char *SEVERITY_STRING[];

enum LOG_ENUM { FOREACH_LOG_MODULE(GENERATE_ENUM) };

#endif
