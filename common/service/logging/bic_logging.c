#include "bic_logging.h"

const char *LOG_MODULE_STRING[] = { FOREACH_LOG_MODULE(GENERATE_STRING) };
const size_t LOG_MODULE_COUNT = ARRAY_SIZE(LOG_MODULE_STRING);

// clang-format off
const char *SEVERITY_STRING[] = {
	"DEBUG",
	"INFO",
	"WARNING",
	"ERROR",
	"OFF",
};
// clang-format on
const size_t SEVERITY_COUNT = ARRAY_SIZE(SEVERITY_STRING);
