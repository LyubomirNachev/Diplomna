#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ot_br, LOG_LEVEL_DBG);
#define APP_BANNER "***** OpenThread NCP on Zephyr %s *****"

void main(void)
{
	LOG_INF(APP_BANNER, CONFIG_NET_SAMPLE_APPLICATION_VERSION);

}
