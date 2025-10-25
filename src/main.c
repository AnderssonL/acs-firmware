#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

int main()
{
	LOG_INF("Hello from native_posix!");
	while (1) {
		LOG_INF("Run");
		k_msleep(10);
	}
	return 0;
}
