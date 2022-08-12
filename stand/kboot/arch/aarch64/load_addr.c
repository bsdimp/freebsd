#include <sys/param.h>

#include "kboot.h"
#include "bootstrap.h"

uint64_t
kboot_get_phys_load_segment(void)
{
	return 0;
}

void
bi_loadsmap(struct preloaded_file *kfp)
{
}
