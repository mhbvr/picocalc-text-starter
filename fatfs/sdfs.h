#pragma once
#include "ff.h"
#include <stdbool.h>
extern FATFS sdfs_volume;
void sdfs_init(void);
bool sdfs_is_ready(void);
