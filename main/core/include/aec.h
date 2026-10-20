#pragma once
#include <stdint.h>

void
aec_run (int16_t *mic,
             int16_t *ref,
             int16_t *out);

void aec_init();
