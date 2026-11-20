#pragma once
#include <stdint.h>

void
aec_run (int16_t *mic,
             int16_t *ref,
             int16_t *out,
             int16_t *nohdres);

void aec_init();
