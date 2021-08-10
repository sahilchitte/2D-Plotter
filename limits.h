
#define limits_h


void limits_init();

void limits_disable();

uint8_t limits_get_state();

void limits_go_home(uint8_t cycle_mask);

void limits_soft_check(float *target);

#endif
