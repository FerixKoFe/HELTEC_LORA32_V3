#pragma once

#include <stdbool.h>

void user_button_init(void);
bool user_button_poll_pressed(void);
void user_button_sync(void);
