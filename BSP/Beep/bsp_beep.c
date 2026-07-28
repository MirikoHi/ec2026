#include "bsp_beep.h"
#include "board.h"

void beep_init(void)
{
}

void beep_on(void)
{
    DL_GPIO_clearPins(BEEP_PORT, BEEP_PIN_14_PIN);
}

void beep_off(void)
{
    DL_GPIO_setPins(BEEP_PORT, BEEP_PIN_14_PIN);
}
