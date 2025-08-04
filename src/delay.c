#include "delay.h"

static uint32_t reload_val = 0;
static uint32_t ticks_per_ms = 0;

void delay_init(uint8_t freq_mhz)
{
	/* check csr clocksource and set ticks_per_ms accordingly: */
	if ((STK_CSR & STK_CSR_CLKSOURCE) == STK_CSR_CLKSOURCE_AHB)
	{
		ticks_per_ms = freq_mhz * 1000U; /* xxxMHz = xxxM ticks/s -> divide by 1000 to get ticks/ms */
	}
	else /* == STK_CSR_CLKSOURCE_AHB_DIV8 */
	{
		ticks_per_ms = freq_mhz * 1000U / 8U;
	}

	/* get reload value or set own one if 0: */
	reload_val = systick_get_reload();
	if (reload_val == 0)
	{
		systick_set_reload(ticks_per_ms);
		reload_val = ticks_per_ms;
	}

	/* enable systick and set init_done_flag: */
	systick_counter_enable();
}

void delay_ms(uint32_t milliseconds)
{
	uint32_t num_of_reloads = 0;

	while ((num_of_reloads * reload_val) < (milliseconds * ticks_per_ms))
	{
		if (systick_get_countflag()) /* flag gets automatically cleared when read */
		{
			num_of_reloads++;
		}
	}
}
