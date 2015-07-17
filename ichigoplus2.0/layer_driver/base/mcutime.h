#ifndef MCUTIME_H
#define MCUTIME_H
/*******************************************
 * mcutime ver1.0 2015/3/3
 * Handle time for stm32f4 discovery 2-layer board ver 3.5.
 *
 * [Dependency]
 * config_systick
 * stdint
 *
 * [Note]
 * Run timer_setup() at mcu wake up to use functions.
 *
 * [Change history]
 * ver2.0 2015/ 3/ 6 Wait inline.Rename time to mcutime.Time handle by int64_t.
 * ver1.0 2015/ 3/ 3 The first version.
 ******************************************/
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

 int64_t millis();
 int64_t micros();
 void delay_ms(int64_t time);
 void timer_setup();
 void wait(int64_t wait_time);


#ifdef __cplusplus
}
#endif

#endif
