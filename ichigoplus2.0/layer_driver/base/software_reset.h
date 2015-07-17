#ifndef SOFTWARE_RESET_H
#define SOFTWARE_RESET_H
/*******************************************
 * sd ver1.0 2015/3/3
 * Functions of software reset for stm32f4 discovery 2-layer board ver 3.5.
 *
 * [Dependency]
 * iwdg_software_reset
 *
 * [Note]
 *
 * [Change history]
 * ver1.0 2015/ 3/ 3 The first version.
 ******************************************/
#ifdef __cplusplus
 extern "C" {
#endif

 void software_reset();

#ifdef __cplusplus
}
#endif

#endif//SOFTWARE_RESET_H
