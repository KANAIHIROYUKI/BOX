#include "software_reset.h"
#include "iwdg_software_reset.h"
#ifdef STM32F407VG

void software_reset(){
	soft_reset_durch();
}
#endif//STM32F407VG
