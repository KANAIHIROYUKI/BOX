#ifdef STM32F407VG
#include "config_systick.h"
#include "stm32f4xx_conf.h"



#define WEAK __attribute__ ((weak))
/******************************************************************************
*	�^�C�g�� �F ���荞�݃n���h���@�֐��ꏊ�Œ�
*	  �֐��� �F SysTick_Handler
*	  �߂�l �F void�^ 
*	   ����1 �F void�^  
*	  �쐬�� �F ������		
*	  �쐬�� �F 2014/11/10
******************************************************************************/
/*void WEAK SysTick_Handler(void){
	//�C��ms���Ɋ��荞��ŁA1�J�E���g����@�P��:[ms]
}*/

/******************************************************************************
*	�^�C�g�� �F ���荞�݃^�C�}�ݒ�֐�
*	  �֐��� �F Init_SysTick
*	  �߂�l �F void�^ �Ȃ�
*	   ����1 �F float�^ time  ���荞�ݎ�����ݒ�
*	  �쐬�� �F ������
*	  �쐬�� �F 2014/11/10
******************************************************************************/
void Init_SysTick(float time){
	//systick���荞�݂̐ݒ�	time�̒P��:[s]
	SystemCoreClockUpdate();									//SystemCoreClock�̍X�V
	NVIC_SetPriority(SysTick_IRQn,1);						//���荞�ݗD��x�̐ݒ�		�D��x��1��
	if(SysTick_Config(SystemCoreClock*time)){		//systick�ɂ�銄�荞�ݎ�����ݒ�
						//���s������@�������ݎ��R
	}
}


#endif//STM32f407VG