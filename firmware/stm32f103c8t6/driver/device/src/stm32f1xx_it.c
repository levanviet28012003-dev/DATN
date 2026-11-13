#include <stdio.h>
#include <stdint.h>

#include "stm32f1xx_it.h"

/* Reset_Handler được định nghĩa trong startup, không ghi đè bằng vòng lặp C. */

void NMI_Handler(void){
    while(1){

    }
}

void HardFault_Handler(void){
    while(1){

    }
}

void MemManage_Handler(void){
    while(1){

    }
}

void BusFault_Handler(void){
    while(1){

    }
}

void UsageFault_Handler(void){
    while(1){

    }
}

void DebugMon_Handler(void){
    while(1){

    }
}

